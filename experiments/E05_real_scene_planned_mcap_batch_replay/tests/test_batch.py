import importlib.util
import json
import os
import pty
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import termios
import time
import unittest

E05 = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("e05_batch", E05 / "run_batch.py")
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)


class BatchTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.data = self.root / "data"
        self.data.mkdir()
        self.output = self.root / "runs"
        self.environment = dict(os.environ, MCL_BINARY=str(E05 / "tests/fake_replay.py"),
                                MCL_CPU_SET="", MCL_RT_PRIORITY="")

    def files(self, *names):
        for name in names:
            path = self.data / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(name.encode())

    def command(self, *arguments):
        return [sys.executable, str(E05 / "run_batch.py"), "--dataset-dir", str(self.data),
                "--output-root", str(self.output), "--run-id", "test", *arguments]

    def run_batch(self, *arguments):
        return subprocess.run(self.command(*arguments), env=self.environment,
                              capture_output=True, timeout=20)

    def manifest(self):
        return json.loads((self.output / "test/manifest.json").read_text())

    def test_filter_sort_recursive_and_limit(self):
        self.files("z.mcap", "a.mcap", "b.mcap", "nested/a.mcap", "ignore.txt")
        options, extra = batch.parse_args(["--dataset-dir", str(self.data), "--recursive",
                                          "--include", "*.mcap", "--exclude", "b*", "--limit", "2"])
        self.assertEqual([p.relative_to(self.data).as_posix() for p in batch.discover(options)], ["a.mcap", "nested/a.mcap"])
        self.assertEqual(extra, [])
        options.recursive = False
        self.assertEqual([p.name for p in batch.discover(options)], ["a.mcap", "z.mcap"])

    def test_dry_run_no_artifacts_and_forwarding(self):
        self.files("a.mcap")
        result = self.run_batch("--dry-run", "--", "--red-rate", "321")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(b"--red-rate 321", result.stdout)
        self.assertFalse(self.output.exists())

    def test_reserved_arguments_and_abbreviations(self):
        self.files("a.mcap")
        for flag in ("--input", "--output-dir=bad", "--out", "--mcap", "--dump-resolved-options"):
            result = self.run_batch("--", flag)
            self.assertEqual(result.returncode, 2)
        self.assertFalse(self.output.exists())

    def test_failure_then_success_and_original_errors(self):
        self.files("a_failed.mcap", "b_ok.mcap", "c_missing.mcap", "d_ok.mcap")
        result = self.run_batch("--", "--red-rate", "321")
        self.assertEqual(result.returncode, 1, result.stderr)
        manifest = self.manifest()
        rows = manifest["units"]
        self.assertEqual([r["state"] for r in rows], ["failed", "succeeded", "missing_result", "succeeded"])
        self.assertIn("ORIGINAL SOLVER FAILURE", rows[0]["error"])
        self.assertEqual(rows[1]["red_solver_p95_ms"], 0.2)
        self.assertIsNone(rows[2]["input_sha256"])
        self.assertEqual(manifest["failures"]["total"], 2)
        self.assertTrue(manifest["outputs"])
        self.assertEqual(len({r["artifact_dir"] for r in rows}), 4)
        command = json.loads((self.output / "test" / rows[1]["artifact_dir"] / "command.json").read_text())
        self.assertEqual(command[command.index("--red-rate") + 1], "321")

    def test_success_and_manifest_contract(self):
        self.files("a_ok.mcap", "b_ok.mcap")
        result = self.run_batch()
        self.assertEqual(result.returncode, 0, result.stderr)
        validate = subprocess.run([sys.executable, str(E05.parents[1] / "tests/validate_contracts.py"),
                                   "manifest", str(self.output / "test/manifest.json")], capture_output=True)
        self.assertEqual(validate.returncode, 0, validate.stderr)

    def test_zero_exit_missing_artifact_is_failure(self):
        self.files("a_noartifact.mcap", "b_ok.mcap")
        result = self.run_batch()
        self.assertEqual(result.returncode, 1)
        rows = self.manifest()["units"]
        self.assertEqual(rows[0]["state"], "failed")
        self.assertEqual(rows[0]["native_state"], "succeeded")
        self.assertEqual(rows[1]["state"], "succeeded")

    def test_stopped_zero_exit_does_not_start_next(self):
        self.files("a_stopped.mcap", "b_ok.mcap")
        result = self.run_batch()
        self.assertEqual(result.returncode, 130)
        self.assertEqual([r["state"] for r in self.manifest()["units"]], ["stopped", "not_run"])

    def test_append_only_and_empty_selection(self):
        empty = self.run_batch()
        self.assertNotEqual(empty.returncode, 0)
        self.assertFalse(self.output.exists())
        self.files("a_ok.mcap")
        self.assertEqual(self.run_batch().returncode, 0)
        previous = (self.output / "test/manifest.json").read_bytes()
        self.assertNotEqual(self.run_batch().returncode, 0)
        self.assertEqual((self.output / "test/manifest.json").read_bytes(), previous)

    def test_signal_graceful_shutdown_preserves_artifacts(self):
        self.files("a_wait.mcap", "b_ok.mcap")
        process = subprocess.Popen(self.command(), env=self.environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   start_new_session=True)
        self.addCleanup(lambda: process.kill() if process.poll() is None else None)
        output = b""
        deadline = time.monotonic() + 10
        while b"READY FOR STOP" not in output and time.monotonic() < deadline:
            if select.select([process.stdout], [], [], 0.1)[0]:
                output += os.read(process.stdout.fileno(), 65536)
        self.assertIn(b"READY FOR STOP", output)
        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=15)
        self.assertEqual(process.returncode, 130, stderr)
        rows = self.manifest()["units"]
        self.assertEqual([r["state"] for r in rows], ["interrupted", "not_run"])
        item = self.output / "test" / rows[0]["artifact_dir"]
        self.assertTrue((item / "replay/status.json").exists())
        with self.assertRaises(ProcessLookupError):
            os.kill(int((item / "child.pid").read_text()), 0)

    def test_outer_terminal_forwarding_resize_and_restore(self):
        self.files("a_wait.mcap", "b_ok.mcap")
        master, slave = pty.openpty()
        before = termios.tcgetattr(slave)
        termios.tcsetwinsize(slave, (48, 160))
        process = subprocess.Popen(self.command(), env=self.environment, stdin=slave,
                                   stdout=slave, stderr=slave, start_new_session=True)
        try:
            output = b""
            deadline = time.monotonic() + 10
            while b"READY FOR STOP" not in output and time.monotonic() < deadline:
                if select.select([master], [], [], 0.1)[0]:
                    output += os.read(master, 65536)
            self.assertIn(b"READY FOR STOP", output)
            termios.tcsetwinsize(slave, (61, 170))
            time.sleep(0.15)
            os.write(master, b"w")
            window = self.output / "test/items/001_a_wait/window.json"
            deadline = time.monotonic() + 5
            while not window.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertEqual(json.loads(window.read_text()), [61, 170])
            os.write(master, b"x")
            self.assertEqual(process.wait(timeout=10), 130)
            self.assertEqual(termios.tcgetattr(slave), before)
            self.assertEqual([r["state"] for r in self.manifest()["units"]], ["stopped", "not_run"])
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
