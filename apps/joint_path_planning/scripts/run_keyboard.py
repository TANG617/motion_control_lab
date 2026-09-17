#!/usr/bin/env python3
"""Launch the installed app and a local, CORS-enabled R1 asset server. Never builds."""
import argparse
import functools
import hashlib
import http.server
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import xml.etree.ElementTree as ET


class AssetHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def log_message(self, *_args):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--urdf", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--asset-host", default="127.0.0.1")
    parser.add_argument("--asset-port", type=int, default=8766)
    parser.add_argument("--asset-url-host", default="127.0.0.1")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--serve-only", action="store_true", help="Serve model assets for an existing MCAP recording; do not start the app")
    args, extra = parser.parse_known_args()
    prefix = Path(os.environ.get("MCL_INSTALL_PREFIX", "/workspace/install/algorithm"))
    binary = Path(os.environ.get("MCL_BINARY", str(prefix / "bin/mcl_joint_path_planning")))
    share = prefix / "share/motion-control-lab"
    source = Path(__file__).resolve().parents[1]
    app = source if (source / "configs").is_dir() else share / "apps/joint_path_planning"
    urdf = (args.urdf or Path("/workspace/models/Psi_R1_visual_collision.urdf")).resolve()
    config = (args.config or app / "configs/cube.json").resolve()
    output = (args.output or Path("build/joint_path_planning/runs") / str(time.time_ns())).resolve()
    command = [str(binary), "--config", str(config), "--urdf", str(urdf), "--output", str(output / "run"), "--launcher", "run_keyboard.py", *extra]
    if args.dry_run:
        print(json.dumps(command))
        return 0
    output.mkdir(parents=True, exist_ok=False)
    # Preserve relative mesh locations; the checked-in R1 has urdf/ and meshes/ siblings.
    resource_root = urdf.parent
    url = f"http://{args.asset_url_host}:{args.asset_port}/{urdf.relative_to(resource_root).as_posix()}"
    layout = json.loads((app / "foxglove/joint_path_planning.layout.json").read_text())
    for layer in layout["configById"]["3D!jointpath"]["layers"].values():
        if layer["layerId"] == "foxglove.Urdf":
            layer["url"] = url
    (output / "foxglove.layout.json").write_text(json.dumps(layout, indent=2) + "\n")
    manifest = {"command": command, "urdf_url": url, "resource_root": str(resource_root),
                "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
                "mesh_sha256": {name: hashlib.sha256((urdf.parent / name).read_bytes()).hexdigest()
                                for name in sorted({m.attrib["filename"] for m in ET.parse(urdf).iter("mesh")})}}
    (output / "launcher.json").write_text(json.dumps(manifest, indent=2) + "\n")
    handler = functools.partial(AssetHandler, directory=str(resource_root))
    with http.server.ThreadingHTTPServer((args.asset_host, args.asset_port), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(f"Foxglove layout: {output / 'foxglove.layout.json'}\nModel: {url}\nWebSocket: app --host/--port (default ws://127.0.0.1:8765)", flush=True)
        if args.serve_only:
            try:
                thread.join()
            except KeyboardInterrupt:
                server.shutdown()
                thread.join()
            return 0
        process = subprocess.Popen(command)
        try:
            try:
                return process.wait()
            except KeyboardInterrupt:
                return process.wait(timeout=15)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=15)
            server.shutdown()
            thread.join()


if __name__ == "__main__":
    raise SystemExit(main())
