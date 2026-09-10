#!/usr/bin/env python3
"""Exercise the real SDK and terminal against a loopback-only protocol fixture."""
import contextlib
import fcntl
import json
import os
from pathlib import Path
import pty
import re
import select
import signal
import socketserver
import struct
import subprocess
import sys
import tempfile
import termios
import threading
import time

HEADER = struct.Struct('!IHHHHIII')
NAMES = ['head_yaw_joint', 'head_pitch_joint', 'torso_yaw_joint', 'torso_pitch_joint',
         'knee_pitch_joint', 'ankle_pitch_joint'] + [f'{side}_arm_joint{i}' for side in ['left', 'right'] for i in range(1, 8)]


def wait_for(check, timeout=8):
    end = time.monotonic() + timeout
    while not check():
        if time.monotonic() >= end:
            raise AssertionError('Timed out waiting for test condition')
        time.sleep(.01)


class ProtocolServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, failure=None, profile=3):
        self.calls = []
        self.failure = failure
        self.enabled = {part: False for part in range(5)}
        self.profile = {part: profile for part in range(5)}
        self.poses = [[.4, .2, .5, 0, 0, 0, 1], [.4, -.2, .5, 0, 0, 0, 1]]
        super().__init__(('127.0.0.1', 0), ProtocolHandler)
        self.thread = threading.Thread(target=self.serve_forever, daemon=True)
        self.thread.start()

    def count(self, name):
        return sum(call['msg_name'] == name for call in self.calls)

    def close(self):
        self.shutdown()
        self.server_close()
        self.thread.join()


class ProtocolHandler(socketserver.BaseRequestHandler):
    def exact(self, size):
        data = bytearray()
        while len(data) < size:
            part = self.request.recv(size - len(data))
            if not part:
                raise EOFError
            data.extend(part)
        return bytes(data)

    def handle(self):
        try:
            while True:
                header = HEADER.unpack(self.exact(24))
                request = json.loads(self.exact(header[6]))
                server = self.server
                server.calls.append(request)
                name = request['msg_name']
                part = request.get('component', -1)
                params = request.get('parameters', {})
                result = dict(error_code=0, error_msg='ok')
                if name == 'get_joint_states':
                    result.update(stamp_us=time.time_ns()//1000, names=NAMES, positions=[0.]*20,
                                  velocities=[0.]*20, accelerations=[0.]*20, torques=[0.]*20)
                elif name == 'get_end_pose':
                    if server.failure == 'disconnect' and server.count(name) > 3:
                        return
                    if server.failure == 'timeout' and server.count(name) > 3:
                        continue  # SDK must expose its own 1 second RPC timeout.
                    result.update(stamp_us=time.time_ns()//1000, pose_frame=1,
                                  left_pose=server.poses[0], right_pose=server.poses[1])
                elif name == 'get_robot_limits':
                    result.update(joints=[dict(name=n, position_lower=-3., position_upper=3., max_velocity=1.) for n in NAMES], end_poses=[])
                elif name == 'get_enable':
                    result['enabled'] = server.enabled[part]
                elif name == 'get_profile':
                    result['profile'] = server.profile[part]
                elif name == 'set_enable':
                    server.enabled[part] = params['enable']
                    result['enable'] = params['enable']
                elif name == 'set_profile':
                    server.profile[part] = params['profile']
                    result['profile'] = params['profile']
                elif name in ('move_end_pose', 'move_wbc_pose', 'move_joint'):
                    if server.failure == 'reject':
                        result.update(error_code=-100, error_msg='fixture rejection')
                    else:
                        for side, key in enumerate(['left_pose', 'right_pose']):
                            if params.get(key):
                                server.poses[side] = params[key]
                        if server.failure == 'slow':
                            time.sleep(.7)
                elif name not in ('start_wbc', 'stop_wbc', 'hand_shake'):
                    raise AssertionError(f'Unexpected API: {name}')
                response = dict(request, msg_id=str(int(request['msg_id'])+1), need_rsp=False, parameters=result)
                response['from'] = 'robot'
                response['to'] = request.get('from', '')
                payload = json.dumps(response).encode()
                self.request.sendall(HEADER.pack(0x50534952, 1, 24, 0x0200, 1, header[5], len(payload), 0) + payload)
        except (EOFError, ConnectionError, OSError):
            pass


class TerminalApp:
    def __init__(self, binary, address, output, width=100, height=30, extra=()):
        self.master, self.slave = pty.openpty()
        self.original = termios.tcgetattr(self.slave)
        self.resize(width, height)
        self.output = bytearray()
        self.log_dir = Path(output)

        def setup():
            os.setsid()
            fcntl.ioctl(self.slave, termios.TIOCSCTTY, 0)

        env = dict(os.environ, TERM='xterm-256color')
        self.process = subprocess.Popen([str(binary), '--address', address, '--log-dir', str(output), *extra],
            stdin=self.slave, stdout=self.slave, stderr=self.slave, env=env, preexec_fn=setup)

    def resize(self, width, height):
        fcntl.ioctl(self.slave, termios.TIOCSWINSZ, struct.pack('HHHH', height, width, 0, 0))

    def pump(self, duration=.1):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            ready, _, _ = select.select([self.master], [], [], .02)
            if ready:
                try:
                    self.output.extend(os.read(self.master, 65536))
                except OSError:
                    return

    def frame(self):
        self.pump(.1)
        # FTXUI writes a full frame after cursor-home on every redraw.
        raw = bytes(self.output).rsplit(b'\x1b[H', 1)[-1].decode(errors='replace')
        return re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', raw).replace('\r', '')

    def key(self, value):
        os.write(self.master, value)
        self.pump(.12)

    def logged(self, text):
        self.pump(.02)
        log = self.log_dir/'sdk.log'
        return log.exists() and text in log.read_text(errors='replace')

    def ready(self):
        wait_for(lambda: self.logged('Connected to Psi; observation only'))
        self.pump()

    def finish(self, expected=0):
        wait_for(lambda: (self.pump(.02) or self.process.poll() is not None), timeout=12)
        self.pump()
        assert self.process.returncode == expected, self.output.decode(errors='replace')[-4000:]
        assert termios.tcgetattr(self.slave) == self.original, 'termios not restored'
        assert b'\x1b[?1049l' in self.output, 'alternate screen not restored'
        assert b'rpc getJointStates' not in self.output, 'SDK log polluted the TUI'
        assert b'[info]' not in self.output, 'SDK stdout leaked into terminal'

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait()
        os.close(self.master)
        os.close(self.slave)


def successful_flow(binary, folder):
    with contextlib.closing(ProtocolServer()) as server:
        address = f'127.0.0.1:{server.server_address[1]}'
        with contextlib.closing(TerminalApp(binary, address, folder/'success')) as app:
            app.ready()
            assert not server.count('set_enable') and not server.count('set_profile')
            app.key(b'm')
            app.key(b'\x1b')  # Cancel step editing, not the app.
            assert app.process.poll() is None
            app.key(b'w')
            assert not server.count('move_end_pose')
            app.key(b' ')
            wait_for(lambda: app.logged('Sending enabled'))
            app.key(b'w')
            wait_for(lambda: server.count('move_end_pose') == 1)
            call = next(call for call in server.calls if call['msg_name'] == 'move_end_pose')
            assert call['component'] == 1 and abs(call['parameters']['left_pose'][0] - .405) < 1e-9
            app.key(b' ')
            app.key(b'w')
            assert server.count('move_end_pose') == 1
            # Select mode via real menu input; no implicit WBC start.
            app.key(b'c')
            for width, height in [(80, 24), (60, 24), (48, 12)]:
                app.resize(width, height)
                frame = app.frame()
                assert all(label in frame for label in ['Joint control', 'Single-arm Cartesian', 'Dual-arm WBC'])
                assert 'Enter select' in frame and 'Esc cancel' in frame
                assert all(label not in frame for label in ['End app WBC', 'Enable selected', 'Disable selected', 'Set JointPosition'])
            app.resize(100, 30)
            app.key(b'\x1b[B')
            app.key(b'\r')
            assert not server.count('start_wbc')
            app.key(b' ')
            wait_for(lambda: server.count('start_wbc') == 1)
            app.key(b'q')
            wait_for(lambda: server.count('move_wbc_pose') == 1)
            call = next(call for call in server.calls if call['msg_name'] == 'move_wbc_pose')
            assert call['parameters']['left_pose'] and call['parameters']['right_pose'] is None
            for width in [180, 100, 80, 60, 47]:
                app.resize(width, 24 if width != 47 else 11)
                for page in [b'1', b'2', b'3', b'4', b'5']:
                    app.key(page)
            assert b'Terminal too small' in app.output
            app.resize(80, 24)
            app.key(b'h')
            app.key(b'c')  # A menu must be visible even over help / a scrolled page.
            assert b'Control mode |' in app.output
            app.key(b'\x1b')
            app.key(b'h')
            app.key(b'x')
            app.finish()
            assert server.count('stop_wbc') == 1
            assert b'Base / xyzw' in app.output and b'Client RPC' in app.output


def failures(binary, folder):
    for failure in ['reject', 'timeout', 'disconnect', 'slow']:
        with contextlib.closing(ProtocolServer(failure)) as server:
            with contextlib.closing(TerminalApp(binary, f'127.0.0.1:{server.server_address[1]}', folder/failure)) as app:
                app.ready()
                if failure in ('reject', 'slow'):
                    app.key(b' ')
                    app.key(b'w')
                    wait_for(lambda: server.count('move_end_pose') == 1)
                if failure == 'slow':
                    # The menu must render while the SDK is waiting for the response.
                    app.key(b'c')
                    assert b'Control mode |' in app.output
                    app.key(b'\x03')
                    app.finish()
                else:
                    app.finish(expected=1)
                    assert b'SDK error' in app.output or b'disconnected' in app.output
                    assert server.count('move_end_pose') <= 1


def joint_flow(binary, folder):
    with contextlib.closing(ProtocolServer(profile=1)) as server:
        with contextlib.closing(TerminalApp(binary, f'127.0.0.1:{server.server_address[1]}', folder/'joints', width=60, height=24)) as app:
            app.ready()
            app.key(b'w')
            assert server.count('move_joint') == 0
            app.key(b' ')
            wait_for(lambda: app.logged('Sending enabled'))
            app.key(b'\x1b[C')  # J2; does not switch arms or pause.
            app.key(b'w')
            wait_for(lambda: server.count('move_joint') == 1)
            call = next(c for c in server.calls if c['msg_name'] == 'move_joint')
            assert call['component'] == 1 and len(call['parameters']['positions']) == 7
            assert abs(call['parameters']['positions'][1] - 3.141592653589793/180) < 1e-12
            assert call['parameters']['positions'][0] == 0
            app.key(b'q')
            assert server.count('move_end_pose') == 0
            app.key(b'2')
            assert b'Edited' in app.output and b'Joint |' in app.output
            app.key(b'c')
            app.key(b'\x1b[B')
            app.key(b'\r')  # Switch to CartesianPosition, paused.
            wait_for(lambda: server.profile[1] == 3)
            app.key(b' ')
            app.key(b'w')
            wait_for(lambda: server.count('move_end_pose') == 1)
            assert server.count('move_joint') == 1
            app.key(b'c')
            app.key(b'\x1b[B')  # Cartesian -> WBC.
            app.key(b'\r')
            assert server.count('start_wbc') == 0
            app.key(b' ')
            app.key(b'q')
            wait_for(lambda: server.count('move_wbc_pose') == 1)
            app.key(b'c')
            app.key(b'\x1b[B')  # WBC -> Joint.
            app.key(b'\x1b[D')  # left arm -> head.
            app.key(b'\r')
            wait_for(lambda: server.profile[4] == 1)
            names = [c['msg_name'] for c in server.calls]
            assert names.index('stop_wbc') < max(i for i, n in enumerate(names) if n == 'set_profile')
            assert not server.count('set_enable')
            assert server.count('move_joint') == 1  # Selecting a mode does not move.
            app.key(b' ')
            app.key(b'w')
            wait_for(lambda: server.count('move_joint') == 2)
            call = [c for c in server.calls if c['msg_name'] == 'move_joint'][-1]
            assert call['component'] == 4 and len(call['parameters']['positions']) == 2
            app.key(b'x')
            app.finish()
            assert server.count('stop_wbc') == 1  # No duplicate session cleanup.


def plain_and_cli(binary, folder):
    with contextlib.closing(ProtocolServer()) as server:
        result = subprocess.run([str(binary), '--ui', 'none', '--duration', '.4',
            '--address', f'127.0.0.1:{server.server_address[1]}', '--log-dir', str(folder/'plain')],
            capture_output=True, text=True, timeout=8)
        assert result.returncode == 0, result.stderr
        assert '\x1b' not in result.stdout and 'connected=' in result.stdout
        assert not any(call['msg_name'].startswith(('move_', 'set_enable', 'set_profile', 'start_wbc')) for call in server.calls)
        count = len(server.calls)
        result = subprocess.run([str(binary), '--address', f'127.0.0.1:{server.server_address[1]}',
            '--log-dir', str(folder/'not-a-tty')], capture_output=True, text=True, timeout=8)
        assert result.returncode == 1 and 'interactive TTY' in result.stderr
        assert len(server.calls) == count, 'non-TTY failure connected to the server'
    # Launcher forwards explicit arguments after presets and respects MCL_BINARY.
    fake = folder/'argv-probe'
    fake.write_text('#!/usr/bin/env python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
    fake.chmod(0o755)
    launcher = Path(__file__).resolve().parents[1]/'scripts/run_keyboard.sh'
    env = dict(os.environ, MCL_BINARY=str(fake), MCL_PSIBOT_MODE='single')
    out = subprocess.check_output([str(launcher), '--mode', 'wbc'], env=env, text=True)
    assert json.loads(out)[-2:] == ['--mode', 'wbc']


if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='psibot-teleop-pty-') as root:
        folder = Path(root)
        successful_flow(Path(sys.argv[1]), folder)
        joint_flow(Path(sys.argv[1]), folder)
        failures(Path(sys.argv[1]), folder)
        plain_and_cli(Path(sys.argv[1]), folder)
    print('Real SDK loopback PTY: motion routing, modes, resize, logs, rejection, timeout, disconnect and Ctrl-C passed')
