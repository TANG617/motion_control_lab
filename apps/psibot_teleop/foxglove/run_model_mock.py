#!/usr/bin/env python3
"""Run the installed Cortex Mock and serve its exact URDF to Foxglove."""
import argparse
import datetime
import functools
import hashlib
import http.server
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading

MODEL = Path('/workspace/models/Psi_R1_visual_collision.urdf')
# App-local snapshot of hierarchical_kinematics_step RobotOptions.default_positions.
MCL_INITIAL = [0.0, 0.31, 0.0, 0.5, 0.5, -0.5, 0.9, -1.38, -1.57, -1.4,
               -0.45, 0.0, 0.0, -0.9, 1.38, 1.57, 1.4, 0.45, 0.0, 0.0]


class ModelHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache')
        super().end_headers()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, default=Path('/workspace/install/psi-cortex-mock/psi_cortex'))
    parser.add_argument('--output-dir', type=Path, default=Path('/workspace/runs/psibot_teleop_model') / datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    parser.add_argument('--port', type=int, default=6060)
    parser.add_argument('--viz-port', type=int, default=8765)
    parser.add_argument('--model-port', type=int, default=8766)
    parser.add_argument('--duration', type=float, default=0)
    args = parser.parse_args()
    prefix = args.prefix.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(prefix / 'share/psi_cortex/config', output / 'config')
    # Cortex derives the collision mesh root three directories above the URDF.
    # Keep its expected package shape with symlinks to the original files.
    (output / 'model/urdf').mkdir(parents=True)
    model_path = output / 'model/urdf' / MODEL.name
    model_path.symlink_to(MODEL)
    (output / 'meshes').symlink_to(MODEL.parent / 'meshes', target_is_directory=True)
    motion_path = output / 'config/motion_config.json'
    motion = json.loads(motion_path.read_text())
    motion['robot']['urdf'] = str(model_path)
    motion_path.write_text(json.dumps(motion, indent=2) + '\n')
    preset_path = output / 'config/psi_r1/preset_poses.json'
    preset = json.loads(preset_path.read_text())
    preset['poses']['mcl_initial'] = {'positions': MCL_INITIAL}
    preset_path.write_text(json.dumps(preset, indent=2) + '\n')
    runtime = dict(motion_config=str(motion_path), backend='mock', initial_pose='mcl_initial',
                   bind='127.0.0.1', tcp_port=args.port, ordinary_scheduling=True,
                   output_dir=str(output), duration_sec=args.duration, strict_recording=True,
                   visualization=dict(websocket=True, host='127.0.0.1', port=args.viz_port,
                                      scene_rate_hz=50, mcap=''))
    runtime_path = output / 'runtime_config.json'
    runtime_path.write_text(json.dumps(runtime, indent=2) + '\n')
    url = f'http://127.0.0.1:{args.model_port}/{MODEL.name}'
    (output / 'model.json').write_text(json.dumps(dict(urdf=str(MODEL), cortex_urdf=str(model_path), url=url,
        initial_pose='mcl_initial', initial_positions=MCL_INITIAL,
        sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest()), indent=2) + '\n')
    env = os.environ.copy()
    env['AMENT_PREFIX_PATH'] = str(prefix) + os.pathsep + env.get('AMENT_PREFIX_PATH', '')
    env['LD_LIBRARY_PATH'] = str(prefix / 'lib') + os.pathsep + env.get('LD_LIBRARY_PATH', '')
    handler = functools.partial(ModelHandler, directory=str(MODEL.parent))
    with http.server.ThreadingHTTPServer(('127.0.0.1', args.model_port), handler) as server:
        threading.Thread(target=server.serve_forever, daemon=True).start()
        with (output / 'runtime.log').open('w') as log:
            process = subprocess.Popen([str(prefix / 'lib/psi_cortex/psi_cortex'), '--ros-args',
                '-p', f'runtime_config:={runtime_path}'], cwd=output, env=env,
                stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            def stop(signum, frame):
                process.send_signal(signal.SIGINT)
            signal.signal(signal.SIGINT, stop)
            signal.signal(signal.SIGTERM, stop)
            print(f'Mock TCP: 127.0.0.1:{args.port}\nFoxglove: ws://127.0.0.1:{args.viz_port}\nModel: {url}\nLogs: {output}', flush=True)
            try:
                return process.wait()
            finally:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=15)
                server.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
