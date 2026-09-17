#!/usr/bin/env python3
"""A late Foxglove subscriber and reconnect receive the unchanged static scene.

Uses the RFC 6455 and Foxglove v1 framing directly, without runtime dependencies.
"""
import argparse
import base64
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pty
import select
import socket
import struct
import subprocess
import tempfile
import termios
import time

p = argparse.ArgumentParser()
p.add_argument('--binary', required=True)
p.add_argument('--urdf', required=True)
p.add_argument('--app', type=Path, required=True)
p.add_argument('--output', type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
out = Path(tempfile.mkdtemp(prefix='subscription-', dir=a.output)) / 'run'
with socket.socket() as probe:
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
master, slave = pty.openpty()
fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', 30, 110, 0, 0))
before = termios.tcgetattr(slave)
process = subprocess.Popen([
    a.binary, '--urdf', a.urdf, '--config', str(a.app/'configs/cube.json'),
    '--output', str(out), '--no-record', '--planning-mode', 'single-arm',
    '--goal', '{"delta":[0.005,0,0]}', '--port', str(port)],
    stdin=slave, stdout=slave, stderr=slave)


def wait_text(text):
    received = bytearray()
    until = time.monotonic() + 15
    while time.monotonic() < until:
        if select.select([master], [], [], .05)[0]:
            received.extend(os.read(master, 65536))
        if text.encode() in received:
            return
        assert process.poll() is None, received[-1000:]
    raise AssertionError((text, received[-1000:]))


def snapshot():
    with socket.create_connection(('127.0.0.1', port), timeout=5) as sock:
        stream = sock.makefile('rb')
        key = base64.b64encode(os.urandom(16)).decode()
        sock.sendall((f'GET / HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n'
                      'Upgrade: websocket\r\nConnection: Upgrade\r\n'
                      'Sec-WebSocket-Version: 13\r\n'
                      'Sec-WebSocket-Protocol: foxglove.sdk.v1\r\n'
                      f'Sec-WebSocket-Key: {key}\r\n\r\n').encode())
        response = stream.readline()
        assert b'101' in response, response
        headers = {}
        while (line := stream.readline()) != b'\r\n':
            k, v = line.decode().split(':', 1)
            headers[k.lower()] = v.strip()
        expected = base64.b64encode(hashlib.sha1(
            (key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
        assert headers['sec-websocket-accept'] == expected

        def receive():
            head = stream.read(2)
            assert len(head) == 2
            assert head[0] & 128 and not head[1] & 128  # unfragmented server frame
            size = head[1] & 127
            if size == 126:
                size = struct.unpack('!H', stream.read(2))[0]
            elif size == 127:
                size = struct.unpack('!Q', stream.read(8))[0]
            payload = stream.read(size)
            assert len(payload) == size
            return head[0] & 15, payload

        until = time.monotonic() + 10
        subscribed = False
        ids = set()
        while time.monotonic() < until:
            opcode, data = receive()
            if opcode == 1:
                message = json.loads(data)
                if message.get('op') == 'advertise' and not subscribed:
                    scene = next((c for c in message['channels']
                                  if c['topic'] == '/joint_path/scene'), None)
                    if scene:
                        payload = json.dumps({'op': 'subscribe', 'subscriptions': [
                            {'id': 1, 'channelId': scene['id']}]}).encode()
                        assert len(payload) < 126
                        mask = os.urandom(4)
                        sock.sendall(bytes([0x81, 128 | len(payload)]) + mask + bytes(
                            v ^ mask[i % 4] for i, v in enumerate(payload)))
                        subscribed = True
            elif opcode == 2:
                assert data[0] == 1 and struct.unpack('<I', data[1:5])[0] == 1
                scene = json.loads(data[13:])
                ids.update(e['id'] for e in scene['entities'])
                if {'front_cube', 'planned-path', 'waypoint-0'} <= ids:
                    return
        raise AssertionError(('missing static scene snapshot', ids))


try:
    wait_text('READY')
    os.write(master, b'\r')
    wait_text('Reached goal')
    snapshot()
    snapshot()
    os.write(master, b'x')
    process.wait(timeout=10)
    assert process.returncode == 0
    assert termios.tcgetattr(slave) == before
finally:
    if process.poll() is None:
        process.terminate()
        process.wait(timeout=10)
    os.close(master)
    os.close(slave)
print('PASS late subscription and reconnect restore cube, path and stop markers:', out)
