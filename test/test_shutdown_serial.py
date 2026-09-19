"""Integration check using two pseudo-terminals, never real motor ports.

Run after colcon build. Checks SIGINT and partial-construction teardown.
"""
import errno
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import threading
import time
import pytest


def crc(data):
    result = 0
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = (result >> 1) ^ (0x8c if result & 1 else 0)
    return result


class FakeMotor:
    def __init__(self, motor_id, respond=True):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave)
        self.motor_id = motor_id
        self.respond = respond
        self.packets = []
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        data = bytearray()
        while not self.done.is_set():
            try:
                if not select.select([self.master], [], [], .02)[0]:
                    continue
                data.extend(os.read(self.master, 1024))
                while len(data) >= 10:
                    packet = bytes(data[:10]); del data[:10]
                    self.packets.append(packet)
                    if packet[0] != self.motor_id or packet[1] == 0xa0 or not self.respond:
                        continue
                    reply = bytes([self.motor_id, 2, 0, 0, 0, 0, 25, 0, 0])
                    os.write(self.master, reply+bytes([crc(reply)]))
            except OSError as exc:
                if exc.errno not in (errno.EIO, errno.EBADF):
                    raise
                time.sleep(.01)

    def brakes(self):
        return sum(p[1] == 0x64 and p[7] == 0xff and crc(p[:9]) == p[9] for p in self.packets)

    def close(self):
        self.done.set(); self.thread.join(timeout=1)
        os.close(self.master); os.close(self.slave)


def launch(left, right, tmp_path):
    executable = Path(__file__).resolve().parents[3]/'build/ddsm115_controller/velocity_control'
    assert executable.exists(), 'Build ddsm115_controller first'
    env = dict(os.environ, ROS_DOMAIN_ID='231', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(tmp_path))
    return subprocess.Popen([str(executable), '--ros-args', '-p', f'left_usb_dev:={left.path}',
                             '-p', f'right_usb_dev:={right.path}'],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


@pytest.mark.parametrize('partial_failure, stop_signal',
                         [(False, signal.SIGINT), (False, signal.SIGTERM), (True, signal.SIGINT)])
def test_shutdown_brakes_both_channels(tmp_path, partial_failure, stop_signal):
    left, right = FakeMotor(2), FakeMotor(1, respond=not partial_failure)
    process = None
    try:
        process = launch(left, right, tmp_path)
        if not partial_failure:
            deadline = time.monotonic()+5
            while (left.brakes() < 1 or right.brakes() < 1) and time.monotonic() < deadline:
                if process.poll() is not None:
                    pytest.fail(process.communicate()[0])
                time.sleep(.02)
            assert left.brakes() >= 1 and right.brakes() >= 1
            before = (left.brakes(), right.brakes())
            process.send_signal(stop_signal)
        output, _ = process.communicate(timeout=5)
        if partial_failure:
            assert process.returncode == 1, output
            assert left.brakes() >= 2 and right.brakes() >= 1, output
        else:
            assert process.returncode == 0, output
            assert left.brakes() > before[0] and right.brakes() > before[1], output
    finally:
        if process is not None and process.poll() is None:
            process.kill(); process.communicate()
        left.close(); right.close()
