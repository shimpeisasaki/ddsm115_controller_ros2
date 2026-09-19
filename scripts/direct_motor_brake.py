#!/usr/bin/env python3
"""One-shot brake for this robot, without ROS. Never sends drive/mode commands.

Use only after all motor drivers have exited. A successful write is NOT proof
of physical stopping: inspect both wheels. Requires Linux fuser and serial access.
"""
import fcntl
import os
import select
import shutil
import subprocess
import termios
import time

CHANNELS = [
    ('left', '/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BD9133ABCD-if06', 2),
    ('right', '/dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BD9133ABCD-if04', 1),
]


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8c if crc & 1 else 0)
    return crc


def packet(motor_id):
    # Identical to MotorControl::set_brake in the existing driver.
    payload = bytes([motor_id, 0x64, 0, 0, 0, 0, 0, 0xff, 0])
    return payload + bytes([crc8(payload)])


def open_port(path):
    if not os.path.exists(path):
        raise RuntimeError('USB port not found: '+path)
    if shutil.which('fuser') is None:
        raise RuntimeError('fuser not installed; cannot check for another serial owner')
    owner = subprocess.run(['fuser', path], capture_output=True, timeout=2)
    if owner.returncode != 1 or owner.stderr:
        raise RuntimeError('Port busy or ownership check failed; close the driver first: '+
                           (owner.stdout+owner.stderr).decode(errors='replace'))
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        settings = termios.tcgetattr(fd)
        settings[0] = settings[1] = settings[3] = 0
        settings[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        settings[4] = settings[5] = termios.B115200
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIFLUSH)
        return fd
    except BaseException:
        os.close(fd)
        raise


def exchange(fd, motor_id):
    payload = packet(motor_id)
    deadline = time.monotonic()+.1
    while payload:
        remaining = deadline-time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise RuntimeError('Serial write timeout')
        try:
            written = os.write(fd, payload)
        except BlockingIOError:
            continue
        if written == 0:
            raise RuntimeError('Zero-byte serial write')
        payload = payload[written:]
    data = bytearray()
    deadline = time.monotonic()+.04
    while time.monotonic() < deadline:
        if not select.select([fd], [], [], max(0., deadline-time.monotonic()))[0]:
            break
        block = os.read(fd, 256)
        if not block:
            break
        data.extend(block)
        while len(data) >= 10:
            reply = data[:10]
            if reply[0] == motor_id and reply[1] in (1, 2) and crc8(reply[:9]) == reply[9]:
                return int.from_bytes(reply[4:6], 'big', signed=True), reply[8]
            del data[0]
    return None


def main():
    ports = []
    failures = False
    try:
        for name, path, motor_id in CHANNELS:
            try:
                fd = open_port(path)
                ports.append((name, fd, motor_id))
            except Exception as exc:
                failures = True
                print(f'{name}: FAILED: {exc}', flush=True)
        latest = {}
        for _ in range(20):
            for name, fd, motor_id in ports:
                try:
                    latest[name] = exchange(fd, motor_id)
                except Exception as exc:
                    failures = True
                    latest[name] = None
                    print(f'{name}: communication error: {exc}', flush=True)
            time.sleep(.05)
        for name, _, _ in ports:
            reply = latest.get(name)
            print(f'{name}: final feedback (RPM, error) = {reply}', flush=True)
            if reply is None or reply[0] != 0 or reply[1] != 0:
                failures = True
        print('VISUALLY CHECK BOTH WHEELS. If not stopped, use the physical emergency stop.', flush=True)
        return 1 if failures else 0
    finally:
        for _, fd, _ in ports:
            os.close(fd)


if __name__ == '__main__':
    raise SystemExit(main())
