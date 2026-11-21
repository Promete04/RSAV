"""Minimal Roomba Open Interface driver for the micro-USB port.

Linux, Python standard library only (no pyserial). On Windows or macOS,
replace open/send/read_exact with pyserial: serial.Serial(port, 115200, timeout=0.5).
"""
import math
import os
import select
import struct
import termios
import time

PORT = "/dev/ttyACM0"

# Geometry from the OI spec (Create 2 figures). Calibrate on your robot.
COUNTS_PER_REV = 508.8
WHEEL_DIAMETER_MM = 72.0
WHEEL_BASE_MM = 235.0
MM_PER_COUNT = math.pi * WHEEL_DIAMETER_MM / COUNTS_PER_REV   # ~0.4446

# Special radius values for Drive (opcode 137)
STRAIGHT = 0x8000
SPIN_LEFT = 1        # counter-clockwise
SPIN_RIGHT = -1      # clockwise

# packet id -> struct format, big-endian. Size comes from the format.
PACKET_FMT = {
    7: "B", 8: "B", 9: "B", 10: "B", 11: "B", 12: "B", 13: "B", 14: "B", 15: "B", 16: "B",
    17: "B", 18: "B", 19: ">h", 20: ">h", 21: "B", 22: ">H", 23: ">h", 24: "b", 25: ">H",
    26: ">H", 27: ">H", 28: ">H", 29: ">H", 30: ">H", 31: ">H", 32: "B", 33: ">H", 34: "B",
    35: "B", 36: "B", 37: "B", 38: "B", 39: ">h", 40: ">h", 41: ">h", 42: ">h", 43: ">H",
    44: ">H", 45: "B", 46: ">H", 47: ">H", 48: ">H", 49: ">H", 50: ">H", 51: ">H", 52: "B",
    53: "B", 54: ">h", 55: ">h", 56: ">h", 57: ">h", 58: "B",
}
PACKET_SIZE = {pid: struct.calcsize(fmt) for pid, fmt in PACKET_FMT.items()}


def enc_delta(old, new):
    """Signed change between two 16-bit encoder readings, handling wrap-around."""
    return ((new - old + 32768) % 65536) - 32768


class Roomba:
    def __init__(self, port=PORT):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0                               # raw mode: no translation
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1, no modem control
        a[4] = a[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, a)

    # ---------- raw I/O ----------
    def send(self, *data):
        os.write(self.fd, bytes(b & 0xFF for b in data))

    def flush_input(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def read_exact(self, n, timeout=0.5):
        out, end = b"", time.time() + timeout
        while len(out) < n and time.time() < end:
            if select.select([self.fd], [], [], 0.02)[0]:
                try:
                    out += os.read(self.fd, n - len(out))
                except BlockingIOError:
                    pass
        if len(out) != n:
            raise TimeoutError(f"wanted {n} bytes, got {len(out)}")
        return out

    # ---------- modes ----------
    def start(self):
        self.send(128); time.sleep(0.03)     # -> Passive

    def safe(self):
        self.send(131); time.sleep(0.03)     # -> Safe

    def full(self):
        self.send(132); time.sleep(0.03)     # -> Full (no protections!)

    def close(self):
        """Stop the wheels, stop any stream, hand control back, close the OI."""
        try:
            self.stop()
            self.send(150, 0)                # pause stream
            self.send(128); time.sleep(0.03) # Passive
            self.send(173)                   # OI off
        finally:
            os.close(self.fd)

    # ---------- wheels ----------
    def drive(self, velocity_mm_s, radius_mm=STRAIGHT):
        """Opcode 137. velocity -500..500, radius -2000..2000 or STRAIGHT/SPIN_LEFT/SPIN_RIGHT."""
        v = max(-500, min(500, int(velocity_mm_s)))
        self.send(137, *struct.pack(">HH", v & 0xFFFF, int(radius_mm) & 0xFFFF))

    def drive_direct(self, left_mm_s, right_mm_s):
        """Opcode 145. Note the wire order: RIGHT wheel first, then LEFT."""
        l = max(-500, min(500, int(left_mm_s)))
        r = max(-500, min(500, int(right_mm_s)))
        self.send(145, *struct.pack(">hh", r, l))

    def stop(self):
        self.drive_direct(0, 0)

    # ---------- sensors ----------
    def sensor(self, pid):
        """Opcode 142: one packet."""
        self.flush_input()
        self.send(142, pid)
        return struct.unpack(PACKET_FMT[pid], self.read_exact(PACKET_SIZE[pid]))[0]

    def query(self, pids):
        """Opcode 149: several packets in one request -> {id: value}."""
        self.flush_input()
        self.send(149, len(pids), *pids)
        raw = self.read_exact(sum(PACKET_SIZE[p] for p in pids))
        out, i = {}, 0
        for p in pids:
            n = PACKET_SIZE[p]
            out[p] = struct.unpack(PACKET_FMT[p], raw[i:i + n])[0]
            i += n
        return out

    def stream(self, pids):
        """Opcode 148: generator yielding {id: value} every 15 ms. Skips log text and bad frames."""
        self.flush_input()
        self.send(148, len(pids), *pids)
        buf = b""
        while True:
            if select.select([self.fd], [], [], 0.1)[0]:
                try:
                    buf += os.read(self.fd, 4096)
                except BlockingIOError:
                    pass
            while True:
                i = buf.find(b"\x13")
                if i < 0:
                    buf = b""; break
                buf = buf[i:]
                if len(buf) < 2 or len(buf) < buf[1] + 3:
                    break                                  # wait for more bytes
                n = buf[1]
                frame, body = buf[:n + 3], buf[2:n + 2]
                values = self._decode(body) if sum(frame) & 0xFF == 0 else None
                if values is None:
                    buf = buf[1:]                          # false header, resync
                    continue
                buf = buf[n + 3:]
                yield values

    @staticmethod
    def _decode(body):
        out, j = {}, 0
        while j < len(body):
            pid = body[j]
            if pid not in PACKET_FMT or j + 1 + PACKET_SIZE[pid] > len(body):
                return None
            out[pid] = struct.unpack(PACKET_FMT[pid], body[j + 1:j + 1 + PACKET_SIZE[pid]])[0]
            j += 1 + PACKET_SIZE[pid]
        return out

    # ---------- sound ----------
    def beep(self):
        self.send(140, 0, 3, 72, 8, 76, 8, 79, 12)   # song 0: C5 E5 G5
        self.send(141, 0)                           # play it (Safe/Full)


# ---------- closed-loop motion using the wheel encoders ----------
class Blocked(Exception):
    """Raised when a bump, wheel drop or mode drop stops a move."""


def _guard(s, check_bump=True):
    if check_bump and s[7] & 0x03:
        raise Blocked("bumper pressed")
    if s[7] & 0x0C:
        raise Blocked("wheel off the floor")
    if s[35] < 2:
        raise Blocked("robot left Safe/Full mode (cliff, wheel drop or charger)")


def move_mm(robot, distance_mm, speed=200):
    """Drive straight; negative distance = backwards. Returns mm actually travelled."""
    sign = 1 if distance_mm >= 0 else -1
    s0 = robot.query([43, 44, 7, 35])
    _guard(s0, check_bump=sign > 0)                          # backing off a wall is allowed
    robot.drive_direct(sign * speed, sign * speed)
    try:
        while True:
            time.sleep(0.02)
            s = robot.query([43, 44, 7, 35])
            _guard(s, check_bump=sign > 0)
            dl = enc_delta(s0[43], s[43]) * MM_PER_COUNT
            dr = enc_delta(s0[44], s[44]) * MM_PER_COUNT
            done = (dl + dr) / 2
            if abs(done) >= abs(distance_mm):
                return done
    finally:
        robot.stop()


def turn_deg(robot, degrees, speed=100):
    """Spin in place; positive = left (counter-clockwise). Returns degrees actually turned."""
    sign = 1 if degrees >= 0 else -1
    arc = math.pi * WHEEL_BASE_MM * abs(degrees) / 360.0     # mm each wheel must travel
    s0 = robot.query([43, 44, 7, 35])
    _guard(s0, check_bump=False)                             # turning away from a wall is allowed
    robot.drive_direct(-sign * speed, sign * speed)          # left back, right forward = left turn
    try:
        while True:
            time.sleep(0.02)
            s = robot.query([43, 44, 7, 35])
            _guard(s, check_bump=False)
            dl = enc_delta(s0[43], s[43]) * MM_PER_COUNT
            dr = enc_delta(s0[44], s[44]) * MM_PER_COUNT
            if (abs(dl) + abs(dr)) / 2 >= arc:
                return math.degrees((dr - dl) / WHEEL_BASE_MM)
    finally:
        robot.stop()
