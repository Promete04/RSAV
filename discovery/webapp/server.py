#!/usr/bin/env python3
"""Roomba web control over the micro-USB Open Interface port.

Run:  python3 -I server.py [--port 8980] [--lan] [--dev /dev/ttyACM0]
Open: http://127.0.0.1:8980

Only the Python standard library is used. One thread reads the checksummed
sensor stream (OI opcode 148); HTTP handlers write commands under a lock.
A watchdog stops the wheels if the browser stops sending drive commands.
Precise moves (distance, angle, square, circle) run in a server thread that
closes the loop on the wheel encoders and stops on bumps, drops or mode loss.
"""
import argparse, json, math, os, select, signal, struct, termios, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))

# packet id -> (size, struct format)
PACKETS = {
    7: (1, "B"), 8: (1, "B"), 9: (1, "B"), 10: (1, "B"), 11: (1, "B"), 12: (1, "B"),
    13: (1, "B"), 14: (1, "B"), 15: (1, "B"), 17: (1, "B"), 21: (1, "B"), 22: (2, ">H"),
    23: (2, ">h"), 24: (1, "b"), 25: (2, ">H"), 26: (2, ">H"), 27: (2, ">H"),
    28: (2, ">H"), 29: (2, ">H"), 30: (2, ">H"), 31: (2, ">H"), 34: (1, "B"), 35: (1, "B"),
    37: (1, "B"), 43: (2, ">H"), 44: (2, ">H"), 45: (1, "B"),
    46: (2, ">H"), 47: (2, ">H"), 48: (2, ">H"), 49: (2, ">H"), 50: (2, ">H"), 51: (2, ">H"),
    54: (2, ">h"), 55: (2, ">h"), 56: (2, ">h"), 57: (2, ">h"), 58: (1, "B"),
}
STREAM = list(PACKETS)          # ~97 data bytes per 15 ms frame, inside the 115200 budget
MAX_SPEED = 500                 # mm/s, OI limit
DEADMAN_S = 0.35                # stop wheels if no drive command for this long

# Geometry from the OI spec (Create 2 figures); calibrate on your robot
MM_PER_COUNT = math.pi * 72.0 / 508.8
WHEEL_BASE_MM = 235.0

# Opcodes the raw-command box refuses: reset, baud change, and the ones that
# would break this server's sensor stream or close the OI under it.
RAW_BLOCKED = {7: "Reset reinicia el robot", 129: "Baud cortaría la comunicación",
               148: "Stream lo gestiona el servidor", 150: "Pause Stream lo gestiona el servidor",
               173: "Stop cierra el Open Interface"}


def enc_delta(old, new):
    return ((new - old + 32768) % 65536) - 32768


class Blocked(Exception):
    pass


class Robot:
    def __init__(self, dev):
        self.dev, self.fd = dev, None
        self.wlock = threading.Lock()
        self.state, self.frame_t, self.frames = {}, 0.0, 0
        self.last_drive_t, self.moving = 0.0, False
        self.log_line, self._text = "", b""
        self.error = None
        self.job, self.job_status = None, "sin movimiento"
        self.job_cancel = threading.Event()

    # ---- serial ----
    def open(self):
        fd = os.open(self.dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, a)
        termios.tcflush(fd, termios.TCIOFLUSH)
        self.fd = fd
        self.send(128)                                  # Start -> Passive
        time.sleep(0.1)
        self.start_stream()

    def start_stream(self):
        self.send(148, len(STREAM), *STREAM)

    def send(self, *data):
        with self.wlock:
            if self.fd is None:
                raise OSError("puerto no abierto")
            os.write(self.fd, bytes(b & 0xFF for b in data))

    def close(self):
        if self.fd is None:
            return
        try:
            self.cancel_job()
            self.send(145, 0, 0, 0, 0)    # wheels off
            self.send(144, 0, 0, 0)       # brushes and vacuum off
            self.send(150, 0)             # pause stream
            self.send(128)                # back to Passive
            time.sleep(0.05)
            self.send(173)                # leave OI
        finally:
            os.close(self.fd); self.fd = None

    # ---- modes ----
    def mode(self):
        return self.state.get(35)

    def ensure_control(self):
        """Wheels, motors, LEDs and Play need Safe or Full: switch to Safe if in Passive."""
        if self.mode() is None or self.mode() < 2:
            self.send(131)
            time.sleep(0.05)

    # ---- drive ----
    def drive_direct(self, left, right):
        left = max(-MAX_SPEED, min(MAX_SPEED, int(left)))
        right = max(-MAX_SPEED, min(MAX_SPEED, int(right)))
        self.send(145, *struct.pack(">hh", right, left))   # Drive Direct: right first

    def drive(self, left, right):
        """Joystick drive: cancels any precise move, subject to the dead-man watchdog."""
        self.cancel_job()
        self.drive_direct(left, right)
        self.last_drive_t = time.time()
        self.moving = bool(left or right)

    def stop(self):
        self.cancel_job()
        self.send(145, 0, 0, 0, 0)
        self.moving = False

    # ---- precise moves (server-side closed loop) ----
    def cancel_job(self):
        job = self.job
        if job and job.is_alive() and job is not threading.current_thread():
            self.job_cancel.set()
            job.join(timeout=1.0)
        self.job = None

    def start_job(self, steps, speed):
        self.cancel_job()
        self.ensure_control()
        self.job_cancel.clear()
        self.job = threading.Thread(target=self._run_steps, args=(steps, speed), daemon=True)
        self.job_status = "en marcha"
        self.job.start()

    def _snap(self):
        s = self.state
        if time.time() - self.frame_t > 0.3:
            raise Blocked("se perdieron los datos del robot")
        if self.job_cancel.is_set():
            raise Blocked("cancelado")
        return s

    def _guard(self, s, check_bump):
        if check_bump and s.get(7, 0) & 0x03:
            raise Blocked("choque")
        if s.get(7, 0) & 0x0C:
            raise Blocked("rueda en el aire")
        if s.get(35, 0) < 2:
            raise Blocked("el robot salió del modo Safe/Full")

    def _move(self, mm, speed):
        sign = 1 if mm >= 0 else -1
        s0 = dict(self._snap()); self._guard(s0, sign > 0)
        limit = time.time() + abs(mm) / max(speed, 20) * 3 + 2
        self.drive_direct(sign * speed, sign * speed)
        while True:
            time.sleep(0.015)
            s = self._snap(); self._guard(s, sign > 0)
            done = (enc_delta(s0[43], s[43]) + enc_delta(s0[44], s[44])) / 2 * MM_PER_COUNT
            if abs(done) >= abs(mm):
                return
            if time.time() > limit:
                raise Blocked("tiempo agotado (¿atascado?)")

    def _turn(self, deg, speed):
        sign = 1 if deg >= 0 else -1                        # positive = left
        arc = math.pi * WHEEL_BASE_MM * abs(deg) / 360.0
        s0 = dict(self._snap()); self._guard(s0, False)
        limit = time.time() + arc / max(speed, 20) * 3 + 2
        self.drive_direct(-sign * speed, sign * speed)
        while True:
            time.sleep(0.015)
            s = self._snap(); self._guard(s, False)
            dl = abs(enc_delta(s0[43], s[43])) * MM_PER_COUNT
            dr = abs(enc_delta(s0[44], s[44])) * MM_PER_COUNT
            if (dl + dr) / 2 >= arc:
                return
            if time.time() > limit:
                raise Blocked("tiempo agotado (¿atascado?)")

    def _arc(self, velocity, radius, seconds):
        self._guard(self._snap(), velocity > 0)
        self.send(137, *struct.pack(">HH", int(velocity) & 0xFFFF, int(radius) & 0xFFFF))
        end = time.time() + seconds
        while time.time() < end:
            time.sleep(0.015)
            self._guard(self._snap(), velocity > 0)

    def _run_steps(self, steps, speed):
        try:
            for i, st in enumerate(steps, 1):
                self.job_status = f"paso {i}/{len(steps)}"
                if st["type"] == "move":
                    self._move(float(st["mm"]), speed)
                elif st["type"] == "turn":
                    self._turn(float(st["deg"]), max(60, speed // 2))
                elif st["type"] == "arc":
                    self._arc(st["v"], st["r"], float(st["s"]))
                self.drive_direct(0, 0)
                time.sleep(0.1)
            self.job_status = "terminado"
        except Blocked as e:
            self.job_status = f"parado: {e}"
        except OSError as e:
            self.job_status = f"error: {e}"
        finally:
            try:
                self.drive_direct(0, 0)
            except OSError:
                pass

    # ---- reader + watchdog ----
    def run(self):
        buf = b""
        while True:
            try:
                if self.fd is None:
                    self.open(); self.error = None
                r, _, _ = select.select([self.fd], [], [], 0.05)
                if r:
                    try:
                        chunk = os.read(self.fd, 4096)
                    except BlockingIOError:
                        chunk = None
                    if chunk == b"":                     # readable but empty = USB device gone
                        raise OSError("el robot se desconectó del USB")
                    buf += chunk or b""
                buf = self._parse(buf)
                now = time.time()
                if self.moving and now - self.last_drive_t > DEADMAN_S:
                    self.send(145, 0, 0, 0, 0); self.moving = False
                if now - self.frame_t > 2.0 and now - getattr(self, "_restream_t", 0) > 2.0:
                    self._restream_t = now
                    self._restreams = getattr(self, "_restreams", 0) + 1
                    if self._restreams > 2:              # re-asking did not help: reopen the port
                        self._restreams = 0
                        raise OSError("sin datos del robot, reabriendo el puerto")
                    self.send(128); time.sleep(0.05); self.start_stream()
                elif now - self.frame_t < 1.0:
                    self._restreams = 0
            except OSError as e:
                self.error = f"{e.__class__.__name__}: {e}"
                if self.fd is not None:
                    try: os.close(self.fd)
                    except OSError: pass
                self.fd, buf = None, b""
                time.sleep(1.0)

    def _parse(self, buf):
        while True:
            i = buf.find(b"\x13")
            if i < 0:
                self._collect_text(buf); return b""
            self._collect_text(buf[:i])
            buf = buf[i:]
            if len(buf) < 2:
                return buf
            n = buf[1]
            if len(buf) < n + 3:
                return buf if n < 200 else buf[1:]
            frame = buf[: n + 3]
            if sum(frame) & 0xFF == 0 and self._decode(frame[2:-1]):
                buf = buf[n + 3:]
            else:
                buf = buf[1:]

    def _collect_text(self, b):
        if not b:
            return
        self._text = (self._text + b)[-400:]
        lines = [l.strip() for l in self._text.replace(b"\r", b"").split(b"\n")]
        done = [l for l in lines[:-1] if l]
        if done:
            self.log_line = done[-1].decode("latin1", "replace")

    def _decode(self, body):
        out, j = {}, 0
        while j < len(body):
            pid = body[j]
            if pid not in PACKETS:
                return False
            size, fmt = PACKETS[pid]
            if j + 1 + size > len(body):
                return False
            out[pid] = struct.unpack(fmt, body[j + 1: j + 1 + size])[0]
            j += 1 + size
        self.state, self.frame_t = out, time.time()
        self.frames += 1
        return True

    def snapshot(self):
        s, age = self.state, time.time() - self.frame_t
        b = lambda pid, bit: bool(s.get(pid, 0) >> bit & 1)
        return {
            "connected": self.fd is not None and age < 1.0,
            "age_ms": int(age * 1000) if self.frame_t else None,
            "error": self.error,
            "mode": s.get(35),
            "battery": {"mV": s.get(22), "mA": s.get(23), "tempC": s.get(24),
                        "mAh": s.get(25), "capacity": s.get(26), "chargingState": s.get(21),
                        "onDock": b(34, 1), "charger": b(34, 0)},
            "bump": {"left": b(7, 1), "right": b(7, 0)},
            "wheelDrop": {"left": b(7, 3), "right": b(7, 2)},
            "cliff": {"left": bool(s.get(9)), "frontLeft": bool(s.get(10)),
                      "frontRight": bool(s.get(11)), "right": bool(s.get(12))},
            "cliffSignal": [s.get(28), s.get(29), s.get(30), s.get(31)],
            "wall": bool(s.get(8)), "wallSignal": s.get(27), "virtualWall": bool(s.get(13)),
            "lightBump": [b(45, k) for k in range(6)],
            "lightBumpSignal": [s.get(k) for k in range(46, 52)],
            "overcurrent": {"leftWheel": b(14, 4), "rightWheel": b(14, 3),
                            "mainBrush": b(14, 2), "sideBrush": b(14, 0)},
            "motorCurrent": {"left": s.get(54), "right": s.get(55), "mainBrush": s.get(56), "sideBrush": s.get(57)},
            "dirt": s.get(15), "stasis": b(58, 0), "songPlaying": bool(s.get(37)),
            "ir": s.get(17),
            "encoders": {"left": s.get(43), "right": s.get(44)},
            "moving": self.moving or bool(self.job and self.job.is_alive()),
            "job": self.job_status,
            "log": self.log_line,
        }


COMMANDS = {
    "passive": [128], "safe": [131], "full": [132],
    "clean": [135], "spot": [134], "max": [136], "dock": [143],
    "power": [133],
}


def as_int(v, lo, hi, name):
    v = int(v)
    if not lo <= v <= hi:
        raise ValueError(f"{name} fuera de rango ({lo}..{hi})")
    return v


def make_handler(robot):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers(); self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = open(os.path.join(HERE, "index.html"), "rb").read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers(); self.wfile.write(body)
            elif self.path == "/api/state":
                self._json(200, robot.snapshot())
            else:
                self._json(404, {"error": "no existe"})

        def do_POST(self):
            # reject cross-site form posts: only same-origin fetch with JSON
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                return self._json(415, {"error": "se esperaba JSON"})
            try:
                n = int(self.headers.get("Content-Length", 0))
                d = json.loads(self.rfile.read(n) or b"{}")
                p = self.path
                if p == "/api/drive":
                    robot.drive(d.get("left", 0), d.get("right", 0))
                elif p == "/api/stop":
                    robot.stop()
                elif p == "/api/cmd" and d.get("cmd") in COMMANDS:
                    robot.stop()
                    robot.send(*COMMANDS[d["cmd"]])
                elif p == "/api/move":
                    speed = as_int(d.get("speed", 200), 50, 500, "velocidad")
                    steps = d.get("steps")
                    if not isinstance(steps, list) or not 1 <= len(steps) <= 40:
                        raise ValueError("pasos no válidos")
                    for st in steps:
                        t = st.get("type")
                        if t == "move": as_int(st.get("mm"), -5000, 5000, "distancia")
                        elif t == "turn": as_int(st.get("deg"), -1080, 1080, "ángulo")
                        elif t == "arc":
                            as_int(st.get("v"), -500, 500, "velocidad")
                            as_int(st.get("r"), -2000, 2000, "radio")
                            if not 0 < float(st.get("s", 0)) <= 60: raise ValueError("duración 0..60 s")
                        else: raise ValueError(f"paso desconocido: {t}")
                    robot.start_job(steps, speed)
                elif p == "/api/song":
                    slot = as_int(d.get("slot", 0), 0, 4, "ranura")
                    notes = d.get("notes") or []
                    if notes:
                        if not 1 <= len(notes) <= 16:
                            raise ValueError("una canción tiene de 1 a 16 notas")
                        data = []
                        for note, dur in notes:
                            # 0 = silence; 31..127 = MIDI note; duration in 1/64 s
                            note = int(note)
                            if note != 0 and not 31 <= note <= 127:
                                raise ValueError("nota fuera de rango (31..127, 0 = silencio)")
                            data += [note, as_int(dur, 1, 255, "duración")]
                        robot.send(140, slot, len(notes), *data)
                        time.sleep(0.1)              # robot drops commands sent right after a Song store (tested)
                    if d.get("play"):
                        robot.ensure_control()       # Play is ignored in Passive (tested)
                        robot.send(141, slot)
                elif p == "/api/motors":
                    robot.ensure_control()
                    main = as_int(d.get("main", 0), -127, 127, "cepillo principal")
                    side = as_int(d.get("side", 0), -127, 127, "cepillo lateral")
                    vac = as_int(d.get("vacuum", 0), 0, 127, "aspiración")
                    robot.send(144, *struct.pack(">bbB", main, side, vac))
                elif p == "/api/leds":
                    robot.ensure_control()
                    robot.send(139, as_int(d.get("bits", 0), 0, 15, "LEDs"),
                               as_int(d.get("color", 0), 0, 255, "color"),
                               as_int(d.get("intensity", 0), 0, 255, "intensidad"))
                elif p == "/api/raw":
                    raw = bytes.fromhex(str(d.get("hex", "")).replace(",", " "))
                    if not 1 <= len(raw) <= 64:
                        raise ValueError("entre 1 y 64 bytes")
                    if raw[0] in RAW_BLOCKED:
                        raise ValueError(f"opcode {raw[0]} bloqueado: {RAW_BLOCKED[raw[0]]}")
                    if raw[0] < 128:
                        raise ValueError("el primer byte debe ser un opcode (128..173)")
                    robot.cancel_job()
                    robot.send(*raw)
                else:
                    return self._json(400, {"error": "orden desconocida"})
                self._json(200, {"ok": True})
            except OSError as e:
                self._json(503, {"error": f"robot no disponible: {e}"})
            except (ValueError, TypeError, KeyError) as e:
                self._json(400, {"error": str(e)})
    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/serial/by-id/usb-iRobot_Corporation_iRobot_Roomba_5CF36F3A3635-if00")
    ap.add_argument("--port", type=int, default=8980)
    ap.add_argument("--lan", action="store_true", help="listen on all interfaces (phone on same Wi-Fi)")
    a = ap.parse_args()
    def _term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    robot = Robot(a.dev)
    threading.Thread(target=robot.run, daemon=True).start()
    host = "0.0.0.0" if a.lan else "127.0.0.1"
    srv = ThreadingHTTPServer((host, a.port), make_handler(robot))
    print(f"Roomba control on http://{'<this-pc-ip>' if a.lan else '127.0.0.1'}:{a.port}  (Ctrl+C to quit)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close(); robot.close(); print("robot released (Passive, OI stopped)")


if __name__ == "__main__":
    main()
