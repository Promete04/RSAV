#!/usr/bin/env python3
"""Roomba OI examples. Usage: python3 examples.py <example>

Motion examples need the robot on the floor with free space around it.
Users of `discovery/scripts/control.sh`: stop the web remote first, only one
program can own the serial port.
"""
import sys
import time

from roomba_oi import Blocked, Roomba, SPIN_LEFT, move_mm, turn_deg


def forward(r):          # 50 cm straight ahead
    print(f"moved {move_mm(r, 500):.0f} mm")


def backward(r):         # 30 cm back
    print(f"moved {move_mm(r, -300):.0f} mm")


def left(r):             # 90 degrees left in place
    print(f"turned {turn_deg(r, 90):.0f} deg")


def right(r):            # 90 degrees right in place
    print(f"turned {turn_deg(r, -90):.0f} deg")


def square(r):           # 40 cm square, turning left at each corner
    for side in range(4):
        move_mm(r, 400)
        turn_deg(r, 90)
        print("side", side + 1, "done")


def circle(r):           # open-loop arc: radius 300 mm at 150 mm/s for one lap
    lap_s = 2 * 3.1416 * 300 / 150          # circumference / speed ~ 12.6 s
    r.drive(150, 300)                       # positive radius = curve to the left
    time.sleep(lap_s)
    r.stop()


def spin_timed(r):       # the tested command: 1 s spin left at 100 mm/s
    r.drive(100, SPIN_LEFT)
    time.sleep(1.0)
    r.stop()


def wander(r, seconds=30):   # drive until a bump, back off, turn, repeat
    end = time.time() + seconds
    while time.time() < end:
        try:
            move_mm(r, 2000, speed=150)     # stops early on a bump
        except Blocked as e:
            print("blocked:", e)
            if "bumper" not in str(e):
                return                       # cliff / wheel drop: stop for good
            move_mm(r, -100, speed=100)
            turn_deg(r, 60)


def sensors(r):          # read once with Query List (no movement)
    s = r.query([7, 21, 22, 23, 24, 25, 26, 34, 35, 43, 44, 45])
    print(f"mode={s[35]} voltage={s[22]} mV current={s[23]} mA temp={s[24]} C "
          f"charge={s[25]}/{s[26]} mAh on_dock={bool(s[34] & 2)}")
    print(f"bump L/R={bool(s[7] & 2)}/{bool(s[7] & 1)} drop L/R={bool(s[7] & 8)}/{bool(s[7] & 4)} "
          f"light_bumper={s[45] & 0x3F:06b} encoders={s[43]},{s[44]}")


def stream(r, seconds=2):    # 15 ms stream, print 4 times a second (no movement)
    end, last = time.time() + seconds, 0
    for s in r.stream([22, 23, 35, 43, 44]):
        if time.time() - last > 0.25:
            print(s); last = time.time()
        if time.time() > end:
            break


def beep(r):
    r.beep()
    time.sleep(1)


EXAMPLES = {f.__name__: f for f in (forward, backward, left, right, square, circle,
                                    spin_timed, wander, sensors, stream, beep)}
NEEDS_SAFE = {"forward", "backward", "left", "right", "square", "circle", "spin_timed", "wander", "beep"}

if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else ""
    if name not in EXAMPLES:
        sys.exit("examples: " + ", ".join(EXAMPLES))
    robot = Roomba()
    try:
        robot.start()                        # always first: opens the OI (Passive)
        if name in NEEDS_SAFE:
            robot.safe()                     # wheels only move in Safe or Full
        EXAMPLES[name](robot)
    except Blocked as e:
        print("stopped:", e)
    finally:
        robot.close()                        # wheels off, Passive, OI off
