"""Pretend STM32 board, for testing the Pi programs without the rig.

It opens a virtual serial port (a "pty") and talks the same protocol as
bsp_comm.c: sends $TEL at 50 Hz, accepts $SET / $YAW / $DIS. The "rig" is a
very simple model: each angle moves smoothly towards its target.

Run:   python3 fake_board.py
It prints a port name such as /dev/pts/3. Give that to the other program:
       python3 mqtt_bridge.py --port /dev/pts/3

Works on Linux and on the Pi (not on Windows: no pty there).
Arming: the fake board starts disarmed, then "presses K1" by itself after 2 s
(6 s countdown like the real one). After $DIS it waits 10 s and arms again.
"""

import os
import pty
import random
import time
import tty

import protocol

TEL_PERIOD_S = 0.02
ARM_COUNTDOWN_S = 6.0
REARM_AFTER_S = 10.0
LINK_TIMEOUT_S = 0.5


def main():
    master, slave = pty.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    print("Fake board on %s  (Ctrl+C to stop)" % os.ttyname(slave))

    t0 = time.time()
    pitch = yaw = 0.0
    target = 0.0
    yaw_target = 0.0
    state = 0
    countdown_start = t0 + 2.0          # "K1 pressed" 2 s after start
    last_rx = None
    link_lost = 0
    bad = 0
    gains = [6.0, 2.5, 0.85, 3.0, 0.0, 0.5]     # pitch kp ki kd, yaw kp ki kd
    next_gain = t0
    buf = b""
    next_tel = t0

    while True:
        now = time.time()
        # --- read commands ---
        try:
            buf += os.read(master, 1024)
        except (BlockingIOError, OSError):
            pass
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", "replace").strip()
            if not line:
                continue
            body, _, cs = line[1:].partition("*")
            try:
                ok = line.startswith("$") and int(cs, 16) == protocol.checksum(body)
            except ValueError:
                ok = False
            if not ok:
                bad += 1
                continue
            parts = body.split(",")
            if parts[0] == "SET" and len(parts) == 2:
                target = max(-35.0, min(35.0, float(parts[1])))
            elif parts[0] == "YAW" and len(parts) == 2:
                yaw_target = max(-150.0, min(150.0, float(parts[1])))
            elif parts[0] == "GP" and len(parts) == 4:
                gains[0:3] = [float(x) for x in parts[1:]]
                next_gain = now
            elif parts[0] == "GY" and len(parts) == 4:
                gains[3:6] = [float(x) for x in parts[1:]]
                next_gain = now
            elif parts[0] == "DIS":
                state = 0
                countdown_start = now + REARM_AFTER_S
            else:
                bad += 1
                continue
            last_rx = now
            link_lost = 0

        if last_rx is not None and not link_lost and now - last_rx > LINK_TIMEOUT_S:
            link_lost = 1
            target = 0.0

        # --- arming ---
        if state == 0 and countdown_start is not None and now >= countdown_start:
            state = 1
        if state == 1 and now - countdown_start >= ARM_COUNTDOWN_S:
            state = 2
            countdown_start = None

        if now >= next_gain:
            next_gain = now + 1.0
            try:
                os.write(master, protocol.build_line("GAIN,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f" % tuple(gains)).encode("ascii"))
            except OSError:
                pass

        # --- "rig" ---
        if next_tel <= now:
            next_tel += TEL_PERIOD_S
            if state == 2:
                pitch += (target - pitch) * 0.04 + random.gauss(0, 0.15)
                yaw += (yaw_target - yaw) * 0.03 + random.gauss(0, 0.1)
                output = 6.0 * (target - pitch)
                yaw_out = max(-75.0, min(75.0, 3.0 * (yaw_target - yaw)))
                base = 1200
            else:
                pitch += (-25.0 - pitch) * 0.02    # falls onto its stop
                output = yaw_out = 0.0
                base = 1000
            left = int(base + output) if state == 2 else 1000
            right = int(base - output) if state == 2 else 1000
            m = [right - yaw_out, left + yaw_out, left - yaw_out, right + yaw_out]
            m = [int(v) if state == 2 else 1000 for v in m]
            body = "TEL,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%.1f,%.1f,%d,%d,%d,%d,%d" % (
                int((now - t0) * 1000), pitch, pitch + 0.4, yaw, target, output,
                left, right, 1 if state == 2 else 0, link_lost, bad,
                yaw_target, yaw_out, m[0], m[1], m[2], m[3], state)
            try:
                os.write(master, protocol.build_line(body).encode("ascii"))
            except OSError:
                pass
        time.sleep(0.002)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
