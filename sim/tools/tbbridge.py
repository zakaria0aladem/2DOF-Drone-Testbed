#!/usr/bin/env python3
"""tbbridge.py - log testbed telemetry to CSV and close the loop from Python.

Needs: pip install pyserial

Examples
  # passive logging, motors stay off
  python tbbridge.py log --port COM5 --out runs/idle.csv --duration 10

  # system-ID run: arm, PRBS on all four motors around 1250 us, +-80 us
  python tbbridge.py log --port /dev/ttyUSB0 --arm --excite prbs --base 1250 \
      --amp 80 --hold 0.05 --duration 60 --out runs/prbs_01.csv

  # closed loop with your model (a class with step(obs) -> 4 pulse widths)
  python tbbridge.py run --port /dev/ttyUSB0 --arm \
      --controller examples/controller_template.py:HoldAngle --duration 30

Every run writes <out>.csv plus <out>.json with the metadata (rate, limits,
columns, units, link stats). Ctrl-C disarms the board before exiting.
"""
from __future__ import annotations

import argparse
import csv
import importlib
import importlib.util
import json
import math
import os
import random
import sys
import time
from pathlib import Path

import tbproto as P

try:
    import serial
except ImportError:  # the protocol module is still importable without it
    serial = None

COLUMNS = [
    "host_t", "seq", "t_us", "t",
    "q1", "q2", "dq1", "dq2",
    "ax", "ay", "az", "gx", "gy", "gz",
    "u1", "u2", "u3", "u4",          # pulse the board applied right after this sample
    "cmd_seq", "cmd_delay", "flags", "armed", "lost",
    "cmd1", "cmd2", "cmd3", "cmd4",  # command the host sent in response to this sample
]
UNITS = {
    "host_t": "s (PC clock)", "t_us": "us (board clock, raw, wraps)", "t": "s (board clock, from 0)",
    "q1": "rad", "q2": "rad", "dq1": "rad/s", "dq2": "rad/s",
    "ax": "m/s^2", "ay": "m/s^2", "az": "m/s^2", "gx": "rad/s", "gy": "rad/s", "gz": "rad/s",
    "u1": "us", "cmd1": "us",
    "cmd_delay": "samples between the frame a command answered and the frame it was applied at",
}


class Testbed:
    """Serial link to the board. Single-threaded: call read() in your loop."""

    def __init__(self, port: str, baud: int = 1000000, timeout: float = 0.002):
        if serial is None:
            raise RuntimeError("pyserial is not installed: pip install pyserial")
        # serial_for_url also opens socket://localhost:5333 (the simulator)
        self.ser = serial.serial_for_url(port, baud, timeout=timeout)
        self.parser = P.Parser()
        self.hello: P.Hello | None = None
        self.texts: list[str] = []

    def close(self):
        try:
            self.disarm()
        finally:
            self.ser.close()

    def send(self, frame: bytes):
        self.ser.write(frame)

    def read(self) -> list[P.Telemetry]:
        data = self.ser.read(max(1, self.ser.in_waiting))
        sock = getattr(self.ser, "_socket", None)
        if data and sock is not None:
            # socket:// (the simulator) reports 1 byte waiting whatever is
            # there: take the rest in one go instead of byte by byte
            try:
                data += sock.recv(65536)
            except (BlockingIOError, InterruptedError):
                pass
        out = []
        for ftype, payload in self.parser.feed(data):
            if ftype == P.T_TELEMETRY and len(payload) == P.TELEM_FMT.size:
                out.append(P.Telemetry.unpack(payload))
            elif ftype == P.T_HELLO and len(payload) == P.HELLO_FMT.size:
                self.hello = P.Hello.unpack(payload)
            elif ftype == P.T_TEXT:
                msg = payload.decode("ascii", "replace")
                self.texts.append(msg)
                print(f"[board] {msg}", file=sys.stderr)
        return out

    def handshake(self, rate_hz: int | None = None, wait: float = 2.0) -> P.Hello:
        self.ser.reset_input_buffer()
        self.hello = None
        req = P.set_rate(rate_hz) if rate_hz else P.encode(P.T_PING)
        t_end = time.monotonic() + wait
        t_send = 0.0
        while time.monotonic() < t_end:
            # ask again every 0.25 s: the firmware drops a HELLO that comes
            # while a telemetry frame is still being sent
            if time.monotonic() >= t_send:
                self.send(req)
                t_send = time.monotonic() + 0.25
            self.read()
            if self.hello:
                break
        if not self.hello:
            raise RuntimeError("no HELLO from board: check port, baud, and that the firmware runs")
        if self.hello.proto_version != P.PROTO_VERSION:
            raise RuntimeError(f"firmware protocol v{self.hello.proto_version}, host v{P.PROTO_VERSION}")
        if rate_hz and self.hello.rate_hz != rate_hz:
            raise RuntimeError(f"board refused rate {rate_hz} Hz (running {self.hello.rate_hz} Hz)")
        return self.hello

    def arm(self):
        self.send(P.encode(P.T_ARM))

    def disarm(self):
        for _ in range(3):
            self.send(P.encode(P.T_DISARM))
        self.ser.flush()

    def command(self, seq: int, us):
        self.send(P.motor_cmd(seq, us))


# ---------------------------------------------------------------- excitation

class Excitation:
    """Open-loop input for system identification. u_i = base + mix_i * amp * s(t)."""

    def __init__(self, kind, base, amp, mix, hold=0.05, f0=0.1, f1=5.0, period=4.0,
                 duration=60.0, seed=0):
        self.kind, self.base, self.amp, self.mix = kind, base, amp, mix
        self.hold, self.f0, self.f1, self.period, self.T = hold, f0, f1, period, duration
        self.rng = random.Random(seed)
        self._level, self._next_switch = 1.0, 0.0

    def step(self, obs, t):
        if self.kind == "const":
            s = 0.0
        elif self.kind == "step":
            s = 1.0 if (t % self.period) < self.period / 2 else -1.0
        elif self.kind == "prbs":
            if t >= self._next_switch:
                self._level = self.rng.choice((-1.0, 1.0))
                self._next_switch = t + self.hold
            s = self._level
        elif self.kind == "chirp":   # linear sweep f0 -> f1 over the run
            k = (self.f1 - self.f0) / max(self.T, 1e-6)
            s = math.sin(2 * math.pi * (self.f0 * t + 0.5 * k * t * t))
        else:
            raise ValueError(self.kind)
        return [self.base + m * self.amp * s for m in self.mix]


def load_controller(spec: str, args):
    """spec = 'path/to/file.py:ClassName' or 'package.module:ClassName'."""
    target, _, cls_name = spec.partition(":")
    if target.endswith(".py"):
        mod_spec = importlib.util.spec_from_file_location(Path(target).stem, target)
        mod = importlib.util.module_from_spec(mod_spec)
        mod_spec.loader.exec_module(mod)
    else:
        mod = importlib.import_module(target)
    cls = getattr(mod, cls_name)
    return cls(**json.loads(args.controller_args)) if args.controller_args else cls()


# ---------------------------------------------------------------- run loop

def run(args, policy):
    tb = Testbed(args.port, args.baud)
    hello = tb.handshake(args.rate)
    print(f"board: proto v{hello.proto_version}, {hello.rate_hz} Hz, "
          f"pulse {hello.us_min}-{hello.us_max} us", file=sys.stderr)
    if hasattr(policy, "reset"):
        policy.reset(hello)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    f = open(out, "w", newline="")
    w = csv.writer(f)
    w.writerow(COLUMNS)

    stats = dict(frames=0, lost=0, timeouts=0, clamped=0, max_cmd_delay=0, delay_hist={})
    last_seq = None
    t0_us = None
    unwrap = 0
    prev_us = None
    first_sent = None
    q_limit = args.q_limit
    stop_reason = "duration"

    if args.arm:
        tb.arm()
    t_start = time.monotonic()
    try:
        while time.monotonic() - t_start < args.duration:
            for m in tb.read():
                host_t = time.time()
                lost = 0 if last_seq is None else (m.seq - last_seq - 1) & 0xFFFFFFFF
                last_seq = m.seq
                if prev_us is not None and m.t_us < prev_us:
                    unwrap += 1 << 32
                prev_us = m.t_us
                t_abs = m.t_us + unwrap
                if t0_us is None:
                    t0_us = t_abs
                t = (t_abs - t0_us) * 1e-6
                cmd_delay = ""
                if first_sent is not None and m.seq > first_sent:
                    d = (m.seq - m.cmd_seq) & 0xFFFF
                    if d <= m.seq - first_sent:   # ignore a cmd_seq left over from an earlier run
                        cmd_delay = d

                cmd = [""] * 4
                if policy is not None:
                    cmd = [float(x) for x in policy.step(m, t)]
                    tb.command(m.seq, cmd)   # host seq = frame seq -> board echoes it
                    if first_sent is None:
                        first_sent = m.seq

                w.writerow([f"{host_t:.6f}", m.seq, m.t_us, f"{t:.6f}",
                            *(f"{x:.6g}" for x in (*m.q, *m.dq, *m.acc, *m.gyro)),
                            *m.motor_us, m.cmd_seq, cmd_delay, m.flags, int(m.armed), lost,
                            *(f"{x:.1f}" if x != "" else "" for x in cmd)])

                stats["frames"] += 1
                stats["lost"] += lost
                stats["timeouts"] += bool(m.flags & P.F_CMD_TIMEOUT)
                stats["clamped"] += bool(m.flags & P.F_CLAMPED)
                if cmd_delay != "" and m.armed:
                    stats["max_cmd_delay"] = max(stats["max_cmd_delay"], cmd_delay)
                    stats["delay_hist"][cmd_delay] = stats["delay_hist"].get(cmd_delay, 0) + 1

                if m.flags & P.F_KILLED:
                    stop_reason = f"board kill switch latched at seq {m.seq}, reset the board"
                    raise KeyboardInterrupt
                if q_limit and (abs(m.q[0]) > q_limit or abs(m.q[1]) > q_limit):
                    stop_reason = f"joint limit {q_limit} rad exceeded at seq {m.seq}"
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        if stop_reason == "duration":
            stop_reason = "ctrl-c"
    finally:
        tb.close()
        f.close()

    stats["crc_errors"] = tb.parser.crc_errors
    stats["stop_reason"] = stop_reason
    stats["delay_hist"] = {str(k): v for k, v in sorted(stats["delay_hist"].items())}
    meta = dict(
        proto_version=hello.proto_version, rate_hz=hello.rate_hz,
        us_min=hello.us_min, us_max=hello.us_max,
        mode=args.mode, argv=sys.argv, started=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        columns=COLUMNS, units=UNITS, stats=stats,
        timing=("row k: sensors sampled at t, then u1..u4 applied until the next row; "
                "cmd1..4 is what the host sent after seeing row k"),
    )
    out.with_suffix(".json").write_text(json.dumps(meta, indent=2))
    print(f"saved {stats['frames']} rows to {out} "
          f"(lost {stats['lost']}, crc errors {stats['crc_errors']}, stop: {stop_reason})",
          file=sys.stderr)
    return stats


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    for name in ("log", "run"):
        s = sub.add_parser(name)
        s.add_argument("--port", required=True,
                       help="COM9, /dev/ttyUSB0, ... or socket://localhost:5333 for the simulator")
        s.add_argument("--baud", type=int, default=1000000,
                       help="1000000 for the Hiwonder board's USB port 2 (921600 on the old board)")
        s.add_argument("--rate", type=int, default=500, help="Hz, must divide 1000")
        s.add_argument("--duration", type=float, default=30.0, help="s")
        s.add_argument("--out", default=f"runs/{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        s.add_argument("--arm", action="store_true", help="arm the board (motors can spin)")
        s.add_argument("--q-limit", type=float, default=0.0, help="disarm if |q| exceeds this, rad")
    lg = sub.choices["log"]
    lg.add_argument("--excite", choices=["none", "const", "step", "prbs", "chirp"], default="none")
    lg.add_argument("--base", type=float, default=1000.0, help="us")
    lg.add_argument("--amp", type=float, default=0.0, help="us")
    lg.add_argument("--mix", default="1,1,1,1", help="per-motor sign/weight of the excitation")
    lg.add_argument("--hold", type=float, default=0.05, help="PRBS minimum hold time, s")
    lg.add_argument("--f0", type=float, default=0.1)
    lg.add_argument("--f1", type=float, default=5.0)
    lg.add_argument("--period", type=float, default=4.0, help="step period, s")
    lg.add_argument("--seed", type=int, default=0)
    rn = sub.choices["run"]
    rn.add_argument("--controller", required=True, help="file.py:Class or module:Class")
    rn.add_argument("--controller-args", default="", help='JSON kwargs, e.g. \'{"kp": 40}\'')
    args = ap.parse_args(argv)

    if args.mode == "log":
        if args.excite != "none" and not args.arm:
            ap.error("--excite needs --arm (otherwise the board ignores the commands)")
        policy = None if args.excite == "none" else Excitation(
            args.excite, args.base, args.amp, [float(x) for x in args.mix.split(",")],
            args.hold, args.f0, args.f1, args.period, args.duration, args.seed)
    else:
        policy = load_controller(args.controller, args)
    return run(args, policy)


if __name__ == "__main__":
    main()
