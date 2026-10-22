"""api.py - the Testbed class: run your firmware on the simulated rig from Python.

    from testbed_sim import Testbed
    tb = Testbed("Debug/drone_fw.bin")        # .bin, .hex or .elf from CubeIDE
    tb.run(2.0)                               # 2 simulated seconds
    print(tb.read_text("USART1"))             # what the board printed on USB port 1
    print(tb.state())                         # true angles, motor speeds, pulses...
"""
from __future__ import annotations

import csv
import math

from . import config as config_mod
from .board import Board
from .loader import load_firmware


class Testbed:
    def __init__(self, firmware, config=None, overrides=None, log=print, quiet=False):
        self.cfg = config_mod.load(config, overrides)
        self._log = (lambda s: None) if quiet else log
        self.board = Board(self.cfg, lambda s: self._log(s))   # so serve() can redirect it
        self.m = self.board.m
        self.rig = self.board.rig
        segs, syms = load_firmware(firmware)
        for addr, data in segs:
            self.m.load_image(data, addr)
        self.m.symbols = syms
        self.m.cpu_reset()
        self._csv = None
        self._csv_every = None
        self._csv_next = 0.0

    # ---------------------------------------------------------------- running
    @property
    def t(self):
        """Simulated time in seconds."""
        return self.m.now_s()

    def run(self, seconds):
        """Run the firmware and the rig for `seconds` of simulated time."""
        self.m.run(seconds)

    def run_until(self, predicate, timeout=10.0, step=0.001):
        """Run until predicate(testbed) is true. Returns True if it happened."""
        t_end = self.t + timeout
        while self.t < t_end:
            if predicate(self):
                return True
            self.m.run(step)
        return predicate(self)

    # ---------------------------------------------------------------- serial
    def write(self, uart, data):
        """Send bytes from the PC to the board (uart = 'USART1' or 'USART3')."""
        self.board.usb_send(uart, data)

    def read(self, uart):
        """Bytes the board sent to the PC since the last read."""
        return self.board.usb_read(uart)

    def read_text(self, uart="USART1"):
        return self.read(uart).decode("utf-8", "replace")

    # ---------------------------------------------------------------- inputs
    def press_key(self, key=1, seconds=0.2):
        """Hold KEY1 or KEY2 for `seconds` (simulated)."""
        self.board.press_key(key, True)
        self.run(seconds)
        self.board.press_key(key, False)

    def push(self, yaw_torque=0.0, pitch_torque=0.0, seconds=0.1):
        """Push the frame by hand: extra torque (N m) on the joints for a while."""
        self.rig.disturbance = [yaw_torque, pitch_torque]
        self.run(seconds)
        self.rig.disturbance = [0.0, 0.0]

    def reset(self):
        """Press the board's RESET button."""
        self.m.system_reset("RESET button")

    # ---------------------------------------------------------------- outputs
    def state(self):
        """True state of the rig and what the firmware is driving (a dict)."""
        rig = self.rig
        sig = [self.board.esc_signal(i) for i in range(4)]
        return {
            "t": self.t,
            "yaw": rig.yaw.q, "pitch": rig.pitch.q,
            "yaw_rate": rig.yaw.dq, "pitch_rate": rig.pitch.dq,
            "esc_us": [round(s[1] * 1e6, 1) if s else None for s in sig],
            "esc_hz": [round(1 / s[0], 1) if s else None for s in sig],
            "esc_state": [e.state for e in rig.escs],
            "throttle": [e.throttle for e in rig.escs],
            "motor_rpm": [m.w * 60 / (2 * math.pi) for m in rig.motors],
            "thrust_n": [m.thrust() for m in rig.motors],
            "v_bus": rig.v_bus, "current_a": rig.total_current(),
            "led": self.board.led_on, "buzzer_hz": self.board.buzzer_hz,
            "enc_counts": [self.board.encoder_x4("yaw"), self.board.encoder_x4("pitch")],
            "cpu_halted": self.m.halted,
        }

    def log_csv(self, path, every=0.001):
        """Write the true rig state to a CSV file every `every` simulated seconds."""
        f = open(path, "w", newline="")
        self._csv = csv.writer(f)
        self._csv_file = f
        self._csv.writerow(["t", "yaw", "pitch", "yaw_rate", "pitch_rate",
                            "esc1_us", "esc2_us", "esc3_us", "esc4_us",
                            "rpm1", "rpm2", "rpm3", "rpm4", "v_bus", "current_a"])
        self._csv_every = every
        self._csv_next = self.t
        self.board.on_physics.append(self._csv_row)

    def _csv_row(self, board):
        t = self.rig.t
        if t + 1e-12 < self._csv_next:
            return
        self._csv_next += self._csv_every
        rig = self.rig
        sig = [board.esc_signal(i) for i in range(4)]
        self._csv.writerow([f"{t:.6f}", f"{rig.yaw.q:.6f}", f"{rig.pitch.q:.6f}",
                            f"{rig.yaw.dq:.5f}", f"{rig.pitch.dq:.5f}"]
                           + [f"{s[1]*1e6:.1f}" if s else "" for s in sig]
                           + [f"{m.w*60/(2*math.pi):.0f}" for m in rig.motors]
                           + [f"{rig.v_bus:.3f}", f"{rig.total_current():.3f}"])

    def close(self):
        if self._csv:
            self._csv_file.close()
            self._csv = None
