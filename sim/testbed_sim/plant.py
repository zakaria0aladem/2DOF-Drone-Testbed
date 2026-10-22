"""plant.py - the testbed without the firmware: a fast model for PIML and DeepC.

This is the same physics as the full simulator (rig.py: ESCs, motors, props,
joint friction, end stops, power supply) and the same sensor errors
(encoder resolution, QMI8658 noise, bias and scale), but you drive the motors
directly from Python instead of through firmware. One simulated second takes
about 50 ms, so you can generate hours of data in minutes.

    from testbed_sim.plant import RigEnv
    env = RigEnv(dt=0.002)                     # 500 Hz, like the telemetry
    obs = env.reset()
    for k in range(5000):
        obs = env.step([1250, 1300, 1300, 1250])   # ESC pulses in us
    print(obs.q, obs.gyro)

The observation has what the board would report (q from the encoder counts,
IMU readings with their errors) and the true state next to it (true_q,
true_dq), so a model can be trained on realistic data and checked against
the truth.

The CSV written by RigEnv.log_csv() has the same column names as the CSV
from tbbridge.py on the real rig (t, q1, q2, dq1, dq2, ax..gz, u1..u4), so
the same training code reads both.
"""
from __future__ import annotations

import csv
import math
import random
from dataclasses import dataclass, field

from . import config as config_mod
from .rig import ESC, Rig

G0 = 9.80665
DEG = math.pi / 180


@dataclass
class Obs:
    t: float                         # s
    q: tuple                         # (yaw, pitch) rad, from the encoder counts
    dq: tuple                        # rad/s, difference of q over one step
    acc: tuple                       # m/s^2, IMU accelerometer (chip axes)
    gyro: tuple                      # rad/s, IMU gyroscope (chip axes)
    motor_us: tuple                  # the pulses applied during the step that ended here
    true_q: tuple = field(default=(0.0, 0.0))
    true_dq: tuple = field(default=(0.0, 0.0))
    rpm: tuple = field(default=(0.0, 0.0, 0.0, 0.0))
    v_bus: float = 0.0
    current_a: float = 0.0


class _Imu:
    """QMI8658 errors at its 896.8 Hz output rate, +-8 g / +-512 dps ranges."""

    ODR = 896.8

    def __init__(self, cfg, rng):
        self.cfg = cfg
        self.rng = rng
        self.g_bias = [b * DEG for b in cfg["gyro_bias_dps"]]
        self.a_bias = [b * 1e-3 * G0 for b in cfg["accel_bias_mg"]]
        self.a_sig = cfg["accel_noise_ug_rthz"] * 1e-6 * G0 * math.sqrt(self.ODR / 2)
        self.g_sig = cfg["gyro_noise_mdps_rthz"] * 1e-3 * DEG * math.sqrt(self.ODR / 2)
        self.a_lsb = 8 * G0 / 32768
        self.g_lsb = 512 * DEG / 32768

    def read(self, f, w, dt, noise=True):
        c = self.cfg
        if noise:
            walk = c["gyro_bias_walk_dps_rts"] * DEG * math.sqrt(dt)
            for i in range(3):
                self.g_bias[i] += self.rng.gauss(0, walk)
        acc, gyro = [], []
        for i in range(3):
            a = f[i] * (1 + c["accel_scale_error"][i]) + self.a_bias[i]
            g = w[i] * (1 + c["gyro_scale_error"][i]) + self.g_bias[i]
            if noise:
                a += self.rng.gauss(0, self.a_sig)
                g += self.rng.gauss(0, self.g_sig)
            a = max(-32768, min(32767, round(a / self.a_lsb))) * self.a_lsb
            g = max(-32768, min(32767, round(g / self.g_lsb))) * self.g_lsb
            acc.append(a)
            gyro.append(g)
        return tuple(acc), tuple(gyro)


class RigEnv:
    """The 2-DOF rig as a step-by-step plant.

    config / overrides: same as Testbed (a .toml path or dict).
    dt: control period in s (0.002 = the board's 500 Hz telemetry).
    noise: False gives exact sensors (still quantised) for debugging a model.
    pwm_hz: ESC signal rate (the firmware uses 400 Hz)."""

    def __init__(self, config=None, overrides=None, dt=0.002, seed=None, noise=True,
                 pwm_hz=400.0):
        self.cfg = config_mod.load(config, overrides)
        self.dt = dt
        self.noise = noise
        self.pwm_hz = pwm_hz
        self.seed = self.cfg["sim"]["seed"] if seed is None else seed
        self.substep = self.cfg["sim"]["physics_dt"]
        self.n_sub = max(1, round(dt / self.substep))
        self._csv = None
        self.reset()

    def reset(self, q=(0.0, 0.0), dq=(0.0, 0.0), seed=None):
        """Start again with the frame at rest at q (rad), ESCs armed, motors stopped."""
        if seed is not None:
            self.seed = seed
        cfg = self.cfg
        self.rig = Rig(cfg, seed=self.seed)
        self.rig.yaw.q, self.rig.pitch.q = q
        self.rig.yaw.dq, self.rig.pitch.dq = dq
        for e in self.rig.escs:
            e.state = ESC.ARMED
            e.last_valid_t = 0.0
            e.pulse_us = 1000.0
        self.rng = random.Random(self.seed + 7)
        self.imu = _Imu(cfg["imu"], self.rng)
        c = cfg["encoders"]
        self.counts_per_rad = 4 * c["cpr"] / (2 * math.pi)
        self.enc_sign = (c["yaw_sign"], c["pitch_sign"])
        self.q0 = tuple(q)
        self.u = (1000.0, 1000.0, 1000.0, 1000.0)
        self._q_prev = self._encoders()
        return self._observe()

    # ---------------------------------------------------------------- stepping
    def step(self, u_us):
        """Apply ESC pulses u_us (4 values, us) for one period dt; returns Obs."""
        self.u = tuple(float(x) for x in u_us)
        sig = [(1.0 / self.pwm_hz, x * 1e-6) for x in self.u]
        rig = self.rig
        for _ in range(self.n_sub):
            rig.step(sig, self.substep)
        rig.notes.clear()
        obs = self._observe()
        if self._csv:
            self._csv_row(obs)
        return obs

    def rollout(self, u_seq):
        """Apply a sequence of 4-pulse commands; returns the list of Obs."""
        return [self.step(u) for u in u_seq]

    def push(self, yaw_torque=0.0, pitch_torque=0.0):
        """Extra torque on the joints (N m) until you call push() again."""
        self.rig.disturbance = [yaw_torque, pitch_torque]

    # ---------------------------------------------------------------- sensors
    def _encoders(self):
        rig = self.rig
        out = []
        for j, q0, sgn in ((rig.yaw, self.q0[0], self.enc_sign[0]),
                           (rig.pitch, self.q0[1], self.enc_sign[1])):
            counts = math.floor(sgn * (j.q - q0) * self.counts_per_rad)
            out.append(sgn * counts / self.counts_per_rad + q0)
        return tuple(out)

    def _observe(self):
        rig = self.rig
        c = self.cfg["imu"]
        q = self._encoders()
        dq = tuple((a - b) / self.dt for a, b in zip(q, self._q_prev))
        self._q_prev = q
        f, w = rig.imu_truth(c["position"], c["rotation"], c["vibration_ms2"] if self.noise else 0)
        acc, gyro = self.imu.read(f, w, self.dt, self.noise)
        return Obs(t=rig.t, q=q, dq=dq, acc=acc, gyro=gyro, motor_us=self.u,
                   true_q=rig.q, true_dq=rig.dq,
                   rpm=tuple(m.w * 60 / (2 * math.pi) for m in rig.motors),
                   v_bus=rig.v_bus, current_a=rig.total_current())

    # ---------------------------------------------------------------- logging
    COLUMNS = ["t", "q1", "q2", "dq1", "dq2", "ax", "ay", "az", "gx", "gy", "gz",
               "u1", "u2", "u3", "u4", "true_q1", "true_q2", "true_dq1", "true_dq2",
               "rpm1", "rpm2", "rpm3", "rpm4", "v_bus"]

    def log_csv(self, path):
        """Write every step to a CSV (same names as tbbridge.py where they overlap)."""
        self._csv_file = open(path, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(self.COLUMNS)

    def _csv_row(self, o):
        self._csv.writerow([f"{o.t:.4f}", *(f"{x:.6f}" for x in o.q), *(f"{x:.5f}" for x in o.dq),
                            *(f"{x:.5f}" for x in o.acc), *(f"{x:.6f}" for x in o.gyro),
                            *(f"{x:.0f}" for x in o.motor_us),
                            *(f"{x:.6f}" for x in o.true_q), *(f"{x:.5f}" for x in o.true_dq),
                            *(f"{x:.0f}" for x in o.rpm), f"{o.v_bus:.3f}"])

    def close(self):
        if self._csv:
            self._csv_file.close()
            self._csv = None
