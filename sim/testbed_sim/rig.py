"""rig.py - physics of the 2-DOF testbed: ESCs, motors, props, joints.

No firmware needed: this module is also the plant for the AI work. It is
plain Python (no numpy) so one step costs a few microseconds.

Frames: world z up. The yaw joint turns about world z by psi; the pitch joint
then turns about the body y axis by theta (body x forward, y left, z up along
the thrust). So R_body_to_world = Rz(psi) * Ry(theta), and a positive pitch
angle moves the nose (+x) down.

Equations of motion (Lagrange, frame rigid about the pivot):

    M11(theta) = J_arm + Jxx sin^2(theta) + Jzz cos^2(theta)
    M11 psi''  + 2 (Jxx - Jzz) s c theta' psi'          = tau_psi
    Jyy theta'' -   (Jxx - Jzz) s c psi'^2              = tau_theta + tau_gravity

    tau_theta = body-y torque, tau_psi = -s * tau_bx + c * tau_bz
    tau_body  = sum r_i x (0, 0, T_i) + (0, 0, -spin_i * Q_i) + rotor gyroscopic terms

Each motor is a DC-motor model fed by its ESC:

    I = (duty * V_bus - w / Kv) / R        (clamped >= 0 unless braking)
    J_r w' = Kt I - Kq w^2 - Kt I0

so the "ESC lag" comes out of the motor's own time constant (~40-80 ms with
these numbers) plus the ESC's PWM sampling and start-up delay.
"""
from __future__ import annotations

import math
import random

TWO_PI = 2 * math.pi


class Joint:
    def __init__(self, cfg):
        self.b = cfg["viscous"]
        self.fc = cfg["coulomb"]
        self.fs = cfg["stiction"]
        lim = cfg.get("limits") or []
        self.lim = (lim[0], lim[1]) if len(lim) == 2 else None
        self.q = cfg.get("start_angle", 0.0)
        self.dq = 0.0
        self.stuck = True
        self.k_stop = 60.0          # N m/rad, end-stop stiffness
        self.c_stop = 0.6           # N m s/rad, end-stop damping

    def stop_torque(self):
        if self.lim is None:
            return 0.0
        lo, hi = self.lim
        if self.q > hi:
            return -self.k_stop * (self.q - hi) - self.c_stop * max(self.dq, 0.0)
        if self.q < lo:
            return -self.k_stop * (self.q - lo) - self.c_stop * min(self.dq, 0.0)
        return 0.0


class Motor:
    def __init__(self, cfg_m, cfg_p, i):
        self.kv = cfg_m["kv_rpm_per_v"] * TWO_PI / 60.0      # rad/s per volt
        self.kt = 1.0 / self.kv                              # N m / A
        self.R = cfg_m["resistance_ohm"]
        self.i0 = cfg_m["no_load_current_a"]
        self.J = cfg_m["rotor_inertia"]
        self.x, self.y = cfg_m["positions"][i]
        self.spin = cfg_m["spin"][i]
        self.kT = cfg_p["thrust_coeff"]
        self.kQ = cfg_p["torque_coeff"]
        self.w = 0.0            # rad/s
        self.dw = 0.0
        self.current = 0.0
        self.angle = 0.0        # rotor angle, for vibration phase

    def step(self, duty, v_bus, brake, dt):
        v = duty * v_bus
        back = self.w / self.kv
        if duty > 0.0:
            i = (v - back) / self.R
            if i < 0.0 and not brake:
                i = 0.0
        elif brake:
            i = -back / self.R * 0.5            # damped light: windings shorted through FETs
        else:
            i = 0.0
        self.current = max(i, 0.0)
        torque = self.kt * i - self.kQ * self.w * self.w - (self.kt * self.i0 if self.w > 1 else 0.0)
        dw = torque / self.J
        w = self.w + dw * dt
        if w < 0.0:
            w = 0.0
            dw = (w - self.w) / dt
        self.dw = dw
        self.w = w
        self.angle = (self.angle + w * dt) % TWO_PI

    def thrust(self):
        return self.kT * self.w * self.w

    def drag_torque(self):
        return self.kQ * self.w * self.w


class ESC:
    """Behaviour of a hobby ESC between the PWM pulse and the motor duty cycle."""

    OFF, ARMING, ARMED, FAULT = "off", "arming", "armed", "no-signal"

    def __init__(self, cfg, i):
        self.i = i
        self.cfg = cfg
        self.state = self.ARMING
        self.low_since = None
        self.last_valid_t = None
        self.pulse_us = None
        self.throttle = 0.0
        self.running_since = None
        self.refused = False
        self.warned_signal = False
        self.note = None            # message for the log when the state changes

    def update(self, t, signal):
        """signal: None (no pulses) or (period_s, high_s). Returns duty 0..1."""
        c = self.cfg
        pulse = None
        if signal is not None:
            period, high = signal
            us = high * 1e6
            lo, hi = c["pulse_valid_us"]
            if lo <= us <= hi and 1.0 / period <= c["max_frame_hz"] * 1.02:
                pulse = us
            elif not self.warned_signal:
                self.warned_signal = True
                self.note = (f"ESC{self.i + 1} ignores its signal: {us:.0f} us pulses at "
                             f"{1 / period:.0f} Hz (it accepts {lo}-{hi} us, up to "
                             f"{c['max_frame_hz']} Hz)")
        if pulse is not None:
            self.last_valid_t = t
            self.pulse_us = pulse
        elif self.last_valid_t is not None and t - self.last_valid_t < c["signal_timeout_s"]:
            pulse = self.pulse_us                 # keep the last pulse for a short gap
        else:
            self.pulse_us = None

        if self.pulse_us is None:
            if self.state == self.ARMED:
                self._set(self.FAULT, "signal lost, motor stopped")
            elif self.state not in (self.ARMING, self.FAULT):
                self._set(self.ARMING, None)
            self.low_since = None
            self.throttle = 0.0
            self.running_since = None
            return 0.0

        if self.state in (self.ARMING, self.FAULT):
            if self.pulse_us < c["arm_below_us"]:
                if self.low_since is None:
                    self.low_since = t
                if t - self.low_since >= c["arm_time_s"]:
                    self._set(self.ARMED, "armed (beep beep)")
            else:
                self.low_since = None
                if not self.refused and self.pulse_us > c["arm_below_us"] + 50:
                    self.refused = True
                    self.note = (f"ESC{self.i + 1} won't arm: its pulse is {self.pulse_us:.0f} us; "
                                 f"it needs < {c['arm_below_us']} us for {c['arm_time_s']} s first")
            self.throttle = 0.0
            return 0.0

        lo, hi = c["throttle_min_us"], c["throttle_max_us"]
        thr = (self.pulse_us - lo) / (hi - lo)
        thr = min(max(thr, 0.0), 1.0)
        if thr <= 0.0:
            self.running_since = None
            self.throttle = 0.0
            return 0.0
        if self.running_since is None:
            self.running_since = t
        if t - self.running_since < c["startup_delay_s"]:
            self.throttle = 0.0
            return 0.0
        self.throttle = thr
        return thr

    def _set(self, state, note):
        if state != self.state:
            self.state = state
            if note:
                self.note = f"ESC{self.i + 1} {note}"


class Rig:
    """The mechanical testbed. Call step(signals, dt) with the four ESC signals."""

    def __init__(self, cfg, seed=None):
        self.cfg = cfg
        self.rng = random.Random(cfg["sim"]["seed"] if seed is None else seed)
        f = cfg["frame"]
        self.m = f["mass_kg"]
        self.l_cg = f["cg_below_pitch_axis_m"]
        self.x_cg = f["cg_forward_m"]
        self.Jxx, self.Jyy, self.Jzz = f["inertia_xx"], f["inertia_yy"], f["inertia_zz"]
        self.Jarm = f["yaw_arm_inertia"]
        self.yaw = Joint(cfg["joints"]["yaw"])
        self.pitch = Joint(cfg["joints"]["pitch"])
        self.motors = [Motor(cfg["motor"], cfg["prop"], i) for i in range(4)]
        self.escs = [ESC(cfg["esc"], i) for i in range(4)]
        self.p = cfg["power"]
        self.v_bus = self.p["volts"]
        self.t = 0.0
        self.ddq = [0.0, 0.0]        # last joint accelerations
        self.disturbance = [0.0, 0.0]   # extra torque on (yaw, pitch), N m, e.g. a push
        self.notes = []

    # ---------------------------------------------------------------- helpers
    @property
    def q(self):
        return (self.yaw.q, self.pitch.q)

    @property
    def dq(self):
        return (self.yaw.dq, self.pitch.dq)

    def total_current(self):
        return self.p["board_current_a"] + sum(m.current for m in self.motors)

    # ---------------------------------------------------------------- dynamics
    def step(self, signals, dt):
        """Advance by dt. signals: 4 x (None or (period_s, high_s)) from the ESC pins."""
        t = self.t
        duties = []
        for esc, sig in zip(self.escs, signals):
            duties.append(esc.update(t, sig))
            if esc.note:
                self.notes.append((t, esc.note))
                esc.note = None
        brake = self.cfg["esc"]["active_braking"]
        # supply: constant voltage until the current limit, then it sags
        v = self.p["volts"]
        for mot, d in zip(self.motors, duties):
            mot.step(d, self.v_bus, brake, dt)
        i_tot = self.total_current()
        lim = self.p["current_limit_a"]
        target = v if i_tot <= lim else max(v * lim / i_tot, 0.5)
        self.v_bus += (target - self.v_bus) * min(1.0, dt / 0.002)

        # body torques from the motors
        tbx = tby = tbz = 0.0
        hz = 0.0                                   # rotor angular momentum along body z
        for mot in self.motors:
            T = mot.thrust()
            tbx += mot.y * T
            tby -= mot.x * T
            tbz -= mot.spin * (mot.drag_torque() + mot.J * mot.dw)
            hz += mot.spin * mot.J * mot.w
        th, ps = self.pitch, self.yaw
        s, c = math.sin(th.q), math.cos(th.q)
        # body angular velocity
        wbx, wby = -s * ps.dq, th.dq                 # (wbz = c * dpsi is along H)
        # gyroscopic torque of the spinning rotors: -w x H, H = (0, 0, hz)
        tbx += -(wby * hz)
        tby += (wbx * hz)

        g = 9.81
        tau_g = self.m * g * (c * self.x_cg - self.l_cg * s)
        tau_th = tby + tau_g + th.stop_torque() + self.disturbance[1]
        tau_ps = -s * tbx + c * tbz + ps.stop_torque() + self.disturbance[0]

        dJ = self.Jxx - self.Jzz
        M11 = self.Jarm + self.Jxx * s * s + self.Jzz * c * c
        acc_ps = self._friction(ps, tau_ps - 2 * dJ * s * c * th.dq * ps.dq, M11, dt)
        acc_th = self._friction(th, tau_th + dJ * s * c * ps.dq * ps.dq, self.Jyy, dt)

        # semi-implicit Euler
        for j, a in ((ps, acc_ps), (th, acc_th)):
            j.dq += a * dt
            j.q += j.dq * dt
        self.ddq = [acc_ps, acc_th]
        self.t = t + dt

    @staticmethod
    def _friction(j, tau, inertia, dt):
        """Karnopp friction: stick below the stiction torque, slide above it."""
        v_eps = 1e-3
        if abs(j.dq) < v_eps:
            if abs(tau) <= j.fs:
                j.dq = 0.0
                j.stuck = True
                return 0.0
            j.stuck = False
            return (tau - math.copysign(j.fc, tau)) / inertia
        j.stuck = False
        f = -math.copysign(j.fc, j.dq) - j.b * j.dq
        a = (tau + f) / inertia
        # don't let friction reverse the motion within one step
        if (j.dq > 0 and j.dq + a * dt < 0) or (j.dq < 0 and j.dq + a * dt > 0):
            if abs(tau) <= j.fs:
                return -j.dq / dt
        return a

    # ---------------------------------------------------------------- kinematics for sensors
    def body_kinematics(self):
        """Body angular velocity and acceleration (body frame), and R (body->world)."""
        th, ps = self.pitch, self.yaw
        s, c = math.sin(th.q), math.cos(th.q)
        sp, cp = math.sin(ps.q), math.cos(ps.q)
        ddps, ddth = self.ddq
        w = (-s * ps.dq, th.dq, c * ps.dq)
        # derivative of w in the body frame
        dw = (-c * th.dq * ps.dq - s * ddps, ddth, -s * th.dq * ps.dq + c * ddps)
        # R = Rz(psi) Ry(theta)
        R = ((cp * c, -sp, cp * s),
             (sp * c, cp, sp * s),
             (-s, 0.0, c))
        return w, dw, R

    def imu_truth(self, position, rotation, vibration_ms2=0.0, dt_extra=0.0):
        """What an ideal IMU fixed on the frame measures: specific force (m/s^2)
        and angular rate (rad/s), in the chip's axes. position: chip offset from
        the pivot (body frame, m); rotation: rows of body -> chip axes."""
        w, dw, R = self.body_kinematics()
        r = position
        # acceleration of a point fixed on the body, pivot fixed: dw x r + w x (w x r)
        wxr = _cross(w, r)
        a = [x + y for x, y in zip(_cross(dw, r), _cross(w, wxr))]
        g_body = [R[2][0] * -9.81, R[2][1] * -9.81, R[2][2] * -9.81]   # R^T (0,0,-g)
        f = [a[i] - g_body[i] for i in range(3)]
        if vibration_ms2:                         # motor vibration
            for k, mot in enumerate(self.motors):
                if mot.w > 1:
                    ph = mot.angle + mot.w * dt_extra
                    sc = vibration_ms2 * (mot.w / 1000.0) ** 2
                    f[0] += sc * math.sin(ph + k)
                    f[1] += sc * 0.7 * math.cos(ph + 2 * k)
                    f[2] += sc * 0.4 * math.sin(2 * ph + k)
        Rm = rotation
        f_chip = [sum(Rm[i][j] * f[j] for j in range(3)) for i in range(3)]
        w_chip = [sum(Rm[i][j] * w[j] for j in range(3)) for i in range(3)]
        return f_chip, w_chip


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
