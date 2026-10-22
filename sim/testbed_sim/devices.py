"""Chips and parts on the board that talk to the STM32: QMI8658 IMU, joint
encoders, the ESC signal input, keys, LED and buzzer."""
from __future__ import annotations

import math
import random

from .core import PS_PER_S
from .periph.i2c import I2CDevice

G0 = 9.80665
DEG = math.pi / 180

# QMI8658 output data rates (Hz): 6-DOF mode uses the gyro's natural frequency
ODR_6DOF = {0: 7174.4, 1: 3587.2, 2: 1793.6, 3: 896.8, 4: 448.4, 5: 224.2, 6: 112.1,
            7: 56.05, 8: 28.025}
ODR_ACC_ONLY = {3: 1000, 4: 500, 5: 250, 6: 125, 7: 62.5, 8: 31.25, 12: 128, 13: 21,
                14: 11, 15: 3}
LPF_PCT = [0.0266, 0.0363, 0.0539, 0.1337]


class QMI8658(I2CDevice):
    """Register-level model of the QMI8658A (datasheet Rev A, section 5)."""

    def __init__(self, machine, cfg, truth, set_int_pin, seed=1):
        self.m = machine
        self.cfg = cfg
        self.address = cfg["i2c_address"]
        self.truth = truth                    # fn(t_s) -> (spec_force[3] m/s^2, rate[3] rad/s) in chip axes
        self.set_int_pin = set_int_pin        # fn(level)
        self.rng = random.Random(seed + 101)
        self.gyro_bias = [b * DEG for b in cfg["gyro_bias_dps"]]
        self.acc_bias = [b * 1e-3 * G0 for b in cfg["accel_bias_mg"]]
        self.ev = None
        self.reads = 0
        self.power_on()

    # ---------------------------------------------------------------- state
    def power_on(self):
        if self.ev:
            self.ev.cancel()
        self.ev = None
        self.reg = bytearray(128)
        self.reg[0x00] = 0x05                 # WHO_AM_I
        self.reg[0x01] = 0x7C                 # REVISION_ID
        self.reg[0x02] = 0x20                 # CTRL1: big-endian, no auto-increment
        self.ptr = 0
        self.first_byte = False
        self.sample = [0] * 6
        self.temp = int(self.cfg["temperature_c"] * 256)
        self.ts = 0
        self.lpf = [0.0] * 6
        self.acc_on_t = None
        self.gyr_on_t = None
        self.last_t = None
        self.int_level = 0

    def soft_reset(self):
        self.power_on()
        self.m.after(int(15e-3 * PS_PER_S), self._reset_done, world=True)

    def _reset_done(self):
        self.reg[0x4D] = 0x80                 # "reset done" marker drivers poll for

    def _ctrl(self, n):
        return self.reg[{1: 0x02, 2: 0x03, 3: 0x04, 5: 0x06, 7: 0x08}[n]]

    def _odr(self):
        c7 = self._ctrl(7)
        a_on, g_on = c7 & 1, c7 >> 1 & 1
        if not (a_on or g_on):
            return None
        if g_on:
            code = (self._ctrl(3) & 0xF) if not a_on else (self._ctrl(3) & 0xF)
            return ODR_6DOF.get(code, ODR_6DOF[3])
        return ODR_ACC_ONLY.get(self._ctrl(2) & 0xF, 1000)

    def _reschedule(self):
        if self.ev:
            self.ev.cancel()
            self.ev = None
        odr = self._odr()
        if odr:
            self.ev = self.m.after(int(PS_PER_S / odr), self._new_sample, world=True)

    # ---------------------------------------------------------------- sampling
    def _new_sample(self):
        self.ev = None
        odr = self._odr()
        if not odr:
            return
        t = self.m.now_s()
        dt = 1.0 / odr
        cfg = self.cfg
        f, w = self.truth(t)
        c2, c3, c5, c7 = self._ctrl(2), self._ctrl(3), self._ctrl(5), self._ctrl(7)
        a_fs = 2 << ((c2 >> 4) & 3)                     # g
        g_fs = 16 << ((c3 >> 4) & 7)                    # dps
        a_sig = cfg["accel_noise_ug_rthz"] * 1e-6 * G0 * math.sqrt(odr / 2)
        g_sig = cfg["gyro_noise_mdps_rthz"] * 1e-3 * DEG * math.sqrt(odr / 2)
        walk = cfg["gyro_bias_walk_dps_rts"] * DEG * math.sqrt(dt)
        for i in range(3):
            self.gyro_bias[i] += self.rng.gauss(0, walk)
        out = [0] * 6
        a_ready = c7 & 1 and self.acc_on_t is not None and t >= self.acc_on_t
        g_ready = c7 >> 1 & 1 and self.gyr_on_t is not None and t >= self.gyr_on_t
        for i in range(3):
            a = (f[i] * (1 + cfg["accel_scale_error"][i]) + self.acc_bias[i]
                 + self.rng.gauss(0, a_sig))
            g = (w[i] * (1 + cfg["gyro_scale_error"][i]) + self.gyro_bias[i]
                 + self.rng.gauss(0, g_sig))
            if c5 & 1:                                  # accel LPF
                fc = LPF_PCT[(c5 >> 1) & 3] * odr
                al = 1 - math.exp(-2 * math.pi * fc * dt)
                self.lpf[i] += al * (a - self.lpf[i])
                a = self.lpf[i]
            if c5 >> 4 & 1:                             # gyro LPF
                fc = LPF_PCT[(c5 >> 5) & 3] * odr
                al = 1 - math.exp(-2 * math.pi * fc * dt)
                self.lpf[3 + i] += al * (g - self.lpf[3 + i])
                g = self.lpf[3 + i]
            out[i] = _q16(a / G0 * 32768 / a_fs) if a_ready else 0
            out[3 + i] = _q16(g / DEG * 32768 / g_fs) if g_ready else 0
        self.sample = out
        self.ts = (self.ts + 1) & 0xFFFFFF
        self.temp = int((cfg["temperature_c"] + self.rng.gauss(0, 0.05)) * 256)
        if a_ready:
            self.reg[0x2E] |= 1
        if g_ready:
            self.reg[0x2E] |= 2
        # DRDY on INT2 (or INT1), a short pulse per sample
        c1 = self._ctrl(1)
        src = cfg["int_source"]
        en = (c1 >> 4 & 1) if src == 2 else (c1 >> 3 & 1)
        if en and not (src == 2 and c7 >> 5 & 1) and (a_ready or g_ready):
            self.set_int_pin(1)
            self.m.after(int(PS_PER_S / odr / 4), lambda: self.set_int_pin(0), world=True)
        self._reschedule()

    # ---------------------------------------------------------------- registers
    def _read_reg(self, a):
        if 0x30 <= a <= 0x32:
            return (self.ts >> (8 * (a - 0x30))) & 0xFF
        be = self._ctrl(1) >> 5 & 1
        if 0x33 <= a <= 0x40:
            k = (a - 0x33) // 2
            hi = (a - 0x33) & 1
            if be:
                hi ^= 1
            v = self.temp if k == 0 else self.sample[k - 1]
            v &= 0xFFFF
            if a >= 0x35:
                self.reg[0x2E] = 0                      # data read: DA flags clear
            return (v >> 8) & 0xFF if hi else v & 0xFF
        if a == 0x2D:
            v = self.reg[0x2D] & 0x80
            return v | (self.int_level << 0)
        return self.reg[a & 0x7F]

    def _write_reg(self, a, v):
        if a in (0x00, 0x01) or 0x2D <= a <= 0x40:
            return                                      # read-only
        if a == 0x60:
            if v == 0xB0:
                self.soft_reset()
            return
        old7 = self.reg[0x08]
        self.reg[a & 0x7F] = v
        t = self.m.now_s()
        if a == 0x08:
            odr = self._odr() or 1000
            if v & 1 and not old7 & 1:
                self.acc_on_t = t + 3e-3 + 3 / odr      # accel turn-on time
            if v >> 1 & 1 and not old7 >> 1 & 1:
                self.gyr_on_t = t + 150e-3 + 3 / odr    # gyro turn-on time
            if not v & 1:
                self.acc_on_t = None
            if not v >> 1 & 1:
                self.gyr_on_t = None
            self._reschedule()
        elif a in (0x03, 0x04):
            self._reschedule()
        elif a == 0x0A:                                  # CTRL9 command
            if v:
                self.m.after(int(1e-3 * PS_PER_S), self._cmd_done, world=True)
            else:
                self.reg[0x2D] &= ~0x80

    def _cmd_done(self):
        self.reg[0x2D] |= 0x80

    # ---------------------------------------------------------------- I2C
    def i2c_start(self, read):
        self.first_byte = not read
        return True

    def i2c_write(self, byte):
        if self.first_byte:
            self.ptr = byte & 0x7F
            self.first_byte = False
        else:
            self._write_reg(self.ptr, byte)
            if self._ctrl(1) >> 6 & 1:
                self.ptr = (self.ptr + 1) & 0x7F
        return True

    def i2c_read(self):
        v = self._read_reg(self.ptr)
        self.reads += 1
        if self._ctrl(1) >> 6 & 1:                      # ADDR_AI
            self.ptr = (self.ptr + 1) & 0x7F
        return v


def _q16(x):
    v = int(round(x))
    if v > 32767:
        v = 32767
    elif v < -32768:
        v = -32768
    return v & 0xFFFF


class PulseMeter:
    """Measures a PWM signal on a GPIO pin from its edges (bit-banged ESC signal)."""

    def __init__(self):
        self.rise = None
        self.prev_rise = None
        self.high = None
        self.period = None
        self.last_edge = None

    def edge(self, level, t_ps):
        t = t_ps / PS_PER_S
        if level:
            if self.rise is not None:
                self.period = t - self.rise
            self.prev_rise, self.rise = self.rise, t
        elif self.rise is not None:
            self.high = t - self.rise
        self.last_edge = t

    def signal(self, t):
        if self.high is None or self.period is None or self.last_edge is None:
            return None
        if t - self.last_edge > max(3 * self.period, 0.03):
            return None
        return (self.period, self.high)
