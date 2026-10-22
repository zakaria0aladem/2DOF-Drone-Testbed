"""board.py - the Hiwonder ROS Robot Controller v1.2 with the testbed attached.

Puts the STM32F407 peripherals together and wires the outside world to the
pins, the way the real board is wired:

    USB-C port 1 (CH9102F) <-> USART1 PA9/PA10      USB-C port 2 <-> USART3 PD8/PD9
    QMI8658 on I2C2 PB10/PB11, INT -> PB12           KEY1 PE1, KEY2 PE0 (pull-ups)
    LED PE10 (active low), buzzer PA8                battery divider -> PB0 (ADC1_IN8)
    ESC1..4 signal on PA11, PB4, PC8, PC9            yaw encoder PA0/PA1, pitch PA15/PB3

A signal only gets through if the firmware configured the pin for it (the
right alternate function), exactly like on hardware.
"""
from __future__ import annotations

import math

from .core import PS_PER_S, Machine, Unmodeled
from .devices import QMI8658, PulseMeter
from .periph.adc import ADC, SPI
from .periph.dma import DMA
from .periph.gpio import EXTI, GPIO, PORTS
from .periph.i2c import I2C
from .periph.rcc import CRC, IWDG, PWR, RCC, RNG, SYSCFG, FlashIF
from .periph.scs import DBGMCU, DWT, ITM, SCS
from .periph.tim import TIMER_PINS, TIMERS, Timer
from .periph.usart import UARTS, USART
from .rig import Rig

UNMODELED = [("RTC", 0x40002800), ("WWDG", 0x40002C00), ("I2S2ext", 0x40003400),
             ("I2S3ext", 0x40004000), ("CAN1", 0x40006400), ("CAN2", 0x40006800),
             ("DAC", 0x40007400), ("SDIO", 0x40012C00), ("ETH", 0x40028000, 0x1400),
             ("USB_OTG_HS", 0x40040000, 0x40000), ("USB_OTG_FS", 0x50000000, 0x40000),
             ("DCMI", 0x50050000), ("FSMC", 0xA0000000), ("TPIU", 0xE0040000, 0x1000)]

USB_PORTS = {"USART1": ("PA9", "PA10", 7), "USART3": ("PD8", "PD9", 7)}


def pin_split(name):
    return name[1], int(name[2:])


class Board:
    def __init__(self, cfg, log=print):
        self.cfg = cfg
        self.log = log
        m = self.m = Machine(log)
        b = cfg["board"]

        m.add(SCS(m))
        m.add(DWT(m))
        self.swo = bytearray()
        m.add(ITM(m, lambda port, ch: self.swo.append(ch)))
        m.add(DBGMCU(m))
        self.rcc = m.add(RCC(m, b["hse_hz"], b["lse"]))
        m.add(FlashIF(m))
        m.add(PWR(m))
        m.add(SYSCFG(m))
        m.add(EXTI(m))
        m.add(IWDG(m, b["lsi_hz"]))
        m.add(CRC(m))
        m.add(RNG(m))
        self.gpio = {p: m.add(GPIO(m, p)) for p in PORTS}
        self.tim = {n: m.add(Timer(m, n)) for n in TIMERS}
        self.uart = {n: m.add(USART(m, n)) for n in UARTS}
        self.i2c = {n: m.add(I2C(m, n)) for n in ("I2C1", "I2C2", "I2C3")}
        self.dma = {k: m.add(DMA(m, k)) for k in (1, 2)}
        self.adc = m.add(ADC(m, self._adc_volts))
        for name, base in (("SPI1", 0x40013000), ("SPI2", 0x40003800), ("SPI3", 0x40003C00)):
            m.add(SPI(m, name, base))
        for item in UNMODELED:
            m.add(Unmodeled(m, item[0], item[1], item[2] if len(item) > 2 else 0x400))
        for p in list(self.gpio.values()) + list(self.tim.values()) + list(self.uart.values()) \
                + list(self.i2c.values()) + list(self.dma.values()):
            self.rcc.periphs[p.gate] = p
        self.rcc.periphs["ADC1"] = self.adc

        self.rig = Rig(cfg)
        self._diag_n = 0
        self._wire_keys_led_buzzer()
        self._wire_usb()
        self._wire_imu()
        self._wire_encoders()
        self._wire_escs()
        self.esc_pins = cfg["esc"]["signal_pins"]
        self.physics_dt = cfg["sim"]["physics_dt"]
        # physics substeps per event: the ESCs only see a new pulse every 2.5 ms,
        # so sampling their inputs once per millisecond loses nothing
        self.physics_batch = max(1, int(round(cfg["sim"]["physics_event_s"] / self.physics_dt)))
        self.on_physics = []                 # fn(board) after every physics step
        m.reset_hooks.append(self._on_reset)
        m.at(0, self._physics_step, world=True)

    # ---------------------------------------------------------------- pins
    def pin(self, name):
        port, n = pin_split(name)
        return self.gpio[port], n

    def pin_af_is(self, name, af):
        g, n = self.pin(name)
        return g.is_af(n, af)

    def timer_on_pin(self, name):
        """(timer, channel) the firmware routed to this pin, or None."""
        g, n = self.pin(name)
        if g.mode(n) != 2:
            return None
        af = g.af(n)
        for tim, ch, a in TIMER_PINS.get(name, []):
            if a == af:
                return self.tim[tim], ch
        return None

    # ---------------------------------------------------------------- simple parts
    def _wire_keys_led_buzzer(self):
        b = self.cfg["board"]
        for key in ("key1_pin", "key2_pin"):
            g, n = self.pin(b[key])
            g.ext_pull[n] = 1                    # 10 k pull-up on the board
            g._refresh()
        self.led_on = False
        self.led_changes = 0
        g, self._led_n = self.pin(b["led_pin"])
        self._led_port = g.port
        g.listeners.append(self._gpio_event)
        self.buzzer = PulseMeter()
        gb, self._bz_n = self.pin(b["buzzer_pin"])
        self._bz_port = gb.port
        if gb is not g:
            gb.listeners.append(self._gpio_event)
        self.buzzer_hz = 0.0
        self._bz_reported = 0.0

    def _gpio_event(self, port, pin, level, t):
        if port == self._led_port and pin == self._led_n:
            on = level == 0                       # active low
            if on != self.led_on:
                self.led_on = on
                self.led_changes += 1
        if port == self._bz_port and pin == self._bz_n:
            self.buzzer.edge(level, t)

    def press_key(self, key, pressed=True):
        g, n = self.pin(self.cfg["board"]["key1_pin" if key == 1 else "key2_pin"])
        g.set_input(n, 0 if pressed else None)

    def _adc_volts(self, unit, ch):
        name = self.cfg["board"]["battery_adc_pin"]
        g, n = self.pin(name)
        if unit in (1, 2) and ch == 8 and g.mode(n) == 3:
            return self.rig.v_bus / self.cfg["board"]["battery_divider"]
        return 0.0

    # ---------------------------------------------------------------- USB serial
    def _wire_usb(self):
        self.usb_tx = {n: bytearray() for n in USB_PORTS}      # board -> PC bytes, not yet read
        self.usb_listeners = {n: [] for n in USB_PORTS}       # fn(bytes) for live bridges
        for name, (tx, rx, af) in USB_PORTS.items():
            u = self.uart[name]
            u.tx_connected = (lambda tx=tx, af=af: self.pin_af_is(tx, af))
            u.rx_connected = (lambda rx=rx, af=af: self.pin_af_is(rx, af))
            u.tx_sink = (lambda byte, name=name: self._usb_out(name, byte))

    def _usb_out(self, name, byte):
        self.usb_tx[name].append(byte)
        for fn in self.usb_listeners[name]:
            fn(byte)

    def usb_send(self, name, data):
        self.uart[name].host_send(bytes(data))

    def usb_read(self, name):
        data = bytes(self.usb_tx[name])
        self.usb_tx[name].clear()
        return data

    # ---------------------------------------------------------------- IMU
    def _wire_imu(self):
        c = self.cfg["imu"]
        self.imu_R = c["rotation"]
        self.imu_pos = c["position"]
        g, n = self.pin(c["int_pin"])
        self.imu = QMI8658(self.m, c, self._imu_truth,
                           lambda lvl, g=g, n=n: g.set_input(n, lvl), self.cfg["sim"]["seed"])
        i2c = self.i2c["I2C2"]
        i2c.attach(self.imu)
        i2c.connected = lambda: self.pin_af_is("PB10", 4) and self.pin_af_is("PB11", 4)

    def _imu_truth(self, t):
        """Specific force and angular rate at the IMU, in the chip's axes."""
        return self.rig.imu_truth(self.imu_pos, self.imu_R, self.cfg["imu"]["vibration_ms2"],
                                  t - self.rig.t)

    # ---------------------------------------------------------------- encoders
    def _wire_encoders(self):
        c = self.cfg["encoders"]
        self.enc_counts_per_rev = 4 * c["cpr"]
        self.encoders = {"yaw": (c["yaw_pins"], c["yaw_sign"], 0),
                         "pitch": (c["pitch_pins"], c["pitch_sign"], 1)}
        self._enc_x4 = {"yaw": 0, "pitch": 0}
        self._enc_q0 = {"yaw": self.rig.yaw.q, "pitch": self.rig.pitch.q}
        for tname, tim in self.tim.items():
            if tim.nch >= 2:
                tim.encoder_source = (lambda t=tim: self._encoder_for_timer(t))

    def encoder_x4(self, joint):
        pins, sign, idx = self.encoders[joint]
        j = self.rig.yaw if idx == 0 else self.rig.pitch
        dt = self.m.now_s() - self.rig.t
        q = j.q + j.dq * max(dt, 0.0)
        return int(math.floor(sign * (q - self._enc_q0[joint]) * self.enc_counts_per_rev
                              / (2 * math.pi)))

    def _encoder_for_timer(self, tim):
        for joint, (pins, sign, idx) in self.encoders.items():
            a = self.timer_on_pin(pins[0])
            b = self.timer_on_pin(pins[1])
            if a and b and a[0] is tim and b[0] is tim and {a[1], b[1]} == {1, 2}:
                x4 = self.encoder_x4(joint)
                return x4 if a[1] == 1 else -x4
        return 0

    def _update_encoder_pins(self):
        for joint, (pins, sign, idx) in self.encoders.items():
            x4 = self.encoder_x4(joint)
            if x4 == self._enc_x4[joint]:
                continue
            self._enc_x4[joint] = x4
            ph = x4 % 4
            A = 1 if ph in (1, 2) else 0
            B = 1 if ph in (2, 3) else 0
            for pin, lvl in zip(pins, (A, B)):
                g, n = self.pin(pin)
                g.set_input(n, lvl)

    # ---------------------------------------------------------------- ESCs
    def _wire_escs(self):
        self.esc_meters = []
        for name in self.cfg["esc"]["signal_pins"]:
            g, n = self.pin(name)
            pm = PulseMeter()
            self.esc_meters.append(pm)
            g.listeners.append(lambda port, pin, lvl, t, pm=pm, gp=g.port, n=n:
                               pm.edge(lvl, t) if (port == gp and pin == n) else None)

    def _esc_diagnostics(self):
        """Explain an ESC pin that carries no usable signal (checked twice a second)."""
        if self.m.now_s() < 2.0:
            return                                  # the firmware may still be starting
        for i, name in enumerate(self.cfg["esc"]["signal_pins"]):
            if self.esc_signal(i) is not None:
                continue
            g, n = self.pin(name)
            mode = g.mode(n)
            if mode == 2:
                tc = self.timer_on_pin(name)
                if tc is None:
                    self.m.warn(f"ESC{i + 1} pin {name} is set to alternate function AF{g.af(n)}, "
                                f"which is no timer channel on {name}: the ESC gets no pulses")
                else:
                    tim, ch = tc
                    out = tim.channel_output(ch)
                    why = ("the timer isn't running" if not tim.running() else
                           f"channel {ch} isn't making PWM (check the mode, CCER enable, CCR)")
                    if out and out[0] == "const":
                        why = (f"its output is stuck {'high' if out[1] else 'low'} "
                               f"(CCR = 0 or >= ARR, or PWM mode not set)")
                    self.m.warn(f"ESC{i + 1} pin {name} is {tim.name} CH{ch}, but {why}")
            elif mode in (0, 3):
                self.m.warn(f"ESC{i + 1} pin {name} is still an input: nothing drives the ESC")

    def esc_signal(self, i):
        """What ESC i sees on its signal wire: None or (period_s, high_s)."""
        name = self.cfg["esc"]["signal_pins"][i]
        g, n = self.pin(name)
        mode = g.mode(n)
        if mode == 2:
            tc = self.timer_on_pin(name)
            if tc is None:
                return None
            out = tc[0].channel_output(tc[1])
            if out and out[0] == "pwm":
                return out[1], out[2]
            return None
        if mode == 1:
            return self.esc_meters[i].signal(self.m.now_s())
        return None

    # ---------------------------------------------------------------- physics
    def _physics_step(self):
        dt = self.physics_dt
        rig = self.rig
        sig = [self.esc_signal(i) for i in range(4)]
        for _ in range(self.physics_batch):
            rig.step(sig, dt)
        for t, note in rig.notes:
            self.log(f"[sim] t={t:.3f} s: {note}")
        rig.notes.clear()
        self._diag_n += 1
        if self._diag_n % 500 == 0:
            self._esc_diagnostics()
        self._update_encoder_pins()
        for tim in (self.tim["TIM2"], self.tim["TIM5"], self.tim["TIM3"], self.tim["TIM4"]):
            tim.poll_encoder()
        self._buzzer_check()
        for fn in self.on_physics:
            fn(self)
        self.m.after(int(dt * self.physics_batch * PS_PER_S), self._physics_step, world=True)

    def _buzzer_check(self):
        name = self.cfg["board"]["buzzer_pin"]
        hz = 0.0
        tc = self.timer_on_pin(name)
        if tc is not None:
            out = tc[0].channel_output(tc[1])
            if out and out[0] == "pwm":
                hz = 1.0 / out[1]
        else:
            s = self.buzzer.signal(self.m.now_s())
            if s:
                hz = 1.0 / s[0]
        if (hz > 0) != (self._bz_reported > 0) or (hz and abs(hz - self._bz_reported) > 0.2 * hz):
            if hz:
                self.log(f"[sim] t={self.m.now_s():.3f} s: buzzer sounding at {hz:.0f} Hz")
            else:
                self.log(f"[sim] t={self.m.now_s():.3f} s: buzzer quiet")
            self._bz_reported = hz
        self.buzzer_hz = hz

    def _on_reset(self, reason):
        # incremental encoders: the timer counts restart from 0 at the current pose
        pass
