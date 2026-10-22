"""General-purpose, advanced and basic timers (TIM1-TIM14).

Modelled: up/down counting with prescaler, auto-reload and preload (shadow)
registers, update events and interrupts, repetition counter, one-pulse mode,
compare-match interrupts, PWM / forced / toggle output modes on all four
channels (with MOE on TIM1/TIM8), and encoder mode (SMS = 1, 2, 3) fed by the
quadrature signal on the timer's input pins. Input capture, master/slave
triggers, DMA bursts and center-aligned counting are not modelled (you get a
warning if the firmware uses them).
"""
from __future__ import annotations

from ..core import PS_PER_S, Peripheral

# pin -> list of (timer, channel, af); channel 0 = ETR
TIMER_PINS = {}


def _add(tim, ch, af, *pins):
    for p in pins:
        TIMER_PINS.setdefault(p, []).append((tim, ch, af))


_add("TIM1", 1, 1, "PA8", "PE9"); _add("TIM1", 2, 1, "PA9", "PE11")
_add("TIM1", 3, 1, "PA10", "PE13"); _add("TIM1", 4, 1, "PA11", "PE14")
_add("TIM2", 1, 1, "PA0", "PA5", "PA15"); _add("TIM2", 2, 1, "PA1", "PB3")
_add("TIM2", 3, 1, "PA2", "PB10"); _add("TIM2", 4, 1, "PA3", "PB11")
_add("TIM3", 1, 2, "PA6", "PB4", "PC6"); _add("TIM3", 2, 2, "PA7", "PB5", "PC7")
_add("TIM3", 3, 2, "PB0", "PC8"); _add("TIM3", 4, 2, "PB1", "PC9")
_add("TIM4", 1, 2, "PB6", "PD12"); _add("TIM4", 2, 2, "PB7", "PD13")
_add("TIM4", 3, 2, "PB8", "PD14"); _add("TIM4", 4, 2, "PB9", "PD15")
_add("TIM5", 1, 2, "PA0", "PH10"); _add("TIM5", 2, 2, "PA1", "PH11")
_add("TIM5", 3, 2, "PA2", "PH12"); _add("TIM5", 4, 2, "PA3", "PI0")
_add("TIM8", 1, 3, "PC6", "PI5"); _add("TIM8", 2, 3, "PC7", "PI6")
_add("TIM8", 3, 3, "PC8", "PI7"); _add("TIM8", 4, 3, "PC9", "PI2")
_add("TIM9", 1, 3, "PA2", "PE5"); _add("TIM9", 2, 3, "PA3", "PE6")
_add("TIM10", 1, 3, "PB8", "PF6"); _add("TIM11", 1, 3, "PB9", "PF7")
_add("TIM12", 1, 9, "PB14", "PH6"); _add("TIM12", 2, 9, "PB15", "PH9")
_add("TIM13", 1, 9, "PA6", "PF8"); _add("TIM14", 1, 9, "PA7", "PF9")

# name: (base, bits, channels, advanced, {source: irq})
TIMERS = {
    "TIM1": (0x40010000, 16, 4, True, {"brk": 24, "up": 25, "trg": 26, "cc": 27}),
    "TIM8": (0x40010400, 16, 4, True, {"brk": 43, "up": 44, "trg": 45, "cc": 46}),
    "TIM2": (0x40000000, 32, 4, False, {"all": 28}),
    "TIM3": (0x40000400, 16, 4, False, {"all": 29}),
    "TIM4": (0x40000800, 16, 4, False, {"all": 30}),
    "TIM5": (0x40000C00, 32, 4, False, {"all": 50}),
    "TIM6": (0x40001000, 16, 0, False, {"all": 54}),
    "TIM7": (0x40001400, 16, 0, False, {"all": 55}),
    "TIM9": (0x40014000, 16, 2, False, {"all": 24}),
    "TIM10": (0x40014400, 16, 1, False, {"all": 25}),
    "TIM11": (0x40014800, 16, 1, False, {"all": 26}),
    "TIM12": (0x40001800, 16, 2, False, {"all": 43}),
    "TIM13": (0x40001C00, 16, 1, False, {"all": 44}),
    "TIM14": (0x40002000, 16, 1, False, {"all": 45}),
}

CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR = (
    0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30)
CCR = (None, 0x34, 0x38, 0x3C, 0x40)
BDTR = 0x44


class Timer(Peripheral):
    def __init__(self, machine, name):
        base, bits, nch, adv, irqs = TIMERS[name]
        self.gate = name
        self.bits, self.nch, self.adv, self.irqs = bits, nch, adv, irqs
        self.top = (1 << bits) - 1
        self.encoder_source = None       # fn() -> x4 quadrature count, set by the board
        self.ev_up = None
        self.ev_cc = [None] * 5
        self._warned = set()
        super().__init__(machine, name, base)
        machine.periph_clock_listeners.append(self._clock_changed)

    # ---------------------------------------------------------------- state
    def reset(self):
        for e in [getattr(self, "ev_up", None)] + list(getattr(self, "ev_cc", [])):
            if e:
                e.cancel()
        self.ev_up = None
        self.ev_cc = [None] * 5
        self.r = {ARR: self.top}
        self.sr = 0
        self.psc_sh = 0
        self.arr_sh = self.top
        self.ccr_sh = [0] * 5
        self.rep = 0
        self.cnt_ref = 0
        self.t_ref = 0
        self.enc_ref = 0
        self.enc_last_total = 0
        self.dir_down = False
        if self.m.nvic:
            self._update_irq()

    def _warn(self, what):
        if what not in self._warned:
            self._warned.add(what)
            self.m.warn(f"{self.name}: {what}")

    def f_tim(self):
        return self.m.rcc.timer_clock(self.name)

    def tick_ps(self):
        f = self.f_tim()
        return (self.psc_sh + 1) * PS_PER_S / f if f else float("inf")

    def running(self):
        return bool(self.r.get(CR1, 0) & 1)

    def encoder_mode(self):
        return self.r.get(SMCR, 0) & 7 in (1, 2, 3)

    # ---------------------------------------------------------------- counter
    def _elapsed_ticks(self, t=None):
        if not self.running():
            return 0
        t = self.m.now() if t is None else t
        return int((t - self.t_ref) / self.tick_ps())

    def counter(self):
        if self.encoder_mode():
            return self._encoder_cnt()
        n = self._elapsed_ticks()
        span = self.arr_sh + 1
        if self.r.get(CR1, 0) & 0x10:          # down-counting
            return (self.cnt_ref - n) % span
        return (self.cnt_ref + n) % span

    def _resync(self):
        """Freeze the current count as the new reference point."""
        if self.encoder_mode():
            self.cnt_ref = self._encoder_cnt()
            self.enc_ref = self._enc_total()
        else:
            self.cnt_ref = self.counter()
        self.t_ref = self.m.now()

    def _apply_shadows(self):
        self.psc_sh = self.r.get(PSC, 0)
        self.arr_sh = self.r.get(ARR, self.top)
        for ch in range(1, self.nch + 1):
            self.ccr_sh[ch] = self.r.get(CCR[ch], 0)

    def _ccr_preload(self, ch):
        reg = CCMR1 if ch <= 2 else CCMR2
        shift = 3 if ch % 2 else 11
        return bool(self.r.get(reg, 0) >> shift & 1)

    def _update_event(self, from_ug=False):
        """UEV: reload shadows, reset the counter, set UIF (unless suppressed)."""
        cr1 = self.r.get(CR1, 0)
        self._apply_shadows()
        self.cnt_ref = self.arr_sh if cr1 & 0x10 else 0
        self.t_ref = self.m.now()
        if self.adv:
            self.rep = self.r.get(RCR, 0)
        if not (from_ug and cr1 & 4):                 # URS: UG doesn't set UIF
            self.sr |= 1
            self._update_irq()
            self._dma("UP")

    def _overflow(self):
        self.ev_up = None
        if not self.running() or self.encoder_mode():
            return
        cr1 = self.r.get(CR1, 0)
        down = bool(cr1 & 0x10)
        # counter wraps
        self.t_ref = self.t_ref + (self.cnt_ref + 1 if down else self.arr_sh + 1 - self.cnt_ref) * self.tick_ps()
        if self.adv and self.rep > 0:
            self.rep -= 1
            self.cnt_ref = self.arr_sh if down else 0
        elif not cr1 & 2:                              # UDIS clear: UEV happens
            self._apply_shadows()
            if self.adv:
                self.rep = self.r.get(RCR, 0)
            self.cnt_ref = self.arr_sh if down else 0
            self.sr |= 1
            self._update_irq()
            self._dma("UP")
            if cr1 & 8:                                # one-pulse mode: stop
                self.r[CR1] = cr1 & ~1
                return
        else:
            self.cnt_ref = self.arr_sh if down else 0
        self._schedule()

    def _needs_update_events(self):
        dier = self.r.get(DIER, 0)
        if dier & 0x101:                               # UIE or UDE
            return True
        if self.r.get(CR1, 0) & 8:                     # one-pulse mode
            return True
        if self.r.get(PSC, 0) != self.psc_sh or self.r.get(ARR, self.top) != self.arr_sh:
            return True
        for ch in range(1, self.nch + 1):
            if self.r.get(CCR[ch], 0) != self.ccr_sh[ch]:
                return True
        if self.adv and self.r.get(RCR, 0):
            return True
        return False

    def _schedule(self):
        if self.ev_up:
            self.ev_up.cancel()
            self.ev_up = None
        for ch in range(1, 5):
            if self.ev_cc[ch]:
                self.ev_cc[ch].cancel()
                self.ev_cc[ch] = None
        if not self.running() or self.encoder_mode():
            return
        if self.r.get(SMCR, 0) & 7:
            self._warn("slave modes other than encoder mode are not modelled")
        if (self.r.get(CR1, 0) >> 5) & 3:
            self._warn("center-aligned counting is modelled as edge-aligned up-counting")
        tick = self.tick_ps()
        if tick == float("inf"):
            return
        down = bool(self.r.get(CR1, 0) & 0x10)
        if self._needs_update_events():
            left = (self.cnt_ref + 1) if down else (self.arr_sh + 1 - self.cnt_ref)
            self.ev_up = self.m.at(self.t_ref + left * tick, self._overflow)
        dier = self.r.get(DIER, 0)
        for ch in range(1, self.nch + 1):
            if dier >> ch & 1 or dier >> (8 + ch) & 1:
                ccr = self.ccr_sh[ch]
                if ccr <= self.arr_sh:
                    left = (self.cnt_ref - ccr) if down else (ccr - self.cnt_ref)
                    if left < 0 or (left == 0 and self.m.now() > self.t_ref):
                        left += self.arr_sh + 1
                    self.ev_cc[ch] = self.m.at(self.t_ref + left * tick,
                                               lambda c=ch: self._cc_match(c))

    def _cc_match(self, ch):
        self.ev_cc[ch] = None
        if self.sr >> ch & 1:
            self.sr |= 1 << (8 + ch)                  # overcapture flag
        self.sr |= 1 << ch
        self._update_irq()
        self._dma(f"CH{ch}")
        tick = self.tick_ps()
        if self.running() and tick != float("inf"):
            self.ev_cc[ch] = self.m.at(self.m.now() + (self.arr_sh + 1) * tick,
                                       lambda c=ch: self._cc_match(c))

    def _clock_changed(self):
        if self.running() and not self.encoder_mode():
            self._resync()
            self._schedule()

    # ---------------------------------------------------------------- encoder
    def _enc_total(self):
        if self.encoder_source is None:
            return 0
        x4 = self.encoder_source()
        sms = self.r.get(SMCR, 0) & 7
        ccer = self.r.get(CCER, 0)
        sign = -1 if ((ccer >> 1) & 1) ^ ((ccer >> 5) & 1) else 1
        if sms == 3:
            return sign * x4
        return sign * (x4 // 2)

    def _encoder_cnt(self):
        if not self.running():
            return self.cnt_ref
        arr = self.r.get(ARR, self.top)
        total = self._enc_total() - self.enc_ref
        return (self.cnt_ref + total) % (arr + 1)

    def poll_encoder(self):
        """Called by the board every physics step: overflow flags and direction bit."""
        if not (self.encoder_mode() and self.running()):
            return
        total = self._enc_total()
        last = self.enc_last_total
        self.enc_last_total = total
        if total == last:
            return
        arr = self.r.get(ARR, self.top)
        span = arr + 1
        a = self.cnt_ref + last - self.enc_ref
        b = self.cnt_ref + total - self.enc_ref
        if a // span != b // span:
            self.sr |= 1
            self._update_irq()
        cr1 = self.r.get(CR1, 0)
        self.r[CR1] = (cr1 | 0x10) if total < last else (cr1 & ~0x10)

    def check_encoder_setup(self):
        ccmr1 = self.r.get(CCMR1, 0)
        if ccmr1 & 3 != 1 or (ccmr1 >> 8) & 3 != 1:
            self._warn("encoder mode needs CC1S = 01 and CC2S = 01 (TI1/TI2 as inputs)")

    # ---------------------------------------------------------------- outputs
    def channel_output(self, ch):
        """What the timer drives on channel `ch`.

        Returns None (not driving), ('const', level) or ('pwm', period_s, high_s)."""
        if ch > self.nch:
            return None
        ccer = self.r.get(CCER, 0)
        if not ccer >> (4 * (ch - 1)) & 1:
            return None
        pol = (ccer >> (4 * (ch - 1) + 1)) & 1
        if self.adv and not self.r.get(BDTR, 0) & 0x8000:
            self._warn_once_moe()
            return None
        reg = CCMR1 if ch <= 2 else CCMR2
        shift = 0 if ch % 2 else 8
        ccmr = self.r.get(reg, 0) >> shift
        if ccmr & 3:
            return None                                 # channel is an input
        ocm = (ccmr >> 4) & 7
        f = self.f_tim()
        if not f:
            return ("const", pol)
        tick_s = (self.psc_sh + 1) / f
        period = (self.arr_sh + 1) * tick_s
        ccr = self.ccr_sh[ch]
        if ocm in (6, 7):
            if not self.running():
                ref = 1 if (ocm == 6) == (self.counter() < ccr) else 0
                return ("const", ref ^ pol)
            high_ticks = min(ccr, self.arr_sh + 1)
            if ocm == 7:
                high_ticks = self.arr_sh + 1 - high_ticks
            if pol:
                high_ticks = self.arr_sh + 1 - high_ticks
            if high_ticks <= 0:
                return ("const", 0)
            if high_ticks >= self.arr_sh + 1:
                return ("const", 1)
            return ("pwm", period, high_ticks * tick_s)
        if ocm == 5:
            return ("const", 1 ^ pol)
        if ocm == 4:
            return ("const", 0 ^ pol)
        if ocm == 3 and self.running():
            return ("pwm", 2 * period, period)
        return ("const", pol)

    def _warn_once_moe(self):
        if self.running():
            self._warn("advanced timer outputs stay off until MOE is set in BDTR "
                       "(HAL_TIM_PWM_Start does this; a raw register setup must set it)")

    # ---------------------------------------------------------------- IRQ / DMA
    def _update_irq(self):
        nvic = self.m.nvic
        if nvic is None:
            return
        dier = self.r.get(DIER, 0)
        act = self.sr & dier & 0xFF
        if "all" in self.irqs:
            nvic.set_irq(self.irqs["all"], self.name, bool(act))
        else:
            nvic.set_irq(self.irqs["up"], self.name, bool(act & 1))
            nvic.set_irq(self.irqs["cc"], self.name, bool(act & 0x1E))
            nvic.set_irq(self.irqs["trg"], self.name, bool(act & 0x60))
            nvic.set_irq(self.irqs["brk"], self.name, bool(act & 0x80))

    def _dma(self, what):
        dier = self.r.get(DIER, 0)
        bit = {"UP": 8, "CH1": 9, "CH2": 10, "CH3": 11, "CH4": 12}[what]
        if dier >> bit & 1:
            self._warn("timer DMA requests are not modelled")

    # ---------------------------------------------------------------- registers
    def read(self, off):
        if off == SR:
            return self.sr
        if off == CNT:
            return self.counter() & self.top
        if off == EGR:
            return 0
        if off == CR1:
            return self.r.get(CR1, 0)
        return self.r.get(off, 0)

    def write(self, off, value, mask):
        old = self.r.get(off, 0)
        new = (old & ~mask) | (value & mask)
        if off == SR:
            self.sr &= new | ~mask                     # rc_w0: writing 0 clears
            self._update_irq()
            return
        if off == EGR:
            if value & 1:                              # UG
                self._update_event(from_ug=True)
                if self.encoder_mode():
                    self.enc_ref = self._enc_total()
                    self.cnt_ref = 0
                self._schedule()
            for ch in range(1, 5):
                if value >> ch & 1:
                    self.sr |= 1 << ch
            self._update_irq()
            return
        if off == CNT:
            self._resync()
            self.cnt_ref = new & self.top
            if self.encoder_mode():
                self.enc_ref = self._enc_total()
            self.t_ref = self.m.now()
            self.r[CNT] = new
            self._schedule()
            return
        if off == CR1:
            was_running = self.running()
            if was_running:
                self._resync()
            self.r[CR1] = new
            if not was_running and new & 1:
                self.t_ref = self.m.now()
                if self.encoder_mode():
                    self.check_encoder_setup()
                    self.enc_ref = self._enc_total()
                    self.enc_last_total = self.enc_ref
            elif was_running and not new & 1:
                pass
            if not new & 0x80:                         # ARPE clear: ARR is not buffered
                self.arr_sh = self.r.get(ARR, self.top)
            self._schedule()
            return
        if off == ARR:
            self.r[ARR] = new & self.top
            if not self.r.get(CR1, 0) & 0x80:
                if self.running() and not self.encoder_mode():
                    self._resync()
                self.arr_sh = self.r[ARR]
            self._schedule()
            return
        if off == PSC:
            self.r[PSC] = new & 0xFFFF
            self._schedule()
            return
        if off in CCR[1:]:
            ch = CCR.index(off)
            self.r[off] = new & self.top
            if not self._ccr_preload(ch):
                self.ccr_sh[ch] = self.r[off]
            self._schedule()
            return
        if off == SMCR:
            if self.running():
                self._resync()
            self.r[SMCR] = new
            if self.encoder_mode():
                self.enc_ref = self._enc_total()
                self.enc_last_total = self.enc_ref
            self._schedule()
            return
        if off == DIER:
            self.r[DIER] = new
            self._update_irq()
            self._schedule()
            return
        if off in (CCMR1, CCMR2, CCER, BDTR, RCR, CR2):
            self.r[off] = new
            if off in (CCMR1, CCMR2) and not self.encoder_mode():
                for ch in range(1, self.nch + 1):
                    reg = CCMR1 if ch <= 2 else CCMR2
                    if reg == off:
                        sh = 0 if ch % 2 else 8
                        if (new >> sh) & 3 and self.r.get(CCER, 0) >> (4 * (ch - 1)) & 1:
                            self._warn("input capture is not modelled")
            return
        self.r[off] = new
