"""ADC1-3 regular conversions (software start, single/continuous, scan, DMA),
and a minimal SPI so code that touches SPI doesn't hang."""
from __future__ import annotations

from ..core import PS_PER_S, Peripheral

SMP_CYCLES = [3, 15, 28, 56, 84, 112, 144, 480]
SR, CR1, CR2, SMPR1, SMPR2 = 0x00, 0x04, 0x08, 0x0C, 0x10
SQR1, SQR2, SQR3, DR = 0x2C, 0x30, 0x34, 0x4C


class ADCUnit:
    def __init__(self, n):
        self.n = n
        self.r = {}
        self.sr = 0
        self.dr = 0
        self.seq_i = 0
        self.ev = None


class ADC(Peripheral):
    """One block at 0x40012000: ADC1 at +0x000, ADC2 +0x100, ADC3 +0x200, common +0x300."""

    gate = "ADC1"

    def __init__(self, machine, channel_volts=None, vdda=3.3):
        self.channel_volts = channel_volts or (lambda unit, ch: 0.0)
        self.vdda = vdda
        super().__init__(machine, "ADC", 0x40012000)
        machine.dma_sources.setdefault(2, {})
        for k in (1, 2, 3):
            machine.dma_sources[2][f"ADC{k}"] = (lambda u=k: self._dma_req(u))

    def reset(self):
        for u in getattr(self, "u", []):
            if u is not None and u.ev:
                u.ev.cancel()
        self.u = [None, ADCUnit(1), ADCUnit(2), ADCUnit(3)]
        self.ccr = 0
        if self.m.nvic:
            self._irq()

    def _adcclk(self):
        pre = [2, 4, 6, 8][(self.ccr >> 16) & 3]
        return self.m.rcc.pclk2 / pre

    def _seq(self, u):
        n = ((u.r.get(SQR1, 0) >> 20) & 0xF) + 1
        chans = []
        for i in range(n):
            reg = SQR3 if i < 6 else (SQR2 if i < 12 else SQR1)
            chans.append((u.r.get(reg, 0) >> (5 * (i % 6))) & 0x1F)
        if not u.r.get(CR1, 0) >> 8 & 1:          # SCAN off: only the first one
            chans = chans[:1]
        return chans

    def _smp(self, u, ch):
        if ch < 10:
            return SMP_CYCLES[(u.r.get(SMPR2, 0) >> (3 * ch)) & 7]
        return SMP_CYCLES[(u.r.get(SMPR1, 0) >> (3 * (ch - 10))) & 7]

    def _start(self, u):
        if not u.r.get(CR2, 0) & 1:
            return
        u.sr |= 0x10                               # STRT
        u.seq_i = 0
        self._schedule(u)

    def _schedule(self, u):
        seq = self._seq(u)
        ch = seq[u.seq_i]
        res_bits = [12, 10, 8, 6][(u.r.get(CR1, 0) >> 24) & 3]
        cycles = self._smp(u, ch) + res_bits
        clk = self._adcclk()
        u.ev = self.m.after(int(cycles / clk * PS_PER_S) if clk else PS_PER_S,
                            lambda: self._done(u))

    def _done(self, u):
        u.ev = None
        seq = self._seq(u)
        ch = seq[u.seq_i]
        if ch == 16:
            v = 0.76 + 0.0025 * (30.0 - 25.0)      # temperature sensor at ~30 C
        elif ch == 17:
            v = 1.21                               # VREFINT
        elif ch == 18:
            v = 3.0 / 2                            # VBAT / 2 (coin cell)
        else:
            v = self.channel_volts(u.n, ch)
        res_bits = [12, 10, 8, 6][(u.r.get(CR1, 0) >> 24) & 3]
        full = (1 << res_bits) - 1
        code = int(round(max(0.0, min(v, self.vdda)) / self.vdda * full))
        if u.r.get(CR2, 0) >> 11 & 1:              # ALIGN left
            code <<= (16 - res_bits)
        if u.sr & 0x02 and not u.r.get(CR2, 0) >> 8 & 1:
            u.sr |= 0x20                           # OVR (only flagged without DMA here)
        u.dr = code
        last = u.seq_i == len(seq) - 1
        eocs = u.r.get(CR2, 0) >> 10 & 1
        if eocs or last:
            u.sr |= 0x02                           # EOC
        if u.r.get(CR2, 0) >> 8 & 1:
            self.m.dma[2].kick(f"ADC{u.n}") if 2 in self.m.dma else None
        self._irq()
        if not last:
            u.seq_i += 1
            self._schedule(u)
        elif u.r.get(CR2, 0) & 2:                  # CONT
            u.seq_i = 0
            self._schedule(u)
        else:
            u.sr &= ~0x10

    def _dma_req(self, n):
        u = self.u[n]
        return bool(u.r.get(CR2, 0) >> 8 & 1 and u.sr & 0x02 and u.dr is not None and
                    not getattr(u, "_dma_taken", False))

    def _irq(self):
        lvl = False
        for u in self.u[1:]:
            cr1 = u.r.get(CR1, 0)
            if (cr1 >> 5 & 1 and u.sr & 2) or (cr1 >> 26 & 1 and u.sr & 0x20):
                lvl = True
        self.m.nvic.set_irq(18, "ADC", lvl)

    def read(self, off):
        if off >= 0x300:
            if off == 0x304:
                return self.ccr
            if off == 0x300:
                return sum(self.u[k].sr << (8 * (k - 1)) for k in (1, 2, 3))
            return 0
        u = self.u[1 + off // 0x100]
        o = off % 0x100
        if o == SR:
            return u.sr
        if o == DR:
            u.sr &= ~0x02
            v = u.dr
            self._irq()
            return v
        return u.r.get(o, 0)

    def write(self, off, value, mask):
        if off >= 0x300:
            if off == 0x304:
                self.ccr = (self.ccr & ~mask) | (value & mask)
            return
        u = self.u[1 + off // 0x100]
        o = off % 0x100
        if o == SR:
            u.sr &= (value | ~mask)
            self._irq()
            return
        new = (u.r.get(o, 0) & ~mask) | (value & mask)
        if o == CR2:
            start = new >> 30 & 1
            if (new >> 28) & 3:
                self.m.warn("ADC external triggers (timer-started conversions) are not modelled")
            u.r[CR2] = new & ~(1 << 30)
            if not new & 1 and u.ev:
                u.ev.cancel()
                u.ev = None
            if start:
                self._start(u)
            return
        u.r[o] = new
        if o == CR1:
            self._irq()


class SPI(Peripheral):
    """No devices attached: transfers complete at once and read back 0xFF."""

    def __init__(self, machine, name, base):
        self.gate = name
        super().__init__(machine, name, base)

    def reset(self):
        self.r = {}
        self.sr = 0x02
        self.dr = 0xFF

    def read(self, off):
        if off == 0x08:
            return self.sr
        if off == 0x0C:
            self.sr &= ~0x01
            return self.dr
        return self.r.get(off, 0)

    def write(self, off, value, mask):
        if off == 0x0C:
            if not getattr(self, "_warned", False):
                self._warned = True
                self.m.warn(f"{self.name}: no SPI device is modelled; reads return 0xFF")
            self.sr |= 0x03
            self.dr = 0xFF if not (self.r.get(0, 0) >> 11 & 1) else 0xFFFF
            return
        if off == 0x08:
            return
        super().write(off, value, mask)
