"""GPIO ports A-I and the EXTI controller."""
from __future__ import annotations

from ..core import Peripheral

PORTS = "ABCDEFGHI"
EXTI_IRQ = {0: 6, 1: 7, 2: 8, 3: 9, 4: 10}
for _l in range(5, 10):
    EXTI_IRQ[_l] = 23
for _l in range(10, 16):
    EXTI_IRQ[_l] = 40
EXTI_IRQ.update({16: 1, 17: 41, 18: 42, 21: 2, 22: 3})


class GPIO(Peripheral):
    def __init__(self, machine, port):
        self.port = port
        self.idx = PORTS.index(port)
        self.gate = "GPIO" + port
        self.ext = [None] * 16          # level driven by something outside the chip
        self.ext_pull = [None] * 16     # board pull-up (1) / pull-down (0) resistors
        self.listeners = []             # fn(port, pin, level, t_ps) on output changes
        self._idr = 0
        super().__init__(machine, "GPIO" + port, 0x40020000 + 0x400 * self.idx)

    def reset(self):
        if self.port == "A":
            self.r = {0x00: 0xA8000000, 0x08: 0x0C000000, 0x0C: 0x64000000}
        elif self.port == "B":
            self.r = {0x00: 0x00000280, 0x08: 0x000000C0, 0x0C: 0x00000100}
        else:
            self.r = {}
        self.r[0x14] = 0
        old_out = getattr(self, "_out_cache", 0)
        self._out_cache = 0
        self._idr = self._compute_idr()
        if old_out:
            for pin in range(16):
                if old_out >> pin & 1:
                    self._notify(pin, self.output_level(pin))

    # ---------------------------------------------------------------- pin state
    def mode(self, pin):
        return (self.r.get(0x00, 0) >> (2 * pin)) & 3       # 0 in, 1 out, 2 AF, 3 analog

    def af(self, pin):
        reg = 0x20 if pin < 8 else 0x24
        return (self.r.get(reg, 0) >> (4 * (pin % 8))) & 0xF

    def is_af(self, pin, af):
        return self.mode(pin) == 2 and self.af(pin) == af

    def open_drain(self, pin):
        return bool(self.r.get(0x04, 0) >> pin & 1)

    def pull(self, pin):
        return (self.r.get(0x0C, 0) >> (2 * pin)) & 3       # 0 none, 1 up, 2 down

    def output_level(self, pin):
        """Level the chip drives on a GPIO output pin, or None if it doesn't drive it."""
        if self.mode(pin) != 1:
            return None
        odr = self.r.get(0x14, 0) >> pin & 1
        if self.open_drain(pin) and odr:
            return None                                     # released
        return odr

    def pin_level(self, pin):
        """Electrical level on the pin."""
        drv = self.output_level(pin)
        if drv is not None:
            if self.ext[pin] is not None and self.ext[pin] != drv:
                return drv                                  # contention: chip wins here
            return drv
        if self.ext[pin] is not None:
            return self.ext[pin]
        if self.ext_pull[pin] is not None:
            return self.ext_pull[pin]
        p = self.pull(pin)
        if p == 1:
            return 1
        return 0

    def _compute_idr(self):
        v = 0
        for pin in range(16):
            if self.mode(pin) != 3 and self.pin_level(pin):
                v |= 1 << pin
        return v

    def _refresh(self):
        new = self._compute_idr()
        changed = new ^ self._idr
        self._idr = new
        if changed and self.m.exti:
            for pin in range(16):
                if changed >> pin & 1:
                    self.m.exti.edge(self.idx, pin, new >> pin & 1)
        out = 0
        for pin in range(16):
            lvl = self.output_level(pin)
            if lvl:
                out |= 1 << pin
        ch = out ^ self._out_cache
        self._out_cache = out
        if ch:
            for pin in range(16):
                if ch >> pin & 1:
                    self._notify(pin, out >> pin & 1)

    def _notify(self, pin, level):
        t = self.m.now()
        for fn in self.listeners:
            fn(self.port, pin, level, t)

    def set_input(self, pin, level):
        """Drive a pin from outside (None = stop driving)."""
        if self.ext[pin] != level:
            self.ext[pin] = level
            self._refresh()

    # ---------------------------------------------------------------- registers
    def read(self, off):
        if off == 0x10:
            return self._idr                                # kept up to date by _refresh
        if off == 0x18:
            return 0
        return self.r.get(off, 0)

    def write(self, off, value, mask):
        if off == 0x10:
            return                                          # IDR is read-only
        r = self.r
        if off == 0x18:                                     # BSRR
            v = value & mask
            old = r.get(0x14, 0)
            odr = ((old | (v & 0xFFFF)) & ~(v >> 16)) & 0xFFFF
            if odr == old:
                return
            r[0x14] = odr
        elif off == 0x1C:
            r[off] = value & mask
            return
        else:
            old = r.get(off, 0)
            new = (old & ~mask) | (value & mask)
            if off == 0x14:
                new &= 0xFFFF
            if new == old:
                return
            r[off] = new
        self._refresh()


class EXTI(Peripheral):
    def __init__(self, machine):
        super().__init__(machine, "EXTI", 0x40013C00)
        machine.exti = self

    def reset(self):
        self.r = {}
        self.pr = 0
        for irq in set(EXTI_IRQ.values()):
            if self.m.nvic:
                self.m.nvic.set_irq(irq, "EXTI", False)

    def _update_irq(self):
        nvic = self.m.nvic
        active = self.pr & self.r.get(0x00, 0)
        for irq in set(EXTI_IRQ.values()):
            lines = [l for l, i in EXTI_IRQ.items() if i == irq]
            lvl = any(active >> l & 1 for l in lines)
            nvic.set_irq(irq, "EXTI", lvl)

    def edge(self, port_idx, pin, rising):
        syscfg = self.m.syscfg
        if syscfg is None or syscfg.exti_port(pin) != port_idx:
            return
        trig = self.r.get(0x08 if rising else 0x0C, 0)
        if trig >> pin & 1:
            self.pr |= 1 << pin
            self._update_irq()

    def read(self, off):
        if off == 0x14:
            return self.pr
        return self.r.get(off, 0)

    def write(self, off, value, mask):
        v = value & mask
        if off == 0x14:
            self.pr &= ~v
        elif off == 0x10:
            self.pr |= v & self.r.get(0x00, 0)
            self.r[0x10] = 0
        else:
            super().write(off, value, mask)
        self._update_irq()
