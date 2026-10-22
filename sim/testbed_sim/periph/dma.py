"""DMA1 / DMA2 (STM32F4 stream controller).

Peripheral-to-memory, memory-to-peripheral (request driven) and
memory-to-memory transfers, circular mode, double-buffer mode, half/full
transfer flags and interrupts. FIFO packing between different sizes is
simplified: each request moves one peripheral-sized item.
"""
from __future__ import annotations

from ..core import Peripheral

# (controller, stream, channel) -> request name
REQUESTS = {
    (1, 0, 0): "SPI3_RX", (1, 2, 0): "SPI3_RX", (1, 3, 0): "SPI2_RX", (1, 4, 0): "SPI2_TX",
    (1, 5, 0): "SPI3_TX", (1, 7, 0): "SPI3_TX",
    (1, 0, 1): "I2C1_RX", (1, 5, 1): "I2C1_RX", (1, 6, 1): "I2C1_TX", (1, 7, 1): "I2C1_TX",
    (1, 2, 3): "I2C3_RX", (1, 4, 3): "I2C3_TX",
    (1, 0, 4): "UART5_RX", (1, 1, 4): "USART3_RX", (1, 2, 4): "UART4_RX", (1, 3, 4): "USART3_TX",
    (1, 4, 4): "UART4_TX", (1, 5, 4): "USART2_RX", (1, 6, 4): "USART2_TX", (1, 7, 4): "UART5_TX",
    (1, 2, 7): "I2C2_RX", (1, 3, 7): "I2C2_RX", (1, 4, 7): "USART3_TX", (1, 7, 7): "I2C2_TX",
    (2, 0, 0): "ADC1", (2, 4, 0): "ADC1", (2, 2, 1): "ADC2", (2, 3, 1): "ADC2",
    (2, 0, 2): "ADC3", (2, 1, 2): "ADC3",
    (2, 0, 3): "SPI1_RX", (2, 2, 3): "SPI1_RX", (2, 3, 3): "SPI1_TX", (2, 5, 3): "SPI1_TX",
    (2, 2, 4): "USART1_RX", (2, 5, 4): "USART1_RX", (2, 7, 4): "USART1_TX",
    (2, 1, 5): "USART6_RX", (2, 2, 5): "USART6_RX", (2, 6, 5): "USART6_TX", (2, 7, 5): "USART6_TX",
}
IRQS = {1: [11, 12, 13, 14, 15, 16, 17, 47], 2: [56, 57, 58, 59, 60, 68, 69, 70]}
FLAG_SHIFT = [0, 6, 16, 22]


class Stream:
    def __init__(self):
        self.cr = 0
        self.ndtr = 0
        self.par = 0
        self.m0ar = 0
        self.m1ar = 0
        self.fcr = 0x21
        self.flags = 0          # bit0 FEIF, 2 DMEIF, 3 TEIF, 4 HTIF, 5 TCIF
        self.burst = None       # (t0, ps per item, n) while a fast UART burst runs
        self.n0 = 0             # NDTR at enable time
        self.pi = 0             # items done since (re)load
        self.half_done = False


class DMA(Peripheral):
    def __init__(self, machine, unit):
        self.unit = unit
        self.gate = f"DMA{unit}"
        # request name -> fn() -> bool (request line level); peripherals fill it in
        self.sources = machine.dma_sources.setdefault(unit, {})
        self._busy = False
        super().__init__(machine, f"DMA{unit}", 0x40026000 if unit == 1 else 0x40026400)
        machine.dma[unit] = self

    def reset(self):
        self.s = [Stream() for _ in range(8)]
        if self.m.nvic:
            for i in range(8):
                self._irq(i)

    # ---------------------------------------------------------------- engine
    def _irq(self, i):
        st = self.s[i]
        en = 0
        if st.cr & 0x10:
            en |= 0x20
        if st.cr & 0x08:
            en |= 0x10
        if st.cr & 0x04:
            en |= 0x08
        if st.cr & 0x02:
            en |= 0x04
        if st.fcr & 0x80:
            en |= 0x01
        self.m.nvic.set_irq(IRQS[self.unit][i], self.name, bool(st.flags & en))

    def request_name(self, i):
        ch = (self.s[i].cr >> 25) & 7
        return REQUESTS.get((self.unit, i, ch))

    def remaining(self, req):
        for i, st in enumerate(self.s):
            if st.cr & 1 and self.request_name(i) == req:
                return st.ndtr
        return None

    def active_stream(self, req, direction):
        for i, st in enumerate(self.s):
            if st.cr & 1 and (st.cr >> 6) & 3 == direction and self.request_name(i) == req:
                return i
        return None

    def put_burst(self, i, data):
        """Fast path for a UART->memory stream: store bytes that arrived back
        to back, as the stream would have one by one. Returns how many it took."""
        st = self.s[i]
        m = self.m
        taken = 0
        while taken < len(data) and st.cr & 1:
            cr = st.cr
            if (cr >> 11) & 3 or (cr >> 13) & 3 or not cr >> 10 & 1:
                break                                  # only byte-wide, incrementing
            base = st.m1ar if (cr >> 18 & 1 and cr >> 19 & 1) else st.m0ar
            k = min(len(data) - taken, st.ndtr)
            if not st.half_done and st.ndtr - k < st.n0 // 2 < st.ndtr:
                k = st.ndtr - st.n0 // 2               # stop at the half-way point first
            addr = base + st.pi
            if m._pages.get(addr >> 10) is None:
                try:
                    m.uc.mem_write(addr, bytes(data[taken:taken + k]))
                except Exception:
                    m.warn(f"DMA write to unmapped address 0x{addr:08X}")
            else:
                for j in range(k):
                    m.bus_write(addr + j, data[taken + j], 1)
            st.pi += k
            st.ndtr -= k
            taken += k
            if not st.half_done and st.ndtr <= st.n0 // 2:
                st.half_done = True
                st.flags |= 0x10
            if st.ndtr <= 0:
                self._end_of_block(i)
            self._irq(i)
        return taken

    def _end_of_block(self, i):
        st = self.s[i]
        cr = st.cr
        st.flags |= 0x20
        if cr & 0x100 or cr & 0x40000:     # circular or double buffer
            st.ndtr = st.n0
            st.pi = 0
            st.half_done = False
            if cr & 0x40000:
                st.cr ^= 1 << 19           # switch target buffer
        else:
            st.cr &= ~1

    def take_burst(self, i, item_ps):
        """Fast path for a memory->UART stream: hand over all remaining bytes now.

        The data is read at once; NDTR, the HT/TC flags and EN then follow the
        time each byte would really have been moved (one per UART frame)."""
        st = self.s[i]
        cr = st.cr
        n = st.ndtr
        psize = 1 << ((cr >> 11) & 3)
        mstep = psize if cr >> 10 & 1 else 0
        base = st.m1ar if (cr >> 18 & 1 and cr >> 19 & 1) else st.m0ar
        if psize == 1 and mstep == 1 and self.m._pages.get((base + st.pi) >> 10) is None:
            try:
                data = list(self.m.uc.mem_read(base + st.pi, n))
            except Exception:
                data = [self.m.bus_read(base + st.pi + k, 1) for k in range(n)]
        else:
            data = [self.m.bus_read(base + (st.pi + k) * mstep, psize) for k in range(n)]
        t0 = self.m.now()
        st.burst = (t0, item_ps, n)
        st.pi += n
        if not cr & 0x100:
            half_t = t0 + max(0, n // 2 - 1) * item_ps
            end_t = t0 + max(0, n - 2) * item_ps

            def half():
                if not st.half_done:
                    st.half_done = True
                    st.flags |= 0x10
                    self._irq(i)

            def end():
                st.burst = None
                st.ndtr = 0
                st.flags |= 0x20
                if not st.half_done:
                    st.flags |= 0x10
                st.cr &= ~1
                self._irq(i)
            if n >= 2:
                self.m.at(half_t, half)
            self.m.at(end_t, end)
        return data

    def kick(self, req):
        """A peripheral's request line went active: service matching streams."""
        if self._busy:
            return
        self._busy = True
        try:
            for i, st in enumerate(self.s):
                if st.cr & 1 and (st.cr >> 6) & 3 != 2 and self.request_name(i) == req:
                    src = self.sources.get(req)
                    guard = 0
                    while st.cr & 1 and src is not None and src() and guard < 100000:
                        self._transfer_one(i)
                        guard += 1
        finally:
            self._busy = False

    def _transfer_one(self, i):
        st = self.s[i]
        cr = st.cr
        psize = 1 << ((cr >> 11) & 3)
        msize = 1 << ((cr >> 13) & 3)
        direction = (cr >> 6) & 3              # 0 P->M, 1 M->P, 2 M->M
        pinc = cr >> 9 & 1
        minc = cr >> 10 & 1
        mbase = st.m1ar if (cr >> 18 & 1 and cr >> 19 & 1) else st.m0ar
        size = psize if direction != 2 else psize
        mstep = (msize if direction == 2 else psize) if minc else 0
        pstep = psize if pinc else 0
        paddr = st.par + st.pi * pstep
        maddr = mbase + st.pi * mstep
        m = self.m
        if direction == 0:
            m.bus_write(maddr, m.bus_read(paddr, psize), psize)
        elif direction == 1:
            m.bus_write(paddr, m.bus_read(maddr, psize), psize)
        else:
            m.bus_write(maddr, m.bus_read(paddr, size), size)
        st.pi += 1
        st.ndtr -= 1
        if not st.half_done and st.ndtr <= st.n0 // 2:
            st.half_done = True
            st.flags |= 0x10
        if st.ndtr <= 0:
            self._end_of_block(i)
        self._irq(i)

    def _enable(self, i):
        st = self.s[i]
        st.n0 = st.ndtr
        st.pi = 0
        st.half_done = False
        if st.ndtr == 0:
            st.cr &= ~1
            return
        direction = (st.cr >> 6) & 3
        if direction == 2:
            if self.unit == 1:
                st.flags |= 0x08                     # DMA1 can't do memory-to-memory
                st.cr &= ~1
                self._irq(i)
                return
            while st.cr & 1:
                self._transfer_one(i)
            return
        req = self.request_name(i)
        if req is None:
            self.m.warn(f"{self.name} stream {i} channel {(st.cr >> 25) & 7}: no modelled "
                        f"peripheral request on that channel")
            return
        if req not in self.sources:
            self.m.warn(f"{self.name} stream {i}: {req} DMA requests are not modelled")
            return
        self.kick(req)

    # ---------------------------------------------------------------- registers
    def read(self, off):
        if off in (0x00, 0x04):
            v = 0
            for k in range(4):
                v |= self.s[k + (4 if off else 0)].flags << FLAG_SHIFT[k]
            return v
        if off in (0x08, 0x0C):
            return 0
        i, r = divmod(off - 0x10, 0x18)
        if not 0 <= i < 8:
            return 0
        st = self.s[i]
        if r == 0x04 and st.burst:
            t0, item, n = st.burst
            done = 1 + int((self.m.now() - t0) // item) if self.m.now() >= t0 else 0
            return max(n - done, 0)
        return {0x00: st.cr, 0x04: st.ndtr & 0xFFFF, 0x08: st.par, 0x0C: st.m0ar,
                0x10: st.m1ar, 0x14: st.fcr | (0x20 if not st.fcr & 4 else 0)}.get(r, 0)

    def write(self, off, value, mask):
        v = value & mask
        if off in (0x08, 0x0C):
            for k in range(4):
                i = k + (4 if off == 0x0C else 0)
                self.s[i].flags &= ~((v >> FLAG_SHIFT[k]) & 0x3D)
                self._irq(i)
            return
        if off < 0x10:
            return
        i, r = divmod(off - 0x10, 0x18)
        if not 0 <= i < 8:
            return
        st = self.s[i]
        if r == 0x00:
            new = (st.cr & ~mask) | v
            if st.cr & 1:
                st.cr = (st.cr & ~0x1F) | (new & 0x1F)    # only EN and the IE bits change
                self._irq(i)
                return
            st.cr = new
            self._irq(i)
            if new & 1:
                self._enable(i)
            return
        if st.cr & 1 and r in (0x04, 0x08):
            return                                         # locked while enabled
        if r == 0x04:
            st.ndtr = v & 0xFFFF
        elif r == 0x08:
            st.par = v
        elif r == 0x0C:
            st.m0ar = v
        elif r == 0x10:
            st.m1ar = v
        elif r == 0x14:
            st.fcr = (st.fcr & ~mask | v) & 0x87
            self._irq(i)
