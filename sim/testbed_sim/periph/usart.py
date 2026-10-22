"""USART1-3, UART4-5, USART6 (asynchronous mode).

Bytes take real time on the wire at the baud rate the firmware set (BRR and
the APB clock), with TXE/TC/RXNE/IDLE/ORE flags, interrupts and DMA requests
as on the chip. A byte arriving while RXNE is still set is lost and sets ORE,
like on the real board. If `host_baud` is set and differs from the firmware's
baud by more than 3 %, received and sent bytes are garbled (framing errors),
which is what a terminal at the wrong speed shows.
"""
from __future__ import annotations

import collections

from ..core import PS_PER_S, Peripheral

UARTS = {
    "USART1": (0x40011000, 37), "USART2": (0x40004400, 38), "USART3": (0x40004800, 39),
    "UART4": (0x40004C00, 52), "UART5": (0x40005000, 53), "USART6": (0x40011400, 71),
}
SR, DR, BRR, CR1, CR2, CR3 = 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14
PE, FE, NE, ORE, IDLE, RXNE, TC, TXE = 1, 2, 4, 8, 16, 32, 64, 128


class USART(Peripheral):
    def __init__(self, machine, name):
        base, self.irq = UARTS[name]
        self.gate = name
        self.tx_sink = None          # fn(byte) - set by the board (USB-serial chip)
        self.tx_connected = lambda: True
        self.rx_connected = lambda: True
        self.host_baud = None
        self.burst_ok = True             # allow the DMA fast paths
        self.rx_queue = collections.deque()
        self._rx_ev = None
        self._tx_ev = None
        self._idle_ev = None
        self.reported_baud = None
        self.bytes_tx = 0
        self.bytes_rx = 0
        self.overruns = 0
        super().__init__(machine, name, base)
        dma_unit = 2 if name in ("USART1", "USART6") else 1
        self._dma_unit = dma_unit
        self.m.dma_sources.setdefault(dma_unit, {})[name + "_RX"] = self._rx_req
        self.m.dma_sources[dma_unit][name + "_TX"] = self._tx_req

    def reset(self):
        for e in (getattr(self, "_rx_ev", None), getattr(self, "_tx_ev", None),
                  getattr(self, "_idle_ev", None)):
            if e:
                e.cancel()
        self._rx_ev = self._tx_ev = self._idle_ev = None
        self.rx_queue.clear()            # bytes sent while the chip resets are lost
        self.sr = TXE | TC
        self.rdr = 0
        self.tdr = None
        self.shifting = False
        self.r = {}
        self._sr_read = False
        self._line_free_at = 0
        if self.m.nvic:
            self._update_irq()

    # ---------------------------------------------------------------- timing
    def baud(self):
        brr = self.r.get(BRR, 0)
        if brr == 0:
            return 0
        pclk = self.m.rcc.pclk(self.name)
        over8 = self.r.get(CR1, 0) >> 15 & 1
        if over8:
            div = (brr >> 4) + (brr & 7) / 8
            return pclk / (8 * div) if div else 0
        div = (brr >> 4) + (brr & 15) / 16
        return pclk / (16 * div) if div else 0

    def frame_ps(self):
        b = self.baud()
        if b <= 0:
            return PS_PER_S
        cr1 = self.r.get(CR1, 0)
        bits = 1 + (9 if cr1 >> 12 & 1 else 8)
        stop = {0: 1, 1: 0.5, 2: 2, 3: 1.5}[(self.r.get(CR2, 0) >> 12) & 3]
        return int((bits + stop) / b * PS_PER_S)

    def _baud_ok(self):
        b = self.baud()
        if self.reported_baud != round(b):
            self.reported_baud = round(b)
            self.m.log(f"[sim] {self.name} running at {b:.0f} baud")
        if self.host_baud and b and abs(b - self.host_baud) / self.host_baud > 0.03:
            self.m.warn(f"{self.name} is at {b:.0f} baud but the PC side is set to "
                        f"{self.host_baud}: bytes will be garbled")
            return False
        return True

    def enabled_tx(self):
        cr1 = self.r.get(CR1, 0)
        return cr1 >> 13 & 1 and cr1 >> 3 & 1

    def enabled_rx(self):
        cr1 = self.r.get(CR1, 0)
        return cr1 >> 13 & 1 and cr1 >> 2 & 1

    # ---------------------------------------------------------------- TX
    def _start_shift(self, byte):
        self.shifting = True
        self.sr &= ~TC
        self._tx_ev = self.m.after(self.frame_ps(), lambda b=byte: self._shift_done(b))

    def _shift_done(self, byte):
        self._tx_ev = None
        self.shifting = False
        if self.tx_connected():
            if not self._baud_ok():
                byte = (byte * 37 + 11) & 0xFF            # what a mismatched terminal sees
            self.bytes_tx += 1
            if self.tx_sink:
                self.tx_sink(byte)
        if self.tdr is not None:
            b, self.tdr = self.tdr, None
            self.sr |= TXE
            self._start_shift(b)
            self._tx_dma()
        else:
            self.sr |= TC
        self._update_irq()

    def _write_dr(self, value):
        if not self.enabled_tx():
            return
        if self._sr_read:
            self.sr &= ~TC                              # SR read then DR write clears TC
        byte = value & 0x1FF if self.r.get(CR1, 0) >> 12 & 1 else value & 0xFF
        if not self.shifting:
            self.sr |= TXE
            self._start_shift(byte)
        else:
            self.tdr = byte
            self.sr &= ~TXE
        self._update_irq()

    def _tx_req(self):
        return bool(self.r.get(CR3, 0) & 0x80 and self.sr & TXE and self.enabled_tx())

    def _tx_dma(self):
        if self._tx_req():
            dma = self.m.dma.get(self._dma_unit)
            if not dma:
                return
            i = dma.active_stream(self.name + "_TX", 1)
            if i is not None and not self.shifting and self.tdr is None and self.burst_ok:
                self._tx_burst(dma.take_burst(i, self.frame_ps()))
            else:
                dma.kick(self.name + "_TX")

    def _tx_burst(self, data):
        """Send a whole DMA buffer with one event (same timing on the wire)."""
        if not data:
            return
        self.shifting = True
        self.sr &= ~TC
        frame = self.frame_ps()
        t0 = self.m.now()
        self._update_irq()

        def done():
            self._tx_ev = None
            self.shifting = False
            ok = self.tx_connected()
            good = self._baud_ok()
            for b in data:
                if ok:
                    self.bytes_tx += 1
                    if self.tx_sink:
                        self.tx_sink(b if good else (b * 37 + 11) & 0xFF)
            self.sr |= TC | TXE
            self._update_irq()
        self._tx_ev = self.m.at(t0 + len(data) * frame, done)

    # ---------------------------------------------------------------- RX
    def host_send(self, data: bytes):
        """Bytes typed/sent by the PC towards this UART's RX pin."""
        self.rx_queue.extend(data)
        if self._rx_ev is None:
            self._schedule_rx(max(self.m.now(), self._line_free_at))

    def _rx_dma_stream(self):
        if not (self.r.get(CR3, 0) & 0x40 and self.burst_ok):
            return None
        dma = self.m.dma.get(self._dma_unit)
        if dma is None:
            return None
        i = dma.active_stream(self.name + "_RX", 0)
        return None if i is None else (dma, i)

    def _schedule_rx(self, start):
        frame = self.frame_ps()
        if self._rx_dma_stream() and len(self.rx_queue) > 1 and not self.sr & RXNE:
            n = len(self.rx_queue)
            self._rx_ev = self.m.at(start + n * frame, lambda n=n: self._rx_burst(n))
        else:
            self._rx_ev = self.m.at(start + frame, self._rx_arrive)

    def _rx_burst(self, n):
        """Deliver n bytes that arrived back to back, straight into the RX DMA buffer."""
        self._rx_ev = None
        ds = self._rx_dma_stream()
        if ds is None or self.sr & RXNE:
            self._rx_arrive()
            return
        dma, i = ds
        n = min(n, len(self.rx_queue))
        data = [self.rx_queue.popleft() for _ in range(n)]
        if self.enabled_rx() and self.rx_connected():
            if not self._baud_ok():
                self.sr |= FE
                data = [(b * 53 + 7) & 0xFF for b in data]
            took = dma.put_burst(i, data)
            self.bytes_rx += took
            if took:
                self.rdr = data[took - 1]
            if took < n:                                # stream stopped: the rest goes the slow way
                self.rx_queue.extendleft(reversed(data[took:]))
        self._line_free_at = self.m.now()
        self.sr &= ~IDLE
        if self._idle_ev:
            self._idle_ev.cancel()
        self._idle_ev = self.m.after(self.frame_ps(), self._idle)
        self._update_irq()
        if self.rx_queue:
            self._schedule_rx(self.m.now())

    def _rx_arrive(self):
        self._rx_ev = None
        if not self.rx_queue:
            return
        byte = self.rx_queue.popleft()
        self._line_free_at = self.m.now()
        if self.enabled_rx() and self.rx_connected():
            if not self._baud_ok():
                self.sr |= FE
                byte = (byte * 53 + 7) & 0xFF
            if self.sr & RXNE:
                self.sr |= ORE                          # old byte not read yet: new one is lost
                self.overruns += 1
                if self.overruns in (1, 10, 100, 1000):
                    self.m.warn_count(f"{self.name} RX overrun", self.overruns)
            else:
                self.rdr = byte
                self.sr |= RXNE
                self.bytes_rx += 1
            self.sr &= ~IDLE
            if self._idle_ev:
                self._idle_ev.cancel()
            self._idle_ev = self.m.after(self.frame_ps(), self._idle)
            self._update_irq()
            if self.sr & RXNE and self.r.get(CR3, 0) & 0x40:
                dma = self.m.dma.get(self._dma_unit)
                if dma:
                    dma.kick(self.name + "_RX")
        if self.rx_queue:
            self._schedule_rx(self.m.now())

    def _idle(self):
        self._idle_ev = None
        if self.enabled_rx():
            self.sr |= IDLE
            self._update_irq()

    def _rx_req(self):
        return bool(self.r.get(CR3, 0) & 0x40 and self.sr & RXNE)

    # ---------------------------------------------------------------- IRQ
    def _update_irq(self):
        if self.m.nvic is None:
            return
        cr1 = self.r.get(CR1, 0)
        cr3 = self.r.get(CR3, 0)
        sr = self.sr
        lvl = ((cr1 & 0x80 and sr & TXE) or (cr1 & 0x40 and sr & TC)
               or (cr1 & 0x20 and sr & (RXNE | ORE)) or (cr1 & 0x10 and sr & IDLE)
               or (cr1 & 0x100 and sr & PE) or (cr3 & 1 and sr & (FE | NE | ORE)))
        self.m.nvic.set_irq(self.irq, self.name, bool(lvl))

    # ---------------------------------------------------------------- registers
    def read(self, off):
        if off == SR:
            self._sr_read = True
            return self.sr
        if off == DR:
            v = self.rdr
            self.sr &= ~RXNE
            if self._sr_read:
                self.sr &= ~(PE | FE | NE | ORE | IDLE)
            self._sr_read = False
            self._update_irq()
            return v
        return self.r.get(off, 0)

    def write(self, off, value, mask):
        if off == SR:
            v = value | ~mask
            self.sr &= (v | ~(RXNE | TC)) & 0x3FF       # only RXNE, TC can be cleared by 0
            self._update_irq()
            return
        if off == DR:
            self._write_dr(value & mask)
            self._sr_read = False
            return
        old = self.r.get(off, 0)
        new = (old & ~mask) | (value & mask)
        self.r[off] = new
        if off == CR1 and not new >> 13 & 1:
            self.sr = TXE | TC                           # UE off resets the state machine
        if off == CR3 and new & 0x80 and not old & 0x80:
            self._tx_dma()
        if off == CR3 and new & 0x40 and not old & 0x40 and self.sr & RXNE:
            dma = self.m.dma.get(self._dma_unit)
            if dma:
                dma.kick(self.name + "_RX")
        self._update_irq()
