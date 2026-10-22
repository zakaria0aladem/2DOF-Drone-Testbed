"""I2C1-3 master mode (STM32F4 "v1" I2C), byte-accurate.

Follows the flag sequence the reference manual and the HAL rely on: START ->
SB, address -> ADDR (or AF on NACK), ADDR cleared by reading SR1 then SR2,
TXE/BTF while sending, RXNE/BTF while receiving with ACK/POS controlling which
byte gets NACKed, STOP. Each byte takes 9 SCL periods at the speed set in CCR.
Slave devices (the QMI8658) attach with bus.attach(device).
"""
from __future__ import annotations

from ..core import PS_PER_S, Peripheral

I2CS = {"I2C1": (0x40005400, 31, 32), "I2C2": (0x40005800, 33, 34), "I2C3": (0x40005C00, 72, 73)}
CR1, CR2, OAR1, OAR2, DR, SR1, SR2, CCR, TRISE = 0, 4, 8, 0xC, 0x10, 0x14, 0x18, 0x1C, 0x20
SB, ADDR, BTF, STOPF, RXNE, TXE, BERR, ARLO, AF, OVR = 1, 2, 4, 16, 64, 128, 256, 512, 1024, 2048
MSL, BUSY, TRA = 1, 2, 4


class I2CDevice:
    """Interface for an I2C slave. Return True from start/write for ACK."""
    address = 0x00

    def i2c_start(self, read: bool) -> bool:
        return True

    def i2c_write(self, byte: int) -> bool:
        return True

    def i2c_read(self) -> int:
        return 0xFF

    def i2c_stop(self):
        pass


class I2C(Peripheral):
    def __init__(self, machine, name):
        base, self.irq_ev, self.irq_er = I2CS[name]
        self.gate = name
        self.devices = {}
        self.connected = lambda: True        # set by the board: SCL/SDA pins in AF mode
        self._ev = None
        super().__init__(machine, name, base)
        self.m.dma_sources.setdefault(1, {})[name + "_RX"] = self._rx_req
        self.m.dma_sources[1][name + "_TX"] = self._tx_req

    def attach(self, dev):
        self.devices[dev.address] = dev

    def reset(self):
        if getattr(self, "_ev", None):
            self._ev.cancel()
        self._ev = None
        self.r = {}
        self.sr1 = 0
        self._last_sr1 = None
        self.sr2 = 0
        self.dev = None
        self.reading = False
        self.dr_full = False
        self.dr = 0
        self.shift_full = False
        self.shift = 0
        self.tx_busy = False
        self.rx_active = False
        self.rx_index = 0
        self.nacked = False
        self.stop_pending = False
        self.start_pending = False
        self._sr1_read = False
        self.addr_phase = False
        if self.m.nvic:
            self._update_irq()

    # ---------------------------------------------------------------- timing
    def scl_period_ps(self):
        ccr = self.r.get(CCR, 0)
        pclk = self.m.rcc.pclk(self.name)
        c = ccr & 0xFFF
        if c == 0 or pclk == 0:
            return int(PS_PER_S / 100e3)
        if ccr >> 15 & 1:                        # fast mode
            mult = 25 if ccr >> 14 & 1 else 3
        else:
            mult = 2
        return int(mult * c / pclk * PS_PER_S)

    def byte_ps(self):
        return 9 * self.scl_period_ps()

    def _check_freq(self):
        freq = self.r.get(CR2, 0) & 0x3F
        pclk_mhz = self.m.rcc.pclk(self.name) / 1e6
        if freq and abs(freq - pclk_mhz) > 1:
            self.m.warn(f"{self.name}: CR2.FREQ = {freq} MHz but APB1 is {pclk_mhz:.0f} MHz")

    # ---------------------------------------------------------------- bus events
    def _later(self, dt, fn):
        if self._ev:
            self._ev.cancel()
        self._ev = self.m.after(dt, fn)

    def _gen_start(self):
        self._ev = None
        self.start_pending = False
        self.r[CR1] = self.r.get(CR1, 0) & ~0x100
        if self.dev is not None and not self.addr_phase:
            pass                                   # repeated start: same device stays selected
        self.sr1 |= SB
        self.sr2 |= MSL | BUSY
        self.addr_phase = True
        self.rx_active = False
        self.tx_busy = False
        self.dr_full = self.shift_full = False
        self.sr1 &= ~(BTF | TXE | RXNE)
        self._update_irq()

    def _gen_stop(self):
        self._ev = None
        self.stop_pending = False
        self.r[CR1] = self.r.get(CR1, 0) & ~0x200
        if self.dev is not None:
            self.dev.i2c_stop()
        self.dev = None
        self.sr2 &= ~(MSL | BUSY | TRA)
        self.rx_active = False
        self.tx_busy = False
        self._update_irq()

    def _addr_done(self, byte):
        self._ev = None
        addr = byte >> 1
        read = bool(byte & 1)
        dev = self.devices.get(addr) if self.connected() else None
        ack = dev is not None and dev.i2c_start(read)
        if not ack:
            self.sr1 |= AF
            self.dev = None
        else:
            self.dev = dev
            self.reading = read
            self.sr1 |= ADDR
            if read:
                self.sr2 &= ~TRA
            else:
                self.sr2 |= TRA
        self._update_irq()

    def _clear_addr(self):
        self.sr1 &= ~ADDR
        self.addr_phase = False
        if self.dev is None:
            return
        if self.reading:
            self.rx_index = 0
            self.nacked = False
            self._start_rx_byte()
        else:
            self.sr1 |= TXE
            self._tx_dma()
        self._update_irq()

    # transmitter
    def _start_tx_byte(self, byte):
        self.tx_busy = True
        self.sr1 &= ~BTF
        self._later(self.byte_ps(), lambda b=byte: self._tx_done(b))

    def _tx_done(self, byte):
        self._ev = None
        self.tx_busy = False
        ack = self.dev.i2c_write(byte) if self.dev else False
        if not ack:
            self.sr1 |= AF
        if self.dr_full and ack:
            self.dr_full = False
            self.sr1 |= TXE
            self._start_tx_byte(self.dr)
            self._tx_dma()
        else:
            self.sr1 |= BTF if ack else 0
            if self.stop_pending:
                self._later(self.scl_period_ps(), self._gen_stop)
            elif self.start_pending:
                self._later(self.scl_period_ps(), self._gen_start)
        self._update_irq()

    # receiver
    def _start_rx_byte(self):
        if self.nacked or self.dev is None:
            return
        self.rx_active = True
        self._later(self.byte_ps(), self._rx_done)

    def _rx_done(self):
        self._ev = None
        self.rx_active = False
        byte = self.dev.i2c_read() if self.dev else 0xFF
        cr1 = self.r.get(CR1, 0)
        cr2 = self.r.get(CR2, 0)
        if cr1 >> 11 & 1:                           # POS: ACK bit applies to the next byte
            ack = self.rx_index == 0 or bool(cr1 >> 10 & 1)
        else:
            ack = bool(cr1 >> 10 & 1)
        if cr2 >> 12 & 1 and cr2 >> 11 & 1:         # DMA with LAST: NACK the final byte
            dma = self.m.dma.get(1)
            rem = dma.remaining(self.name + "_RX") if dma else None
            if rem is not None and rem <= 1 + (1 if self.dr_full else 0):
                ack = False
        if self.stop_pending:
            ack = False
        self.rx_index += 1
        if not ack:
            self.nacked = True
        if not self.dr_full:
            self.dr = byte
            self.dr_full = True
            self.sr1 |= RXNE
        else:
            self.shift = byte
            self.shift_full = True
            self.sr1 |= BTF
        if self.stop_pending:
            self._later(self.scl_period_ps(), self._gen_stop)
        elif self.start_pending:
            self._later(self.scl_period_ps(), self._gen_start)
        elif ack and not self.shift_full:
            self._start_rx_byte()
        self._update_irq()
        self._rx_dma()

    def _read_dr(self):
        v = self.dr
        if self.shift_full:
            self.dr = self.shift
            self.shift_full = False
            self.sr1 &= ~BTF
            self.sr1 |= RXNE
            self.dr_full = True
            if not self.nacked and not self.rx_active and not self.stop_pending \
                    and self.dev is not None and not self.addr_phase:
                self._start_rx_byte()
        else:
            self.dr_full = False
            self.sr1 &= ~RXNE
            if self.dev is not None and self.reading and not self.nacked and not self.rx_active \
                    and not self.stop_pending and not self.addr_phase and self.sr2 & BUSY:
                self._start_rx_byte()
        self._update_irq()
        return v

    # ---------------------------------------------------------------- DMA / IRQ
    def _rx_req(self):
        return bool(self.r.get(CR2, 0) >> 11 & 1 and self.sr1 & RXNE)

    def _tx_req(self):
        return bool(self.r.get(CR2, 0) >> 11 & 1 and self.sr1 & TXE and not self.addr_phase)

    def _rx_dma(self):
        if self._rx_req():
            self.m.dma[1].kick(self.name + "_RX")

    def _tx_dma(self):
        if self._tx_req() and 1 in self.m.dma:
            self.m.dma[1].kick(self.name + "_TX")

    def _update_irq(self):
        nvic = self.m.nvic
        if nvic is None:
            return
        cr2 = self.r.get(CR2, 0)
        ev = 0
        if cr2 >> 9 & 1:
            ev = self.sr1 & (SB | ADDR | BTF | STOPF)
            if cr2 >> 10 & 1:
                ev |= self.sr1 & (TXE | RXNE)
        er = (cr2 >> 8 & 1) and self.sr1 & (BERR | ARLO | AF | OVR)
        nvic.set_irq(self.irq_ev, self.name, bool(ev))
        nvic.set_irq(self.irq_er, self.name, bool(er))

    # ---------------------------------------------------------------- registers
    def read(self, off):
        if off == SR1:
            self._sr1_read = True
            v = self.sr1
            if v == self._last_sr1 and self._ev is not None and not self._ev.cancelled \
                    and self.m._in_slice:
                # read twice with no change while a byte is on the wire: the
                # firmware is waiting for it, so let time run to the next event
                self.m._skip_to_next_event()
                v = self.sr1
            self._last_sr1 = v
            return v
        if off == SR2:
            v = self.sr2
            if self.sr1 & ADDR and self._sr1_read:
                self._clear_addr()
            self._sr1_read = False
            return v
        if off == DR:
            self._sr1_read = False
            return self._read_dr()
        return self.r.get(off, 0)

    def write(self, off, value, mask):
        v = value & mask
        if off == SR1:
            self.sr1 &= (value | ~mask | ~0xDF00) & 0xFFFF   # error flags: write 0 to clear
            self._update_irq()
            return
        if off == DR:
            self._sr1_read = False
            if not self.r.get(CR1, 0) & 1:
                return
            byte = v & 0xFF
            if self.sr1 & SB:
                self.sr1 &= ~SB
                self._later(self.byte_ps(), lambda b=byte: self._addr_done(b))
                self._update_irq()
                return
            if self.dev is None or self.reading:
                return
            self.sr1 &= ~BTF
            if not self.tx_busy:
                self.sr1 |= TXE
                self._start_tx_byte(byte)
            else:
                self.dr = byte
                self.dr_full = True
                self.sr1 &= ~TXE
            self._update_irq()
            return
        old = self.r.get(off, 0)
        new = (old & ~mask) | v
        if off == CR1:
            if new >> 15 & 1:                            # SWRST
                self.reset()
                self.r[CR1] = new
                return
            self.r[CR1] = new
            if not new & 1:
                self.sr1 = 0
                self.sr2 = 0
                return
            if new >> 8 & 1 and not old >> 8 & 1:       # START
                busy_xfer = self.tx_busy or self.rx_active
                if busy_xfer:
                    self.start_pending = True
                else:
                    self._later(self.scl_period_ps() // 2, self._gen_start)
            if new >> 9 & 1 and not old >> 9 & 1:       # STOP
                if self.tx_busy or self.rx_active:
                    self.stop_pending = True
                else:
                    self._later(self.scl_period_ps() // 2, self._gen_stop)
            self._update_irq()
            return
        self.r[off] = new
        if off == CR2:
            self._check_freq()
            if new >> 11 & 1 and not old >> 11 & 1:
                self._tx_dma()
                self._rx_dma()
        self._update_irq()
