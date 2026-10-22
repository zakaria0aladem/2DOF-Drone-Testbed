"""Cortex-M4 system control space: NVIC, SCB, SysTick, FPU, plus DWT, ITM, DBGMCU."""
from __future__ import annotations

from ..core import (BUSFAULT, HARDFAULT, IRQ0, MEMMANAGE, NMI, PENDSV,
                    SYSTICK, USAGEFAULT, Peripheral)
from unicorn.arm_const import UC_ARM_REG_BASEPRI, UC_ARM_REG_PRIMASK

NUM_IRQ = 82            # STM32F407: IRQ 0..81


class SCS(Peripheral):
    """0xE000E000-0xE000EFFF."""

    size = 0x1000

    def __init__(self, machine):
        self.irq_level = {}         # irq -> set of asserting sources
        super().__init__(machine, "SCS", 0xE000E000, 0x1000)
        machine.nvic = self
        machine.clock_listeners.append(self._hclk_changed)

    def reset(self):
        self.enabled = 0
        self.pending = 0            # bit n = exception n (not IRQ number)
        self.prio = bytearray(16 + NUM_IRQ)
        self.prigroup = 0
        self.vtor = 0
        self.ccr = 0x200
        self.scr = 0
        self.shcsr = 0
        self.cfsr = 0
        self.hfsr = 0
        self.cpacr = 0
        self.fpccr = 0xC0000000
        self.other = {}
        self.demcr = 0
        self.m.fpu_enabled = False
        # SysTick
        self.st_ctrl = 0
        self.st_load = 0
        self.st_t0 = 0              # time the counter last held st_load
        self.st_countflag = 0
        self.st_ev = None
        self.st_cvr_frozen = 0
        for irq, srcs in self.irq_level.items():
            if srcs:
                self.pending |= 1 << (IRQ0 + irq)

    # ------------------------------------------------------------------ IRQ lines
    def set_irq(self, irq, source, level):
        """Peripherals call this with their interrupt output (level-sensitive)."""
        srcs = self.irq_level.setdefault(irq, set())
        if level:
            srcs.add(source)
            bit = 1 << (IRQ0 + irq)
            if not self.pending & bit:
                self.pending |= bit
                if self.enabled >> irq & 1:
                    self.m._irq_check()
        else:
            srcs.discard(source)

    def set_pending(self, exc):
        self.pending |= 1 << exc
        self.m._irq_check()

    def acknowledge(self, exc):
        self.pending &= ~(1 << exc)

    def on_return(self, exc):
        if exc >= IRQ0 and self.irq_level.get(exc - IRQ0):
            self.pending |= 1 << exc

    def _exc_prio(self, exc):
        if exc == NMI:
            return -2
        if exc == HARDFAULT:
            return -1
        return self.prio[exc] if exc < len(self.prio) else 0

    def _group(self, p):
        if p < 0:
            return p
        mask = (~((2 << self.prigroup) - 1)) & 0xFF
        return p & mask

    def _enabled(self, exc):
        if exc >= IRQ0:
            return bool(self.enabled >> (exc - IRQ0) & 1)
        if exc in (MEMMANAGE, BUSFAULT, USAGEFAULT):
            return bool(self.shcsr >> {MEMMANAGE: 16, BUSFAULT: 17, USAGEFAULT: 18}[exc] & 1)
        return True

    def exec_priority(self):
        p = 256
        for a in self.m.active:
            p = min(p, self._group(self._exc_prio(a)))
        if self.m.reg(UC_ARM_REG_PRIMASK) & 1:
            p = min(p, 0)
        bp = self.m.reg(UC_ARM_REG_BASEPRI) & 0xF0
        if bp:
            p = min(p, self._group(bp))
        return p

    def _best_pending(self):
        best = None
        bits = self.pending
        exc = 0
        while bits:
            if bits & 1 and self._enabled(exc):
                pr = self._exc_prio(exc)
                if best is None or pr < best[0]:
                    best = (pr, exc)
            bits >>= 1
            exc += 1
        return best

    def deliverable(self):
        """Exception number that should be taken now, or None."""
        if not self.pending:
            return None
        best = self._best_pending()
        if best is None:
            return None
        if self._group(best[0]) < self.exec_priority():
            return best[1]
        return None

    def any_pending_enabled(self):
        return self.pending != 0 and self._best_pending() is not None

    def cfsr_note(self, why):
        self.hfsr |= 1 << 30            # FORCED
        if "FETCH_UNMAPPED" in why:
            self.cfsr |= 1 << 8         # IBUSERR
        elif "UNMAPPED" in why:
            self.cfsr |= 1 << 10        # IMPRECISERR (data bus error)
        elif "INSN" in why:
            self.cfsr |= 1 << 16        # UNDEFINSTR
        elif "UNALIGNED" in why:
            self.cfsr |= 1 << 24        # UNALIGNED

    # ------------------------------------------------------------------ SysTick
    def _st_clk_ps(self):
        hclk = self.m.hclk
        f = hclk if self.st_ctrl & 4 else hclk / 8
        return 1e12 / f

    def _st_schedule(self):
        if self.st_ev:
            self.st_ev.cancel()
            self.st_ev = None
        if not (self.st_ctrl & 1) or self.st_load == 0:
            return
        clk = self._st_clk_ps()
        period = (self.st_load + 1) * clk
        first_zero = self.st_t0 + self.st_load * clk     # counter holds st_load at st_t0
        now = self.m.now()
        if first_zero > now:
            t = first_zero
        else:
            k = int((now - first_zero) // period) + 1
            t = first_zero + k * period
        while int(t) <= now:          # event times are whole ps: never the same instant again
            t += period
        self.st_ev = self.m.at(t, self._st_wrap)

    def _st_wrap(self):
        self.st_countflag = 1
        if self.st_ctrl & 2:
            self.set_pending(SYSTICK)
        self.st_ev = None
        self._st_schedule()

    def _st_cvr(self):
        if not (self.st_ctrl & 1):
            return self.st_cvr_frozen
        if self.st_load == 0:
            return 0
        now = self.m.now()
        if now < self.st_t0:
            return 0
        ticks = int((now - self.st_t0) / self._st_clk_ps())
        return self.st_load - (ticks % (self.st_load + 1))

    def _st_start(self, value):
        """Counter (re)starts from `value` now; 0 means it reloads on the next clock."""
        clk = self._st_clk_ps()
        now = self.m.now()
        self.st_t0 = now - (self.st_load - value) * clk if value else now + clk

    def _hclk_changed(self, old, new):
        if self.st_ctrl & 1:
            # keep the current count, continue at the new rate
            m = self.m
            m.hclk = old
            cvr = self._st_cvr()
            m.hclk = new
            self._st_start(cvr)
            self._st_schedule()

    # ------------------------------------------------------------------ registers
    def read(self, off):
        if off == 0x004:
            return 2                                   # ICTR: up to 96 IRQs
        if off == 0x010:
            v = self.st_ctrl | (self.st_countflag << 16)
            self.st_countflag = 0
            return v
        if off == 0x014:
            return self.st_load
        if off == 0x018:
            return self._st_cvr()
        if off == 0x01C:
            return 0x40000000 | 21000                  # CALIB: NOREF, 1 ms at 21 MHz
        if 0x100 <= off < 0x120:
            return (self.enabled >> ((off - 0x100) * 8)) & 0xFFFFFFFF
        if 0x180 <= off < 0x1A0:
            return (self.enabled >> ((off - 0x180) * 8)) & 0xFFFFFFFF
        if 0x200 <= off < 0x220 or 0x280 <= off < 0x2A0:
            base = 0x200 if off < 0x280 else 0x280
            return ((self.pending >> IRQ0) >> ((off - base) * 8)) & 0xFFFFFFFF
        if 0x300 <= off < 0x320:
            act = 0
            for a in self.m.active:
                if a >= IRQ0:
                    act |= 1 << (a - IRQ0)
            return (act >> ((off - 0x300) * 8)) & 0xFFFFFFFF
        if 0x400 <= off < 0x400 + NUM_IRQ + 2:
            i = off - 0x400
            v = 0
            for k in range(4):
                if i + k < NUM_IRQ:
                    v |= self.prio[IRQ0 + i + k] << (8 * k)
            return v
        if off == 0xD00:
            return 0x410FC241                          # CPUID: Cortex-M4 r0p1
        if off == 0xD04:
            return self._icsr()
        if off == 0xD08:
            return self.vtor
        if off == 0xD0C:
            return 0xFA050000 | (self.prigroup << 8)
        if off == 0xD10:
            return self.scr
        if off == 0xD14:
            return self.ccr
        if 0xD18 <= off < 0xD24:
            e = 4 + (off - 0xD18)
            return sum(self.prio[e + k] << (8 * k) for k in range(4))
        if off == 0xD24:
            return self.shcsr
        if off == 0xD28:
            return self.cfsr
        if off == 0xD2C:
            return self.hfsr
        if off == 0xD88:
            return self.cpacr
        if off == 0xD90:
            return 0x800                               # MPU TYPE: 8 regions
        if off == 0xDF0:
            return 0                                    # DHCSR: no debugger attached
        if off == 0xDFC:
            return self.demcr
        if off == 0xF34:
            return self.fpccr
        if 0xFD0 <= off:
            return {0xFE0: 0x0C, 0xFE4: 0xB0, 0xFE8: 0x0B, 0xFF0: 0x0D, 0xFF4: 0xE0,
                    0xFF8: 0x05, 0xFFC: 0xB1}.get(off, 0)
        return self.other.get(off, 0)

    def _icsr(self):
        v = 0
        if self.m.active:
            v |= self.m.active[-1] & 0x1FF
        if len(self.m.active) <= 1:
            v |= 1 << 11                                # RETTOBASE
        best = self._best_pending()
        if best:
            v |= (best[1] & 0x1FF) << 12
        if self.pending >> IRQ0:
            v |= 1 << 22
        if self.pending >> PENDSV & 1:
            v |= 1 << 28
        if self.pending >> SYSTICK & 1:
            v |= 1 << 26
        return v

    def write(self, off, value, mask):
        self._write(off, value, mask)
        self.m._irq_check()             # enables / priorities / pend bits may let one in

    def _write(self, off, value, mask):
        if off == 0x010:
            was = self.st_ctrl
            new = (self.st_ctrl & ~mask | value & mask) & 7
            if was & 1 and not (new & 1):
                self.st_cvr_frozen = self._st_cvr()
            self.st_ctrl = new
            if not (was & 1) and new & 1:
                self._st_start(self.st_cvr_frozen)
            self._st_schedule()
            return
        if off == 0x014:
            self.st_load = (self.st_load & ~mask | value & mask) & 0xFFFFFF
            self._st_schedule()
            return
        if off == 0x018:
            # any write clears the counter; it reloads on the next clock
            self.st_countflag = 0
            self.st_cvr_frozen = 0
            if self.st_ctrl & 1:
                self._st_start(0)
            self._st_schedule()
            return
        if 0x100 <= off < 0x120:
            self.enabled |= (value & mask) << ((off - 0x100) * 8)
            return
        if 0x180 <= off < 0x1A0:
            self.enabled &= ~((value & mask) << ((off - 0x180) * 8))
            return
        if 0x200 <= off < 0x220:
            self.pending |= ((value & mask) << ((off - 0x200) * 8)) << IRQ0
            return
        if 0x280 <= off < 0x2A0:
            self.pending &= ~(((value & mask) << ((off - 0x280) * 8)) << IRQ0)
            return
        if 0x400 <= off < 0x400 + NUM_IRQ + 2:
            i = off - 0x400
            for k in range(4):
                if mask >> (8 * k) & 0xFF and i + k < NUM_IRQ:
                    self.prio[IRQ0 + i + k] = (value >> (8 * k)) & 0xF0
            return
        if off == 0xD04:
            if value & (1 << 28):
                self.set_pending(PENDSV)
            if value & (1 << 27):
                self.acknowledge(PENDSV)
            if value & (1 << 26):
                self.set_pending(SYSTICK)
            if value & (1 << 25):
                self.acknowledge(SYSTICK)
            if value & (1 << 31):
                self.set_pending(NMI)
            return
        if off == 0xD08:
            self.vtor = value & 0xFFFFFE00
            return
        if off == 0xD0C:
            if (value >> 16) != 0x05FA:
                return                                  # wrong VECTKEY: ignored
            self.prigroup = (value >> 8) & 7
            if value & 4:
                self.m.system_reset("SYSRESETREQ (NVIC_SystemReset)")
            return
        if off == 0xD10:
            self.scr = value & 0x16
            return
        if off == 0xD14:
            self.ccr = (self.ccr & ~mask | value & mask) & 0x31B
            return
        if 0xD18 <= off < 0xD24:
            e = 4 + (off - 0xD18)
            for k in range(4):
                if mask >> (8 * k) & 0xFF:
                    self.prio[e + k] = (value >> (8 * k)) & 0xF0
            return
        if off == 0xD24:
            self.shcsr = (self.shcsr & ~mask | value & mask)
            return
        if off == 0xD28:
            self.cfsr &= ~(value & mask)                # write 1 to clear
            return
        if off == 0xD2C:
            self.hfsr &= ~(value & mask)
            return
        if off == 0xD88:
            self.cpacr = (self.cpacr & ~mask | value & mask)
            self.m.fpu_enabled = (self.cpacr >> 20) & 0xF == 0xF
            return
        if off == 0xDFC:
            self.demcr = (self.demcr & ~mask | value & mask)
            return
        if off == 0xF00:
            irq = value & 0x1FF
            self.pending |= 1 << (IRQ0 + irq)           # STIR
            return
        if off == 0xF34:
            self.fpccr = (self.fpccr & ~mask | value & mask)
            return
        self.other[off] = (self.other.get(off, 0) & ~mask) | (value & mask)


class DWT(Peripheral):
    """Only the cycle counter (CYCCNT) is modelled; firmware uses it for microsecond timing."""

    def __init__(self, machine):
        super().__init__(machine, "DWT", 0xE0001000, 0x1000)
        machine.clock_listeners.append(self._hclk_changed)

    def reset(self):
        self.ctrl = 0x40000000
        self.base_cnt = 0
        self.t0 = 0
        self.other = {}

    def _running(self):
        return self.ctrl & 1 and self.m.nvic.demcr >> 24 & 1

    def cyccnt(self):
        if not self._running():
            return self.base_cnt & 0xFFFFFFFF
        cyc = (self.m.now() - self.t0) / self.m._ps_per_cycle
        return (self.base_cnt + int(cyc)) & 0xFFFFFFFF

    def _rebase(self):
        self.base_cnt = self.cyccnt()
        self.t0 = self.m.now()

    def _hclk_changed(self, old, new):
        # cycles so far were at the old rate
        if self._running():
            cyc = (self.m.now() - self.t0) * old / 1e12
            self.base_cnt = (self.base_cnt + int(cyc)) & 0xFFFFFFFF
            self.t0 = self.m.now()

    def read(self, off):
        if off == 0:
            return self.ctrl
        if off == 4:
            return self.cyccnt()
        return self.other.get(off, 0)

    def write(self, off, value, mask):
        if off == 0:
            self._rebase()
            self.ctrl = (self.ctrl & ~mask | value & mask)
            self.t0 = self.m.now()
            return
        if off == 4:
            self.base_cnt = value & mask
            self.t0 = self.m.now()
            return
        self.other[off] = value & mask


class ITM(Peripheral):
    """ITM stimulus port 0: ITM_SendChar() text goes to the 'swo' console."""

    def __init__(self, machine, sink=None):
        self.sink = sink
        super().__init__(machine, "ITM", 0xE0000000, 0x1000)

    def reset(self):
        self.ter = 0
        self.tcr = 0
        self.other = {}

    def read(self, off):
        if off < 0x80:
            return 1                                     # port ready
        if off == 0xE00:
            return self.ter
        if off == 0xE80:
            return self.tcr
        return self.other.get(off, 0)

    def write(self, off, value, mask):
        if off < 0x80:
            if self.sink and self.tcr & 1:
                self.sink(off // 4, value & 0xFF)
            return
        if off == 0xE00:
            self.ter = value
        elif off == 0xE80:
            self.tcr = value
        else:
            self.other[off] = value


class DBGMCU(Peripheral):
    def __init__(self, machine):
        super().__init__(machine, "DBGMCU", 0xE0042000, 0x400)

    def read(self, off):
        if off == 0:
            return 0x10076413                            # STM32F40x/41x, rev 1
        return super().read(off)
