"""core.py - the emulated STM32F407: CPU, memory map, time, interrupts.

The CPU is a real Cortex-M4 instruction-set emulator (Unicorn, built from
QEMU). Unicorn knows the instructions but not the chip around them, so this
file adds what a Cortex-M4 does outside the instruction set:

* the memory map (flash at 0x08000000 and aliased at 0, SRAM, CCM RAM,
  system memory with the unique ID, all peripheral registers);
* exception entry and return, exactly as the hardware stacks them (basic and
  FPU frames, stack realignment, MSP/PSP, nesting, priorities, PRIMASK and
  BASEPRI);
* virtual time. One instruction = one HCLK cycle (the F407 runs most code
  close to 1 instruction per clock with the ART accelerator), so firmware
  delays, timeouts and loop rates come out in real seconds at whatever clock
  the firmware configured. Time is an integer number of picoseconds.

Peripherals live in periph/*.py and register themselves on the bus here.
"""
from __future__ import annotations

import ctypes
import heapq
import struct
from pathlib import Path

import unicorn.unicorn_py3.unicorn as _uc_mod
_uclib = _uc_mod.uclib
from unicorn import (UC_ARCH_ARM, UC_HOOK_CODE, UC_HOOK_INTR, UC_MODE_MCLASS, UC_MODE_THUMB,
                     Uc, UcError)
from unicorn.arm_const import (UC_ARM_REG_BASEPRI, UC_ARM_REG_CONTROL, UC_ARM_REG_LR,
                               UC_ARM_REG_MSP, UC_ARM_REG_PC, UC_ARM_REG_PRIMASK,
                               UC_ARM_REG_R0, UC_ARM_REG_R1,
                               UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_R12,
                               UC_ARM_REG_S0, UC_ARM_REG_FPSCR, UC_ARM_REG_SP,
                               UC_ARM_REG_XPSR, UC_CPU_ARM_CORTEX_M4)

PS_PER_S = 1_000_000_000_000

FLASH_BASE, FLASH_SIZE = 0x08000000, 512 * 1024
SRAM_BASE, SRAM_SIZE = 0x20000000, 128 * 1024
CCM_BASE, CCM_SIZE = 0x10000000, 64 * 1024
SYSMEM_BASE, SYSMEM_SIZE = 0x1FFF0000, 0x8000      # bootloader, OTP, unique ID, flash size
OPTBYTES_BASE = 0x1FFFC000
NOTIFY_BASE = 0x1FFFF400                             # emulator-only: exception-return notify
TRAMP_BASE = 0x1FFFF800                              # emulator-only: exception-return code
TRAMP_LR = 0x1FFFFF00                                # LR in handlers = TRAMP_LR | EXC_RETURN low byte

# Peripheral address windows mapped as MMIO (start, end).
MMIO_WINDOWS = [
    (0x40000000, 0x40008000),   # APB1
    (0x40010000, 0x40016C00),   # APB2
    (0x40020000, 0x40080000),   # AHB1
    (0x50000000, 0x50061000),   # AHB2
    (0xA0000000, 0xA0001000),   # FSMC control
    (0xE0000000, 0xE0100000),   # Cortex-M4 private peripheral bus
]

# Exception numbers
RESET, NMI, HARDFAULT, MEMMANAGE, BUSFAULT, USAGEFAULT = 1, 2, 3, 4, 5, 6
SVCALL, DEBUGMON, PENDSV, SYSTICK = 11, 12, 14, 15
IRQ0 = 16

_FRAME_REGS = (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3,
               UC_ARM_REG_R12, UC_ARM_REG_LR)
_FP_REGS = tuple(UC_ARM_REG_S0 + i for i in range(16))
_PROBE_REGS = tuple(UC_ARM_REG_R0 + i for i in range(13)) + (UC_ARM_REG_SP, UC_ARM_REG_LR,
                                                               UC_ARM_REG_XPSR)


class EmulatorError(RuntimeError):
    pass


# Unicorn counts the instructions of a run (that is how emu_start's `count`
# works) but has no call to read that counter, so we find where it lives in
# Unicorn's state once, with a small probe, and read it directly. Writing the
# limit next to it moves the end of a running slice, which lets an event that
# appears in the middle of a slice end it exactly on time without stopping
# and restarting the CPU.
_COUNTER_OFFSET = None


def _find_counter_offset():
    """Returns (counter offset, limit offset) inside Unicorn's uc_struct."""
    global _COUNTER_OFFSET
    if _COUNTER_OFFSET is not None:
        return _COUNTER_OFFSET
    unsupported = EmulatorError("this Unicorn version is not supported by the simulator "
                                "(install the version in sim/requirements.txt)")
    # adds r0,#1 x3; ldr r2,[r1]; b back to the start
    code = bytes.fromhex("0130" "0130" "0130" "0a68" "fae7")

    def make_probe(read_cb):
        probe = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        probe.mem_map(0x08000000, 0x1000)
        probe.mem_write(0x08000000, code)
        probe.mmio_map(0x40000000, 0x400, read_cb, None, lambda *a: None, None)
        probe.reg_write(UC_ARM_REG_R1, 0x40000000)
        return probe, ctypes.cast(probe._uch, ctypes.c_void_p).value

    probe, base = make_probe(lambda *a: 0)
    n_words = 2048
    ctr_c = lim_c = None
    for n in (1234, 4321):
        probe.emu_start(0x08000001, 0xFFFFFFFF, 0, n)
        words = (ctypes.c_uint64 * n_words).from_address(base)
        c = {i for i in range(n_words) if words[i] == n + 1}
        lm = {i for i in range(n_words) if words[i] == n}
        ctr_c = c if ctr_c is None else ctr_c & c
        lim_c = lm if lim_c is None else lim_c & lm
    # check each candidate pair: it must count mid-run, and moving the limit
    # must move the end of the run
    for ci in sorted(ctr_c or ()):
        for li in sorted(lim_c or (), key=lambda x: abs(x - ci)):
            marks = []
            box = {}

            def rd(uc, o, size, ud):
                ctr, lim = box["ctr"], box["lim"]
                marks.append(ctr.value)
                if ctr.value == 9:
                    lim.value = 30
                return 0
            p2, b2 = make_probe(rd)
            box["ctr"] = ctypes.c_uint64.from_address(b2 + 8 * ci)
            box["lim"] = ctypes.c_uint64.from_address(b2 + 8 * li)
            try:
                p2.emu_start(0x08000001, 0xFFFFFFFF, 0, 20)
            except UcError:
                continue
            if marks[:3] == [4, 9, 14] and box["ctr"].value == 31:
                _COUNTER_OFFSET = (8 * ci, 8 * li)
                return _COUNTER_OFFSET
    raise unsupported


class Event:
    __slots__ = ("t", "seq", "fn", "cancelled", "world")

    def __init__(self, t, seq, fn):
        self.t, self.seq, self.fn, self.cancelled = t, seq, fn, False

    def __lt__(self, other):
        return (self.t, self.seq) < (other.t, other.seq)

    def cancel(self):
        self.cancelled = True


class _Probe:
    __slots__ = ("hook", "last", "stack", "hits", "busy", "useful", "drop", "idle_slices")

    def __init__(self):
        self.hook = None
        self.last = self.stack = None
        self.hits = self.busy = self.useful = self.idle_slices = 0
        self.drop = False


class Peripheral:
    """Base class: a block of 32-bit registers at `base`.

    Subclasses implement read(off) -> int and write(off, value, mask).
    `mask` has ones on the bits the CPU actually wrote (byte and half-word
    stores write only some lanes)."""

    size = 0x400
    name = "periph"
    gate = None         # RCC enable-bit name, e.g. "GPIOA"; None = always clocked

    def __init__(self, machine, name, base, size=None):
        self.m = machine
        self.name = name
        self.base = base
        if size is not None:
            self.size = size
        self.r = {}
        self.reset()

    def reset(self):
        self.r = {}

    def read(self, off):
        return self.r.get(off, 0)

    def write(self, off, value, mask):
        self.r[off] = (self.r.get(off, 0) & ~mask) | (value & mask)



class Unmodeled(Peripheral):
    """Plain register storage for peripherals the simulator doesn't model.
    The first access prints a warning so you know the result isn't real."""

    def __init__(self, machine, name, base, size=0x400):
        self.warned = False
        super().__init__(machine, name, base, size)

    def _warn(self, off):
        if not self.warned:
            self.warned = True
            self.m.warn(f"{self.name} (0x{self.base + off:08X}) is not modelled; "
                        f"it behaves like plain memory")

    def read(self, off):
        self._warn(off)
        return super().read(off)

    def write(self, off, value, mask):
        self._warn(off)
        super().write(off, value, mask)


class Machine:
    """The microcontroller. Create it, add peripherals, load firmware, run."""

    def __init__(self, log=print):
        self.log = log
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
        off_ctr, off_lim = _find_counter_offset()
        base = ctypes.cast(self.uc._uch, ctypes.c_void_p).value
        self._ctr = ctypes.c_uint64.from_address(base + off_ctr)   # instructions started
        self._lim = ctypes.c_uint64.from_address(base + off_lim)   # slice ends after this many
        self._uch = self.uc._uch
        self._regbuf = ctypes.c_uint64(0)
        self._regbuf_p = ctypes.byref(self._regbuf)
        self._batch = {}
        self._membuf = ctypes.c_uint64(0)
        self._membuf_p = ctypes.cast(ctypes.pointer(self._membuf), ctypes.c_char_p)
        self._warned = set()

        self.flash = bytearray(b"\xff" * FLASH_SIZE)
        self._map_memory()

        # time
        self.t = 0                  # time at the start of the running slice, ps
        self._cur = 0               # ps inside the slice on top of the instructions
                                    # (bus waits, stacking, skipped polling)
        self._in_slice = False
        self._events = []
        self._ev_seq = 0
        self.hclk = 16_000_000      # updated by RCC
        self._ps_per_cycle = PS_PER_S / self.hclk
        self.clock_listeners = []   # fn(old_hz, new_hz) when HCLK changes
        self.periph_clock_listeners = []   # fn() when any bus clock or enable changes
        self.dma = {}               # unit -> DMA controller
        self.dma_sources = {}       # unit -> {request name: fn() -> bool}
        self.exti = None
        self.syscfg = None
        self._counts = {}
        self.instructions = 0

        # bus
        self._pages = {}            # (addr >> 10) -> Peripheral
        self.rcc = None
        # idle-loop detection: the firmware keeps doing the same register
        # accesses with the same values -> nothing can change until the next event
        self._last_rd = {}
        self._last_wr = {}
        self._idle_n = 0
        self._stop_requested = False
        self._inline = False
        self._mmio_n = 0            # register accesses so far (to spot loops that only touch RAM)
        self._probes = {}           # pc -> _Probe, see _probe_loop
        self._no_probe = {}         # pc -> slices to wait before probing it again
        self.poll_skip = True
        self.poll_threshold = 4     # same register value this many times in a row = polling
        self._slice_end = 0
        self._run_end = 0
        self._max_slice = 0

        # exceptions
        self.nvic = None            # set by SCS peripheral
        self.active = []            # stack of active exception numbers
        self.fpu_enabled = False    # set by CPACR writes
        self.halted = None          # reason string if the CPU stopped for good
        self.resets = 0
        self.reset_hooks = []       # called on every system reset
        self.symbols = {}           # address -> name, from an ELF if loaded

        self.uc.hook_add(UC_HOOK_INTR, self._on_intr)

    # ------------------------------------------------------------------ memory
    def _mmio_map(self, lo, size, rd, wr, base):
        """uc.mmio_map without the binding's per-access wrappers (they cost
        more than the peripheral models themselves). rd(addr, size) and
        wr(addr, size, value) get the full address."""
        uc = self.uc

        def r(h, off, sz, ud):
            try:
                return rd(base + off, sz)
            except BaseException as e:          # pass the error out of emu_start
                if self._cb_error is None:
                    self._cb_error = e
                uc.emu_stop()
                return 0

        def w(h, off, sz, val, ud):
            try:
                wr(base + off, sz, val)
            except BaseException as e:
                if self._cb_error is None:
                    self._cb_error = e
                uc.emu_stop()

        cr, cw = _uc_mod.MMIO_READ_CFUNC(r), _uc_mod.MMIO_WRITE_CFUNC(w)
        self._cfuncs.append((cr, cw))
        status = _uclib.uc_mmio_map(uc._uch, lo, size, cr, None, cw, None)
        if status != 0:
            raise UcError(status)

    def _map_memory(self):
        uc = self.uc
        self._cfuncs = []
        self._cb_error = None
        uc.mem_map(FLASH_BASE, FLASH_SIZE)
        uc.mem_map(0x00000000, FLASH_SIZE)          # boot alias (BOOT0 = 0)
        uc.mem_map(SRAM_BASE, SRAM_SIZE)
        uc.mem_map(CCM_BASE, CCM_SIZE)
        uc.mem_map(SYSMEM_BASE, SYSMEM_SIZE)
        uc.mem_map(OPTBYTES_BASE, 0x400)
        uc.mem_map(0x40024000, 0x1000)              # backup SRAM (4 KB), plain RAM here
        self._mmio_map(NOTIFY_BASE, 0x400, self._notify_read, self._notify_write, 0)
        uc.mem_map(TRAMP_BASE, 0x800)
        tramp = (Path(__file__).parent / "asm" / "exc_return.bin").read_bytes()
        uc.mem_write(TRAMP_BASE, tramp)
        for lo, hi in MMIO_WINDOWS:
            if lo == 0x40020000:
                # leave a hole for backup SRAM at 0x40024000
                self._mmio_map(lo, 0x40024000 - lo, self._read_io, self._write_io, lo)
                self._mmio_map(0x40025000, hi - 0x40025000, self._read_io, self._write_io,
                               0x40025000)
            else:
                self._mmio_map(lo, hi - lo, self._read_io, self._write_io, lo)
        # bit-band aliases: one word per bit of SRAM and of the peripherals
        self._mmio_map(0x22000000, 0x02000000, self._bb_read, self._bb_write, 0x22000000)
        self._mmio_map(0x42000000, 0x02000000, self._bb_read, self._bb_write, 0x42000000)
        # system memory contents the HAL may read
        uid = struct.pack("<III", 0x00470031, 0x3233510A, 0x31383930)
        uc.mem_write(0x1FFF7A10, uid)                       # 96-bit unique ID
        uc.mem_write(0x1FFF7A22, struct.pack("<H", 512))    # flash size in KB
        uc.mem_write(0x1FFF7A2A, struct.pack("<H", 1500))   # VREFINT_CAL (approx 1.21 V)
        uc.mem_write(0x1FFF7A2C, struct.pack("<H", 950))    # TS_CAL1 (30 C)
        uc.mem_write(0x1FFF7A2E, struct.pack("<H", 1200))   # TS_CAL2 (110 C)
        uc.mem_write(OPTBYTES_BASE, struct.pack("<II", 0x0FFFAAED, 0x0FFF0000))

    def load_image(self, data: bytes, addr=FLASH_BASE):
        if addr >= FLASH_BASE and addr + len(data) <= FLASH_BASE + FLASH_SIZE:
            off = addr - FLASH_BASE
            self.flash[off:off + len(data)] = data
            self.uc.mem_write(addr, data)
            self.uc.mem_write(off, data)             # the alias at 0
        else:
            self.uc.mem_write(addr, data)

    def update_gates(self):
        """Refresh each peripheral's clock-enable state after an RCC change."""
        for p in set(self._pages.values()):
            p.clk_on = (not p.gate) or self.rcc is None or self.rcc.is_enabled(p.gate)

    def add(self, p: Peripheral):
        p.clk_on = (not p.gate) or self.rcc is None or self.rcc.is_enabled(p.gate)
        for page in range(p.base >> 10, (p.base + p.size + 0x3FF) >> 10):
            self._pages[page] = p
        return p

    def peripheral_at(self, addr):
        return self._pages.get(addr >> 10)

    def warn_count(self, what, n):
        self.log(f"[sim] t={self.now_s():.3f} s: {what} x{n}")

    def warn(self, msg):
        if msg not in self._warned:
            self._warned.add(msg)
            self.log(f"[sim] warning: {msg}")

    # ------------------------------------------------------------------ bus
    # Stopping the CPU from inside a register access is done by lowering the
    # slice's instruction limit (see _request_stop), never with emu_stop:
    # Unicorn would run the interrupted instruction a second time on resume.
    def _bb_read(self, alias, size):
        base = alias & 0xF0000000            # 0x22000000 -> SRAM, 0x42000000 -> peripherals
        offset = alias - (base | 0x02000000)
        addr = base + ((offset >> 5) & ~3)
        bit = (offset >> 2) & 31
        if base == 0x40000000:
            return (self._read_io(addr, 4) >> bit) & 1
        return (self.bus_read(addr) >> bit) & 1

    def _bb_write(self, alias, size, value):
        self._mmio_n += 1
        base = alias & 0xF0000000
        offset = alias - (base | 0x02000000)
        addr = base + ((offset >> 5) & ~3)
        bit = (offset >> 2) & 31
        if base == 0x40000000:
            p = self._pages.get(addr >> 10)
            self._cur += self._ps_per_cycle * 4
            self._idle_n = 0
            if p is None:
                self._unmapped(addr, True)
            elif not p.clk_on:
                self.warn(f"firmware wrote {p.name} while its clock is off; the write is ignored")
            else:
                # a bit-band write is a read-modify-write of the whole register
                old = p.read((addr - p.base) & ~3)
                p.write((addr - p.base) & ~3, (old & ~(1 << bit)) | ((value & 1) << bit), 1 << bit)
        else:
            old = self.bus_read(addr)
            self.bus_write(addr, (old & ~(1 << bit)) | ((value & 1) << bit))

    def _read_io(self, addr, size):
        self._mmio_n += 1
        p = self._pages.get(addr >> 10)
        self._cur += self._ps_per_cycle * 4             # APB access costs a few cycles
        if p is None:
            self._unmapped(addr, False)
            return 0
        if not p.clk_on:
            self.warn(f"firmware read {p.name} while its clock is off (enable it in RCC first); "
                      f"the real chip returns 0")
            return 0
        v = p.read((addr - p.base) & ~3)
        if size < 4:
            v = (v >> ((addr & 3) * 8)) & ((1 << (size * 8)) - 1)
        if self.poll_skip:
            if self._last_rd.get(addr) == v:
                self._idle_n += 1
                if self._idle_n >= self.poll_threshold:
                    self._idle_n = 0
                    self._skip_to_next_event()
            else:
                self._last_rd[addr] = v
                self._idle_n = 0
        return v

    def _write_io(self, addr, size, value):
        self._mmio_n += 1
        p = self._pages.get(addr >> 10)
        self._cur += self._ps_per_cycle * 4
        if self._last_wr.get(addr) == value:
            self._idle_n += 1                     # same write as last time: no progress
        else:
            self._last_wr[addr] = value
            self._idle_n = 0
        if p is None:
            self._unmapped(addr, True)
            return
        if not p.clk_on:
            self.warn(f"firmware wrote {p.name} while its clock is off (enable it in RCC first); "
                      f"the real chip ignores the write")
            return
        shift = (addr & 3) * 8
        mask = ((1 << (size * 8)) - 1) << shift
        p.write((addr - p.base) & ~3, (value << shift) & 0xFFFFFFFF, mask)

    def bus_read(self, addr, size=4):
        """Read as a bus master other than the CPU (DMA)."""
        p = self._pages.get(addr >> 10)
        if p is not None:
            v = p.read((addr - p.base) & ~3)
            shift = (addr & 3) * 8
            return (v >> shift) & ((1 << (size * 8)) - 1)
        buf = self._membuf
        buf.value = 0
        if _uclib.uc_mem_read(self._uch, addr, self._membuf_p, size) != 0:
            self.warn(f"DMA read from unmapped address 0x{addr:08X}")
            return 0
        return buf.value & ((1 << (size * 8)) - 1)

    def bus_write(self, addr, value, size=4):
        p = self._pages.get(addr >> 10)
        if p is not None:
            shift = (addr & 3) * 8
            mask = ((1 << (size * 8)) - 1) << shift
            p.write((addr - p.base) & ~3, (value << shift) & 0xFFFFFFFF, mask)
            return
        try:
            self.uc.mem_write(addr, (value & ((1 << (size * 8)) - 1)).to_bytes(size, "little"))
        except UcError:
            self.warn(f"DMA write to unmapped address 0x{addr:08X}")

    def _unmapped(self, addr, write):
        self.warn(f"firmware {'wrote' if write else 'read'} reserved address 0x{addr:08X} "
                  f"(no peripheral there)")

    # ------------------------------------------------------------------ time
    def now(self):
        if self._in_slice:
            # instructions finished before the one running now, plus extras
            n = self._ctr.value - 1
            return self.t + int((n if n > 0 else 0) * self._ps_per_cycle + self._cur)
        return self.t

    def now_s(self):
        return self.now() / PS_PER_S

    def cycles_to_ps(self, n):
        return n * self._ps_per_cycle

    def set_hclk(self, hz):
        if hz != self.hclk:
            old = self.hclk
            self.hclk = hz
            self._ps_per_cycle = PS_PER_S / hz
            for fn in self.clock_listeners:
                fn(old, hz)

    def at(self, t_ps, fn, world=False):
        """Run fn() at absolute time t_ps. Returns the Event (call .cancel()).

        world=True marks events of things outside the MCU (physics, the IMU,
        the PC); they keep running when the chip resets."""
        self._ev_seq += 1
        ev = Event(int(max(t_ps, self.now())), self._ev_seq, fn)
        ev.world = world
        heapq.heappush(self._events, ev)
        if self._in_slice and ev.t < self._slice_end:
            self._set_slice_end(ev.t)       # end the slice exactly when it is due
        return ev

    def after(self, dt_ps, fn, world=False):
        return self.at(self.now() + dt_ps, fn, world)

    def _next_event_t(self):
        ev = self._events
        while ev and ev[0].cancelled:
            heapq.heappop(ev)
        return ev[0].t if ev else None

    def _run_due_events(self):
        now = self.now()
        ev = self._events
        k = 0
        while ev and (ev[0].cancelled or ev[0].t <= now):
            e = heapq.heappop(ev)
            if not e.cancelled:
                e.fn()
                k += 1
                if k > 100000:
                    raise EmulatorError(f"simulator bug: event {getattr(e.fn, '__qualname__', e.fn)} "
                                        f"keeps rescheduling itself at t={now} ps")

    def _set_slice_end(self, t_ps):
        """Move the end of the running slice (it is counted in instructions)."""
        self._slice_end = t_ps
        if self._stop_requested:
            return
        left = (t_ps - self.now()) / self._ps_per_cycle
        self._lim.value = self._ctr.value + (int(left) if left >= 1 else 0)

    def _skip_to_next_event(self):
        """The firmware is spinning on a register waiting for something to
        happen: jump to the next event, run it here, and keep the CPU going."""
        if self._inline or self._stop_requested:
            return
        nt = self._next_event_t()
        if nt is None or nt > self._run_end:
            self._cur += max(0, self._run_end - self.now())
            self._request_stop()
            return
        now = self.now()
        while now < nt:                     # (float rounding: land exactly on nt or after)
            self._cur += nt - now
            now = self.now()
        self._inline = True
        ev = self._events
        k = 0
        while ev and (ev[0].cancelled or ev[0].t <= now):
            e = heapq.heappop(ev)
            if not e.cancelled:
                e.fn()
                k += 1
                if k > 100000:
                    self._inline = False
                    raise EmulatorError(f"simulator bug: event {getattr(e.fn, '__qualname__', e.fn)} "
                                        f"keeps rescheduling itself at t={nt} ps")
        self._inline = False
        if not self._stop_requested:
            self._set_slice_end(self._plan_end())

    def _plan_end(self):
        """Where a slice starting now should end: the next event, the end of
        the run, or the longest slice, whichever comes first."""
        now = self.now()
        end = min(self._run_end, now + self._max_slice)
        nt = self._next_event_t()
        if nt is not None and nt < end:
            end = nt
        return end

    def _irq_check(self):
        """An interrupt may have become deliverable: stop the slice to take it."""
        if self._in_slice and self.nvic is not None and self.nvic.deliverable() is not None:
            self._request_stop()

    def _request_stop(self, hard=False):
        """End the running slice after the current instruction."""
        if self._in_slice and not self._stop_requested:
            self._stop_requested = True
            self._lim.value = self._ctr.value
            if hard:                            # (from a CPU-exception hook)
                self.uc.emu_stop()

    # ------------------------------------------------------------------ CPU state
    def reg(self, r):
        """Read a CPU register (faster than uc.reg_read, which is generic)."""
        _uclib.uc_reg_read(self._uch, r, self._regbuf_p)
        return self._regbuf.value & 0xFFFFFFFF

    def regs(self, ids):
        """Read several CPU registers in one call."""
        c = self._batch.get(ids)
        if c is None:
            n = len(ids)
            vals = (ctypes.c_uint64 * n)()
            ptrs = (ctypes.c_void_p * n)(*[ctypes.addressof(vals) + 8 * i for i in range(n)])
            c = self._batch[ids] = ((ctypes.c_int * n)(*ids), vals, ptrs, n)
        _uclib.uc_reg_read_batch(self._uch, c[0], c[2], c[3])
        return [v & 0xFFFFFFFF for v in c[1]]

    def _read_word(self, addr):
        return struct.unpack("<I", self.uc.mem_read(addr, 4))[0]

    def cpu_reset(self):
        """Core reset: SP and PC from the vector table at address 0."""
        uc = self.uc
        self.active = []
        self.fpu_enabled = False
        uc.reg_write(UC_ARM_REG_CONTROL, 0)
        uc.reg_write(UC_ARM_REG_PRIMASK, 0)
        uc.reg_write(UC_ARM_REG_BASEPRI, 0)
        sp = self._read_word(0)
        pc = self._read_word(4)
        uc.reg_write(UC_ARM_REG_MSP, sp)
        uc.reg_write(UC_ARM_REG_SP, sp)
        uc.reg_write(UC_ARM_REG_LR, 0xFFFFFFFF)
        uc.reg_write(UC_ARM_REG_XPSR, 0x01000000)
        self._pc = pc | 1
        self.halted = None

    def system_reset(self, reason="software"):
        """What NRST, SYSRESETREQ or the watchdog do: reset the chip, not the world."""
        self.resets += 1
        self.log(f"[sim] t={self.now_s():.4f} s: system reset ({reason})")
        self._events = [e for e in self._events if e.world and not e.cancelled]
        heapq.heapify(self._events)
        for p in set(self._pages.values()):
            p.reset()
        for h in self.reset_hooks:
            h(reason)
        self.set_hclk(16_000_000)
        self.cpu_reset()
        self._request_stop()

    # ------------------------------------------------------------------ exceptions
    def _vtor(self):
        return self.nvic.vtor if self.nvic else 0

    def _take_exception(self, exc):
        uc = self.uc
        handler_mode = bool(self.active)
        control, sp, xpsr = self.regs((UC_ARM_REG_CONTROL, UC_ARM_REG_SP, UC_ARM_REG_XPSR))
        on_psp = (not handler_mode) and (control & 2)
        fp = self.fpu_enabled
        frame_size = 0x68 if fp else 0x20
        ipsr = self.active[-1] if self.active else 0
        xpsr = (xpsr & ~0x1FF) | ipsr | 0x01000000
        pad = (sp - frame_size) & 4
        frame = (sp - frame_size) & ~7
        if pad:
            xpsr |= 0x200
        pc = self._pc if self._pc is not None else uc.reg_read(UC_ARM_REG_PC) | 1
        if fp:
            vals = self.regs(_FRAME_REGS + _FP_REGS + (UC_ARM_REG_FPSCR,))
            data = struct.pack("<6I2I16I2I", *vals[:6], pc & ~1, xpsr, *vals[6:22], vals[22], 0)
        else:
            vals = self.regs(_FRAME_REGS)
            data = struct.pack("<6I2I", *vals, pc & ~1, xpsr)
        try:
            uc.mem_write(frame, data)
        except UcError:
            self._lockup(f"stack overflow: exception frame at 0x{frame:08X} is outside RAM")
            return
        uc.reg_write(UC_ARM_REG_SP, frame)
        if on_psp:
            uc.reg_write(UC_ARM_REG_CONTROL, control & ~2)   # handlers run on MSP
        exc_ret = 0xF1 if handler_mode else (0xFD if on_psp else 0xF9)
        if fp:
            exc_ret &= ~0x10
        uc.reg_write(UC_ARM_REG_LR, TRAMP_LR | exc_ret)
        # Unicorn must keep believing it runs in thread mode (IPSR = 0); the
        # simulator tracks the active exception itself in self.active.
        uc.reg_write(UC_ARM_REG_XPSR, (xpsr & 0xF80F0000) | 0x01000000)
        self.active.append(exc)
        vec = self._read_word(self._vtor() + 4 * exc)
        if vec == 0 or vec == 0xFFFFFFFF:
            self._lockup(f"no handler for exception {exc} in the vector table")
            return
        self._pc = vec | 1
        self.t += int(self._ps_per_cycle * 12)               # stacking takes 12 cycles

    def _notify_read(self, addr, size):
        return 0

    def _notify_write(self, addr, size, value):
        """The trampoline reports an exception return (value = EXC_RETURN low byte)."""
        self._exc_return()

    def _exc_return(self):
        if not self.active:
            self._lockup("exception return while not in a handler")
            return
        exc = self.active.pop()
        if self.nvic:
            self.nvic.on_return(exc)
        self._cur += self._ps_per_cycle * 12
        self._irq_check()                       # tail-chain into a pending interrupt

    def _on_intr(self, uc, intno, ud):
        if intno == 2:                          # SVC
            self.nvic.set_pending(SVCALL)
            self._request_stop(hard=True)
        elif intno == 7:                        # BKPT
            self._fault_pending = f"BKPT instruction at 0x{uc.reg_read(UC_ARM_REG_PC):08X}"
            self._request_stop(hard=True)
        else:
            self._fault_pending = f"CPU exception {intno} at 0x{uc.reg_read(UC_ARM_REG_PC):08X}"
            self._request_stop(hard=True)

    def _lockup(self, why):
        if self.halted is None:
            self.halted = why
            self.log(f"[sim] t={self.now_s():.4f} s: CPU stopped: {why}")

    def symbol(self, addr):
        if not self.symbols:
            return ""
        best = None
        for a, n in self.symbols.items():
            if a <= addr and (best is None or a > best[0]):
                best = (a, n)
        return f" in {best[1]}()" if best else ""

    def _hard_fault(self, why):
        pc = self._pc or self.uc.reg_read(UC_ARM_REG_PC)
        self.log(f"[sim] t={self.now_s():.4f} s: HardFault: {why}, PC=0x{pc & ~1:08X}"
                 f"{self.symbol(pc & ~1)}")
        if self.nvic:
            self.nvic.cfsr_note(why)
        if HARDFAULT in self.active:
            self._lockup("fault inside the HardFault handler (lockup)")
            return
        self._take_exception(HARDFAULT)

    # ------------------------------------------------------------------ run loop
    def run(self, seconds=None, until=None, max_slice_us=1000):
        """Run for `seconds` of simulated time (or until time `until` in ps)."""
        t_end = until if until is not None else self.t + int(seconds * PS_PER_S)
        uc = self.uc
        max_slice = int(max_slice_us * 1e6)
        self._run_end = t_end
        self._fault_pending = None
        while self.t < t_end:
            self._run_due_events()
            if self.halted:
                nt = self._next_event_t()
                self.t = t_end if nt is None else min(max(nt, self.t), t_end)
                continue
            # interrupts
            exc = self.nvic.deliverable() if self.nvic else None
            if exc is not None and not self._in_it_block():
                self.nvic.acknowledge(exc)
                self._take_exception(exc)
                if self.halted:
                    continue
            # A pending interrupt that is masked may be let in by an instruction
            # (CPSIE, MSR BASEPRI) that the simulator doesn't see: check often.
            masked_pending = self.nvic is not None and self.nvic.any_pending_enabled()
            self._max_slice = max_slice // 10 if masked_pending else max_slice
            end = self._plan_end()
            n = max(1, int((end - self.t) / self._ps_per_cycle))
            self._slice_end = end
            self._cur = 0
            self._stop_requested = False
            self._in_slice = True
            mmio0 = self._mmio_n
            pc = self._pc
            self._pc = None
            err = None
            try:
                uc.emu_start(pc, 0xFFFFFFFF, 0, n)
            except UcError as e:
                err = e
            if self._cb_error is not None:
                self._in_slice = False
                e, self._cb_error = self._cb_error, None
                raise e
            ran = max(0, self._ctr.value - 1)
            done = ran * self._ps_per_cycle + self._cur
            self._in_slice = False
            if err is None and not self._stop_requested and ran < self._lim.value \
                    and self._wfi_stopped():
                # WFI/WFE: sleep until the next event
                nt = self._next_event_t()
                done = max(done, min(t_end if nt is None else nt, t_end) - self.t)
            self.instructions += ran
            self.t += max(1, int(done))
            self._cur = 0
            if self._pc is None:                 # (a reset during the slice sets it)
                self._pc = self.reg(UC_ARM_REG_PC) | 1
            if self.poll_skip and ran > 2000:
                self._probe_housekeeping(err is None and not self._stop_requested
                                         and self._mmio_n == mmio0)
            if err is not None:
                self._handle_uc_error(err)
            if self._fault_pending:
                why, self._fault_pending = self._fault_pending, None
                self._hard_fault(why)

    # ---------------------------------------------------------------- RAM loops
    # A loop that waits for an interrupt to change a variable in RAM (HAL_Delay
    # waiting for uwTick, `while (!flag) {}`) touches no register, so the
    # check in _read_io can't see it. When a long slice made no register
    # access, a probe is put on the instruction where it stopped. If the CPU
    # comes back to that instruction with every register the same, the loop
    # is making no progress and time jumps to the next event, as for polling.
    def _probe_housekeeping(self, quiet_slice):
        for pc in list(self._no_probe):
            self._no_probe[pc] -= 1
            if self._no_probe[pc] <= 0:
                del self._no_probe[pc]
        for pc, p in list(self._probes.items()):
            if p.drop or p.hits == 0:
                p.idle_slices += 1
            if p.drop or p.idle_slices > 3:
                self.uc.hook_del(p.hook)
                self.uc.ctl_remove_cache(pc, pc + 4)
                del self._probes[pc]
                if p.drop or not p.useful:
                    self._no_probe[pc] = 200
            else:
                if p.hits:
                    p.idle_slices = 0
                p.hits = 0
        if quiet_slice and len(self._probes) < 4:
            # Probe where the loop returns to (LR) if the CPU is in a function
            # it calls, like HAL_GetTick() from HAL_Delay(); else where it is.
            lr = self.reg(UC_ARM_REG_LR)
            cands = [lr & ~1] if FLASH_BASE <= lr < FLASH_BASE + FLASH_SIZE else []
            cands.append(self._pc & ~1)
            for pc in cands:
                if pc not in self._probes and pc not in self._no_probe:
                    break
            else:
                return
            p = _Probe()
            p.hook = self.uc.hook_add(UC_HOOK_CODE, self._probe_hit, p, begin=pc, end=pc)
            self.uc.ctl_remove_cache(pc, pc + 4)
            self._probes[pc] = p

    def _probe_hit(self, uc, addr, size, p):
        if p.drop or self._stop_requested:
            return
        p.hits += 1
        regs = self.regs(_PROBE_REGS)
        if regs != p.last:
            p.last = regs
            p.stack = None
            self._probe_busy(p)
            return
        # unoptimised (-O0) code keeps its variables on the stack, so the top
        # of the stack is part of the state that must not change
        sp = regs[13]
        end = SRAM_BASE + SRAM_SIZE if sp >= SRAM_BASE else CCM_BASE + CCM_SIZE
        try:
            stack = bytes(uc.mem_read(sp, max(0, min(128, end - sp))))
        except UcError:
            stack = None
        if stack != p.stack:
            p.stack = stack
            self._probe_busy(p)
            return
        p.useful += 1
        self._skip_to_next_event()

    def _probe_busy(self, p):
        p.busy += 1
        if p.busy > 64 and p.busy > 20 * p.useful:
            p.drop = True                   # mostly real work passes here: stop probing it

    def _in_it_block(self):
        x = self.reg(UC_ARM_REG_XPSR)
        return bool(x & 0x0600FC00)

    def _wfi_stopped(self):
        pc = self.uc.reg_read(UC_ARM_REG_PC) & ~1
        try:
            prev = struct.unpack("<H", self.uc.mem_read(pc - 2, 2))[0]
        except UcError:
            return False
        return prev in (0xBF30, 0xBF20)          # WFI, WFE

    def _handle_uc_error(self, err):
        pc = self.uc.reg_read(UC_ARM_REG_PC)
        msg = str(err)
        if pc & 0xFFFFFF00 == 0xFFFFFF00:
            self._lockup("a handler returned with a raw EXC_RETURN value the simulator does "
                         "not support (0x%08X)" % pc)
            return
        self._pc = pc | 1
        plain = {
            "UC_ERR_INSN_INVALID": "undefined instruction (UsageFault UNDEFINSTR)",
            "UC_ERR_READ_UNMAPPED": "read from an address where nothing is mapped (BusFault)",
            "UC_ERR_WRITE_UNMAPPED": "write to an address where nothing is mapped (BusFault)",
            "UC_ERR_FETCH_UNMAPPED": "jumped to an address with no code (BusFault IBUSERR); "
                                     "often a bad function pointer or a corrupted stack",
            "UC_ERR_READ_UNALIGNED": "unaligned read (UsageFault UNALIGNED)",
            "UC_ERR_WRITE_UNALIGNED": "unaligned write (UsageFault UNALIGNED)",
            "UC_ERR_EXCEPTION": "CPU exception (often a division by zero with DIV_0_TRP set, "
                                "or an instruction the M4 does not have)",
        }
        for code, text in plain.items():
            if code in msg:
                msg = f"{text} [{code}]"
                break
        self._hard_fault(msg)
