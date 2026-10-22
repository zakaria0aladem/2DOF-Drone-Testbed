"""RCC (clocks and peripheral enables), FLASH interface, PWR, SYSCFG, IWDG, CRC, RNG."""
from __future__ import annotations

import random

from ..core import PS_PER_S, Peripheral

HSI = 16_000_000

# enable-register offset, bit for each gated peripheral
GATES = {}
for i, n in enumerate("ABCDEFGHI"):
    GATES["GPIO" + n] = (0x30, i)
GATES.update({"CRC": (0x30, 12), "DMA1": (0x30, 21), "DMA2": (0x30, 22),
              "RNG": (0x34, 6),
              "TIM2": (0x40, 0), "TIM3": (0x40, 1), "TIM4": (0x40, 2), "TIM5": (0x40, 3),
              "TIM6": (0x40, 4), "TIM7": (0x40, 5), "TIM12": (0x40, 6), "TIM13": (0x40, 7),
              "TIM14": (0x40, 8), "WWDG": (0x40, 11), "SPI2": (0x40, 14), "SPI3": (0x40, 15),
              "USART2": (0x40, 17), "USART3": (0x40, 18), "UART4": (0x40, 19),
              "UART5": (0x40, 20), "I2C1": (0x40, 21), "I2C2": (0x40, 22), "I2C3": (0x40, 23),
              "CAN1": (0x40, 25), "CAN2": (0x40, 26), "PWR": (0x40, 28), "DAC": (0x40, 29),
              "TIM1": (0x44, 0), "TIM8": (0x44, 1), "USART1": (0x44, 4), "USART6": (0x44, 5),
              "ADC1": (0x44, 8), "ADC2": (0x44, 9), "ADC3": (0x44, 10), "SDIO": (0x44, 11),
              "SPI1": (0x44, 12), "SYSCFG": (0x44, 14), "TIM9": (0x44, 16),
              "TIM10": (0x44, 17), "TIM11": (0x44, 18)})
APB1 = {"TIM2", "TIM3", "TIM4", "TIM5", "TIM6", "TIM7", "TIM12", "TIM13", "TIM14", "WWDG",
        "SPI2", "SPI3", "USART2", "USART3", "UART4", "UART5", "I2C1", "I2C2", "I2C3", "CAN1",
        "CAN2", "PWR", "DAC"}
AHB_DIV = {8: 2, 9: 4, 10: 8, 11: 16, 12: 64, 13: 128, 14: 256, 15: 512}
APB_DIV = {4: 2, 5: 4, 6: 8, 7: 16}
# max HCLK for each flash wait-state setting at 2.7-3.6 V (RM0090 table 10)
FLASH_WS_MAX = [30e6, 60e6, 90e6, 120e6, 150e6, 168e6, 168e6, 168e6]


class RCC(Peripheral):
    def __init__(self, machine, hse_hz=16_000_000, lse=False):
        self.hse_hz = hse_hz            # 0 = no crystal fitted
        self.lse = lse
        self.periphs = {}               # name -> Peripheral, for RSTR resets
        self.reset_flags = 0x0E000000   # POR + PIN + BOR on first power-up
        super().__init__(machine, "RCC", 0x40023800)
        machine.rcc = self
        self._warned_clock = None

    def reset(self):
        self.r = {0x00: 0x83, 0x04: 0x24003010, 0x30: 0x00100000, 0x08: 0,
                  0x50: 0x7E6791FF, 0x54: 0xF1, 0x58: 0x1, 0x60: 0x36FEC9FF, 0x64: 0x75F33,
                  0x84: 0x20003000}
        self.r[0x74] = self.reset_flags
        self.reset_flags = 0
        self._ready_ev = []
        self.sysclk = HSI
        self.hclk = HSI
        self.pclk1 = HSI
        self.pclk2 = HSI
        self.m.set_hclk(HSI)
        if self.m.rcc is self:
            self.m.update_gates()

    def note_reset(self, flag_bit):
        """Set a reset-cause flag that survives the coming reset (e.g. IWDGRSTF)."""
        self.reset_flags |= 1 << flag_bit

    def is_enabled(self, name):
        off, bit = GATES[name]
        return bool(self.r.get(off, 0) >> bit & 1)

    # ---------------------------------------------------------------- clocks
    def _pll_out(self, which="P"):
        cfg = self.r[0x04]
        m = cfg & 0x3F
        n = (cfg >> 6) & 0x1FF
        p = ((cfg >> 16) & 3) * 2 + 2
        q = (cfg >> 24) & 0xF
        src = self.hse_hz if cfg >> 22 & 1 else HSI
        if m < 2 or n < 50 or src == 0:
            return 0
        vco_in = src / m
        vco = vco_in * n
        return vco / (p if which == "P" else max(q, 2))

    def _check_pll(self):
        cfg = self.r[0x04]
        m = cfg & 0x3F
        n = (cfg >> 6) & 0x1FF
        src = self.hse_hz if cfg >> 22 & 1 else HSI
        if m < 2 or src == 0:
            self.m.warn("PLL configured with PLLM < 2 or a missing source")
            return
        vin = src / m
        if not 0.95e6 <= vin <= 2.1e6:
            self.m.warn(f"PLL input {vin/1e6:.2f} MHz is outside 1-2 MHz (check PLLM)")
        vco = vin * n
        if not 100e6 <= vco <= 432e6:
            self.m.warn(f"PLL VCO {vco/1e6:.1f} MHz is outside 100-432 MHz (check PLLN)")

    def update_clocks(self):
        sws = (self.r[0x08] >> 2) & 3
        if sws == 0:
            sysclk = HSI
        elif sws == 1:
            sysclk = self.hse_hz
        else:
            sysclk = self._pll_out()
        cfg = self.r[0x08]
        hclk = sysclk / AHB_DIV.get((cfg >> 4) & 0xF, 1)
        self.sysclk = sysclk
        self.hclk = hclk
        self.pclk1 = hclk / APB_DIV.get((cfg >> 10) & 7, 1)
        self.pclk2 = hclk / APB_DIV.get((cfg >> 13) & 7, 1)
        if hclk > 168.5e6:
            self.m.warn(f"HCLK is {hclk/1e6:.1f} MHz, above the F407's 168 MHz limit")
        if self.pclk1 > 42.5e6:
            self.m.warn(f"APB1 is {self.pclk1/1e6:.1f} MHz, above its 42 MHz limit (PPRE1)")
        if self.pclk2 > 84.5e6:
            self.m.warn(f"APB2 is {self.pclk2/1e6:.1f} MHz, above its 84 MHz limit (PPRE2)")
        flash = getattr(self.m, "flash_if", None)
        if flash is not None:
            ws = flash.latency()
            if hclk > FLASH_WS_MAX[ws] + 1e3:
                self.m.warn(f"flash latency {ws} WS is too low for HCLK {hclk/1e6:.0f} MHz; "
                            f"a real F407 would read wrong instructions and crash")
        if hclk > 0:
            self.m.set_hclk(hclk)
        for fn in getattr(self.m, "periph_clock_listeners", []):
            fn()

    def timer_clock(self, name):
        cfg = self.r[0x08]
        if name in APB1:
            div = APB_DIV.get((cfg >> 10) & 7, 1)
            return self.pclk1 * (1 if div == 1 else 2)
        div = APB_DIV.get((cfg >> 13) & 7, 1)
        return self.pclk2 * (1 if div == 1 else 2)

    def pclk(self, name):
        return self.pclk1 if name in APB1 else self.pclk2

    # ---------------------------------------------------------------- registers
    def _later(self, us, fn):
        self.m.after(int(us * 1e6), fn)

    def _set_bits(self, off, bits):
        self.r[off] = self.r.get(off, 0) | bits

    def write(self, off, value, mask):
        old = self.r.get(off, 0)
        new = (old & ~mask) | (value & mask)
        if off == 0x00:                                  # CR
            ro = 1 << 1 | 1 << 17 | 1 << 25 | 1 << 27
            new = (new & ~ro) | (old & ro)
            self.r[0] = new
            if new & 1 and not old & 1:
                self._later(2, lambda: self._set_bits(0, 1 << 1))
            if not new & 1:
                self.r[0] &= ~(1 << 1)
            if new >> 16 & 1 and not old >> 16 & 1:
                if self.hse_hz:
                    self._later(1000, lambda: self._set_bits(0, 1 << 17))   # crystal start-up
                else:
                    self.m.warn("HSE turned on but this board has no HSE crystal")
            if not new >> 16 & 1:
                self.r[0] &= ~(1 << 17)
            if new >> 24 & 1 and not old >> 24 & 1:
                self._check_pll()

                def lock():
                    if self.r[0] >> 24 & 1 and self._pll_out() > 0:
                        src_hse = self.r[0x04] >> 22 & 1
                        if (src_hse and self.r[0] >> 17 & 1) or (not src_hse and self.r[0] >> 1 & 1):
                            self._set_bits(0, 1 << 25)
                self._later(100, lock)
            if not new >> 24 & 1:
                self.r[0] &= ~(1 << 25)
            if new >> 26 & 1:
                self._later(100, lambda: self._set_bits(0, 1 << 27))
            else:
                self.r[0] &= ~(1 << 27)
            return
        if off == 0x04 and self.r[0] >> 24 & 1:
            return                                       # PLLCFGR is locked while PLL runs
        if off == 0x08:                                  # CFGR
            new = (new & ~0xC) | (old & 0xC)
            self.r[8] = new
            sw = new & 3
            ok = {0: self.r[0] >> 1 & 1, 1: self.r[0] >> 17 & 1, 2: self.r[0] >> 25 & 1}.get(sw, 0)
            if ok:
                self.r[8] = (self.r[8] & ~0xC) | (sw << 2)
            self.update_clocks()
            return
        if off in (0x10, 0x14, 0x18, 0x20, 0x24):        # peripheral reset registers
            en_off = off + 0x20
            for name, (o, bit) in GATES.items():
                if o == en_off and (new & ~old) >> bit & 1:
                    p = self.periphs.get(name)
                    if p is not None:
                        p.reset()
            self.r[off] = new
            return
        if off == 0x70:                                  # BDCR
            self.r[off] = new & ~2
            if new & 1 and self.lse:
                self._later(2000, lambda: self._set_bits(0x70, 2))
            elif new & 1 and not self.lse:
                self.m.warn("LSE turned on but no 32.768 kHz crystal is modelled on this board")
            return
        if off == 0x74:                                  # CSR
            if new & (1 << 24):                          # RMVF clears reset flags
                new &= 0x00FFFFFF
            new = (new & ~2) | (old & 2)
            self.r[off] = new & ~(1 << 24)
            if new & 1 and not old & 1:
                self._later(40, lambda: self._set_bits(0x74, 2))
            if not new & 1:
                self.r[off] &= ~2
            return
        self.r[off] = new
        if off in (0x30, 0x34, 0x40, 0x44):
            self.m.update_gates()
            for fn in getattr(self.m, "periph_clock_listeners", []):
                fn()


class FlashIF(Peripheral):
    """Flash interface registers: wait states (ACR); programming is not modelled."""

    def __init__(self, machine):
        super().__init__(machine, "FLASH", 0x40023C00)
        machine.flash_if = self

    def reset(self):
        self.r = {0x14: 0x80000000, 0x18: 0x0FFFAAED, 0x00: 0}

    def latency(self):
        return self.r[0] & 7

    def write(self, off, value, mask):
        super().write(off, value, mask)
        if off == 0x00:
            self.m.rcc.update_clocks()
        elif off == 0x04:
            if value == 0xCDEF89AB:
                self.r[0x14] &= ~0x80000000             # unlocked
        elif off == 0x10:
            self.r[0x10] = 0                            # SR: write 1 to clear, never busy


class PWR(Peripheral):
    gate = "PWR"

    def __init__(self, machine):
        super().__init__(machine, "PWR", 0x40007000)

    def reset(self):
        self.r = {0x00: 0x4000, 0x04: 0x4000}           # VOS scale 1, VOSRDY

    def read(self, off):
        if off == 0x04:
            return self.r.get(4, 0) | 0x4000
        return super().read(off)


class SYSCFG(Peripheral):
    gate = "SYSCFG"

    def __init__(self, machine):
        super().__init__(machine, "SYSCFG", 0x40013800)
        machine.syscfg = self

    def reset(self):
        self.r = {}

    def exti_port(self, line):
        reg = 0x08 + 4 * (line // 4)
        return (self.r.get(reg, 0) >> (4 * (line % 4))) & 0xF

    def read(self, off):
        if off == 0x20:                                  # CMPCR: READY when enabled
            v = self.r.get(0x20, 0)
            return v | (0x100 if v & 1 else 0)
        return super().read(off)


class IWDG(Peripheral):
    """Independent watchdog on the ~32 kHz LSI. Expiry resets the chip (not the rig)."""

    def __init__(self, machine, lsi_hz=32000):
        self.lsi_hz = lsi_hz
        self.ev = None
        super().__init__(machine, "IWDG", 0x40003000)

    def reset(self):
        if self.ev:
            self.ev.cancel()
        self.ev = None
        self.deadline = 0
        self.running = False
        self.unlocked = False
        self.pr = 0
        self.rlr = 0xFFF

    def _timeout_ps(self):
        div = 4 << self.pr
        return int((self.rlr + 1) * div / self.lsi_hz * PS_PER_S)

    def _kick(self):
        # refreshes are frequent: keep one event and move the deadline lazily
        self.deadline = self.m.now() + self._timeout_ps()
        if self.ev is None:
            self.ev = self.m.at(self.deadline, self._expire)

    def _expire(self):
        self.ev = None
        if self.m.now() < self.deadline:
            self.ev = self.m.at(self.deadline, self._expire)
            return
        self.m.log(f"[sim] t={self.m.now_s():.4f} s: IWDG watchdog expired "
                   f"(not refreshed within {self._timeout_ps()/1e9:.1f} ms)")
        self.m.rcc.note_reset(29)
        self.m.system_reset("IWDG watchdog")

    def read(self, off):
        if off == 0x04:
            return self.pr
        if off == 0x08:
            return self.rlr
        return 0                                         # SR: updates finish at once

    def write(self, off, value, mask):
        value &= mask
        if off == 0x00:
            k = value & 0xFFFF
            if k == 0x5555:
                self.unlocked = True
            elif k == 0xCCCC:
                self.running = True
                self.unlocked = False
                self._kick()
            elif k == 0xAAAA:
                self.unlocked = False
                if self.running:
                    self._kick()
            else:
                self.unlocked = False
        elif off == 0x04 and self.unlocked:
            self.pr = value & 7
        elif off == 0x08 and self.unlocked:
            self.rlr = value & 0xFFF


class CRC(Peripheral):
    gate = "CRC"

    def __init__(self, machine):
        super().__init__(machine, "CRC", 0x40023000)

    def reset(self):
        self.crc = 0xFFFFFFFF
        self.idr = 0

    def read(self, off):
        return {0: self.crc, 4: self.idr}.get(off, 0)

    def write(self, off, value, mask):
        if off == 0:
            crc = self.crc ^ (value & 0xFFFFFFFF)
            for _ in range(32):
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
            self.crc = crc
        elif off == 4:
            self.idr = value & 0xFF
        elif off == 8 and value & 1:
            self.crc = 0xFFFFFFFF


class RNG(Peripheral):
    gate = "RNG"

    def __init__(self, machine, seed=1):
        self.rng = random.Random(seed)
        super().__init__(machine, "RNG", 0x50060800)

    def read(self, off):
        if off == 4:
            return 1 if self.r.get(0, 0) & 4 else 0      # DRDY when enabled
        if off == 8:
            return self.rng.getrandbits(32)
        return super().read(off)
