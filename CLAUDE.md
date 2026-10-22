# CLAUDE.md

Notes for Claude Code working in this repo.

## Who and what

- Two students share this repo: Zakaria (owner, physics-informed ML) and Amr (DeepC, data-driven predictive control).
- Embedded course: fly a 2-DOF drone testbed (quad X-frame on a yaw and a pitch joint) from our own STM32 firmware, with no flight controller.
- AI course: use the testbed (and `sim/`) as the plant for learned models.
- Zakaria is a beginner programmer, new to STM32. Explain what you change and why in plain words, define new terms, and keep steps small.

## Rules

- Commits are authored by Zakaria only. Never add `Co-Authored-By: Claude` or other Claude trailers, and don't list Claude as a contributor.
- Don't push unless asked. Each person works in their own folder (`ai/piml/`, `ai/deepc/`). Pull before starting.
- The clone should live outside OneDrive (e.g. `Documents\GitHub`).

## Hardware (Hiwonder ROS Robot Controller v1.2)

- STM32F407VET6, 16 MHz HSE, 168 MHz SYSCLK (PLL M=8 N=168 P=2).
- IMU: onboard QMI8658 on I2C2 (PB10 SCL, PB11 SDA), address 0x6A, INT on PB12. No magnetometer.
- Joint encoders: TIM5 (PA0/PA1) and TIM2 (PA15/PB3), both 32-bit.
- ESCs at 400 Hz, 1000-2000 us, CCR = pulse in us:
  - ESC1: PA11, TIM1_CH4
  - ESC2: PB4, TIM3_CH1 (needs SYS Debug = Serial Wire)
  - ESC3: PC8, TIM8_CH3
  - ESC4: PC9, TIM8_CH4
  - TIM1/TIM8: PSC=167, ARR=2499. TIM3: PSC=83, ARR=2499.
- LED PE10 (active low), keys PE1/PE0 (press = low), buzzer PA8, battery on PB0 (ADC1_IN8, x11 divider).
- USB port 1 = USART1 (PA9/PA10, flashing + console). USB port 2 = USART3 (PC / Raspberry Pi link, 1 Mbaud).
- Toolchain: STM32CubeIDE + HAL, no ST-Link. Flash over USB port 1 from the `Debug` folder (replug USB or press reset after):
  `stm32loader -p COM9 -f F4 -R -B -e -w -v -g 0x08000000 drone_fw.bin`
  Never use FlyMcu/mcuisp on this board: it turns read protection on.

## Simulator (`sim/`)

- `python -m testbed_sim <firmware.elf|.bin>` runs the real firmware in Unicorn (Cortex-M4). Simulated parts: F407 peripherals, QMI8658, ESCs, motors, encoders, rig physics.
  - USART1 prints to the console.
  - USART3 is on `socket://localhost:5333`.
- `sim/tools/tbbridge.py` talks to the sim the same way it talks to the real board (`--port socket://localhost:5333` instead of `COM9`).
- `testbed_sim.plant.RigEnv` is the same physics without firmware, about 16x real time, for AI data.
- Physical numbers: `sim/testbed_sim/default_config.toml`, each tagged `[datasheet]`, `[project]` or `[placeholder]`. How to measure the placeholders: `docs/SIMULATION.md` section 6.
- Tests: `cd sim && python -m unittest discover tests` (7 tests, about 10 s). Run them after any change in `sim/`.
- Keep `unicorn==2.1.4`. Exact timing reads Unicorn's internal instruction counter via ctypes (`core.py`, `_find_counter_offset`), so another version can break it.
- Known firmware issue: the telemetry `tx_frame()` drops a frame (including HELLO) while the previous frame is still sending. `tbbridge.py` works around it by retrying PING.
