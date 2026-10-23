# Firmware tutorial: the testbed controller, one module at a time

This tutorial builds the STM32 firmware for the 2-DOF drone testbed from an empty project. Each chapter
adds **one module** (one device on the board), explains every line, and ends with a check you can do
on the real board. You can follow it with no earlier STM32 experience.

**Board:** Hiwonder ROS Robot Controller **v1.2**, chip **STM32F407VET6** (Arm Cortex-M4, 168 MHz).
**Tools:** STM32CubeIDE (free, from st.com) and `stm32loader` (Python) for flashing over USB.
Hiwonder's own lessons use Keil and FreeRTOS; we use neither.

## How the code is organized

Every device gets its own **module**: two files in the `BSP` folder (BSP = *Board Support Package*,
the code that knows how this particular board is wired).

- **`bsp_xxx.h`, the "menu".** Lists only the functions other files are allowed to call.
- **`bsp_xxx.c`, the "kitchen".** Holds the code that actually touches pins and timers. Pin names,
  active-low tricks and timer numbers stay hidden in here.

Every module follows the same two-call pattern:

- `XXX_Init()`: called **once** at start-up.
- `XXX_Update()`: called **every pass** of the main loop. It must never wait (no `HAL_Delay`), so all
  modules can share the loop.

`main.c` only includes the headers and calls those functions. If a pin changes, only one module
changes.

```
drone_fw/
 ├─ drone_fw.ioc        chip settings (pins, clock, timers), edited in CubeMX
 ├─ Core/Src/main.c     #include + XXX_Init() once + XXX_Update() in the loop
 └─ BSP/
     ├─ bsp_led.h/.c      status LED, PE10            chapter 01
     ├─ bsp_buzzer.h/.c   buzzer, PA8                 chapter 02 (planned)
     ├─ bsp_key.h/.c      keys K1/K2, PE1/PE0         chapter 03 (planned)
     ├─ esc.h/.c          4 ESC PWM outputs, 400 Hz   (planned)
     ├─ encoder.h/.c      2 joint encoders, TIM2/TIM5 (planned)
     ├─ imu.h/.c          QMI8658 IMU over I2C2       (planned)
     └─ link.h/.c         serial link to the Raspberry Pi, USART3 (planned)
```

## Chapters

| # | Chapter | You end with |
|---|---|---|
| 00 | [Project setup](00-project-setup.md) | an empty project at 168 MHz, built and flashed |
| 01 | [LED module](01-led.md) | `bsp_led`: on, off, toggle and a blink that never blocks |
| 02 | Buzzer module (planned) | `bsp_buzzer` |
| 03 | Keys module (planned) | `bsp_key` |

## Board facts used in this tutorial

| Item | Pin | Note |
|---|---|---|
| Crystal (HSE) | PH0 / PH1 | **16 MHz** on v1.2 (older v1.0/v1.1 material says 8 MHz: don't copy it) |
| Status LED | PE10 | **active-low**: pin LOW = LED on |
| Buzzer | PA8 | high = on, through an S8050 transistor; probably needs a tone |
| Keys K1 / K2 | PE1 / PE0 | board pull-ups, pressed = LOW |
| Flashing port | USB port 1 = USART1 (PA9/PA10) | CH9102 USB-serial chip |
| Pi link | USB port 2 = USART3 (PD8/PD9) | |

Full pin map: `Hiwonder_RRC_v1.2_pinmap.md` (ask Zakaria for a copy).
