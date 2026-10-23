# Testbed wiring

How the testbed is wired to the Hiwonder ROS Robot Controller v1.2 (2026-10-23). Pinout read from the board's V1.1 schematic (`docs/datasheets/RRC V1.1 Schematic.pdf`, motor encoder section). Values marked *check* still need a measurement on our v1.2 board.

## Power

```
Bench PSU 12.0 V ──┬── XT60 ──> 4-in-1 ESC ──> 4 motors   (motor current)
   (+ and – only)  └── screw terminal ──> Hiwonder board  (logic current, < 1 A)
```

- PSU at **12.0 V**, never above **12.6 V** (the board's input range is 5–12.6 V). Later: a **3S LiPo** (12.6 V full).
- Use the PSU's red **+** and black **–** only. The green GND terminal is mains earth, leave it empty.
- Start with a current limit of about 0.5 A for the first power-up, then about 3 A. Four motors can pull more than the PSU's 5 A, so keep throttle low. If the C.C light comes on, the PSU has hit its current limit.
- The Y-cable splits one PSU lead into the XT60 (ESC) and a bare pair (board). The ESC and the board share ground through it.
- Screw terminal: twisted bare strands, not soldered (solder works loose under a screw).
- Yahboom YB-MPP01-V2.0 power board (6–24 V in, 5 V out): powers the **Raspberry Pi** through its USB-C output.

## M ports (6-pin, 2.0 mm)

Every M port has the same pinout:

| Pin | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|
| Signal | motor out B | **GND** | A | B | 5 V | motor out F |

- **Never connect anything to pins 1 and 6.** They are the brushed-motor driver outputs (battery voltage).
- Pins 3 and 4 go straight to the STM32 (no pull-ups or filters). All of them are **FT** (5 V tolerant) in the F407 datasheet.
- *check* on the board, power off: pin 2 beeps to the – screw terminal, pin 6 does not.

## What goes where

| M port | Used for | Pin 3 | Pin 4 | Timer | Pin 5 | Pin 2 |
|---|---|---|---|---|---|---|
| Motor 1 | Yaw encoder | A → PA0 | B → PA1 | TIM5 encoder mode | 5 V to encoder | GND to encoder |
| Motor 2 | Pitch encoder | A → PA15 | B → PB3 | TIM2 encoder mode | 5 V to encoder | GND to encoder |
| Motor 3 | ESC 1 + ESC 2 | PB6 | PB7 | TIM4 CH1 / CH2 PWM | not used | ESC GND |
| Motor 4 | ESC 3 + ESC 4 | PB4 | PB5 | TIM3 CH1 / CH2 PWM | not used | ESC GND |

- Encoder cable order is `5V A G B`, the port is `G A B 5V` on pins 2–5. Make an adapter cable with a PH2.0 6-pin plug.
- ESCs get **signal + GND only**.
- Shared wires are fine: all pin 2s are the same GND and all pin 5s the same 5 V. Our two encoders share one 5 V and one GND wire (to Motor 1 pin 5 / pin 2), and the ESC has one GND wire (to Motor 3 pin 2). The other ports only get their A/B or signal wires.
- Which ESC output drives which motor: *check* and write it here.

## Timers

- TIM3 and TIM4 run at 84 MHz. **PSC = 83** (1 tick = 1 µs), **ARR = 2499** (2.5 ms period = **400 Hz**).
- **CCR = pulse width in µs**: 1000 = stopped, 2000 = full throttle.
- First tests at 50 Hz (ARR = 19999) to confirm the ESC arms, then 400 Hz. The ESC has no datasheet.
- PB4 is a JTAG pin at reset: CubeMX **SYS → Debug = Serial Wire**.
- Encoders: TIM5 and TIM2 are 32-bit. 720 CPR x4 = 2880 counts per revolution.

## ESC start-up

1. Set every CCR to 1000 **before** `HAL_TIM_PWM_Start`.
2. Hold 1000 for about 3 s until the ESC's arming beep.
3. Never power up with a pulse above 1000 (2000 at power-up can start ESC calibration).
4. If commands stop arriving (Pi link lost), the firmware sets all CCRs back to 1000.

Props off for the first tests.

## USB

- **Port 1** (USART1, PA9/PA10): laptop, flashing and console (COM9).
- **Port 2** (USART3, PD8/PD9, 1 Mbaud): Raspberry Pi.
