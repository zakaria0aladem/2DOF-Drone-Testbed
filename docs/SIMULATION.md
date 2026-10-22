# How the testbed simulator works

The simulator (in [`sim/`](../sim)) lets you run the board firmware without
the board or the testbed. This page explains how it works, how close to the
real hardware it is, and how to measure the numbers it still guesses. To just
use it, start with [sim/README.md](../sim/README.md).

## 1. The idea in one paragraph

Your firmware is a list of ARM Cortex-M4 instructions. The simulator reads
them one by one and does what the real CPU would do (it uses Unicorn, the CPU
part of the QEMU emulator). Whenever an instruction reads or writes a
peripheral register (for example `TIM1->CCR4 = 1500` or reading `I2C2->SR1`),
the simulator answers like the STM32F407's peripheral would. The peripherals
are connected like on the Hiwonder board: the timers' PWM pins drive four
simulated ESCs, the ESCs drive motor models, the motors' thrust moves a
simulated 2-DOF frame, and the frame's angles come back through the encoder
inputs and the simulated QMI8658 on I2C2. Time is simulated too: one
instruction takes one clock cycle at whatever clock your firmware sets
(168 MHz normally), so `HAL_Delay(100)` lasts 100 ms of simulated time, a
400 Hz PWM is 400 Hz, and your control loop runs at its real rate.

```
your .bin/.elf -> CPU emulator -> peripheral registers (RCC, GPIO, TIM, I2C, USART, DMA, ADC, NVIC, IWDG...)
                                      |            |              |             |
                                   ESC pins     encoder pins    I2C2 bus      USB ports 1 and 2
                                      v            ^              ^             v
                                  4 ESC models -> motor models -> 2-DOF frame -> QMI8658 model     your PC (TCP / Python)
```

## 2. What is simulated

### The chip (STM32F407VET6)

| Part | Simulated |
|---|---|
| CPU | Cortex-M4 with FPU: all instructions, exceptions and interrupts with priorities, PRIMASK/BASEPRI, MSP/PSP, FPU stacking, SysTick, SVC/PendSV, HardFault on bad memory or bad instructions |
| Memory | 512 KB flash (also at address 0), 128 KB SRAM, 64 KB CCM, bit-band regions, unique ID, flash size |
| RCC | HSE 16 MHz, HSI, LSI, PLL (M/N/P/Q), bus prescalers, timer clock doubling, clock enable bits, peripheral resets, reset flags. Reading or writing a peripheral whose clock is off gives a warning and does nothing, like the chip |
| FLASH, PWR | wait states (warns if too few for the clock), voltage scaling ready bit |
| GPIO A-I | modes, alternate functions, pull-ups/downs, open drain, BSRR, inputs from the board |
| EXTI, SYSCFG | edge interrupts on any pin |
| Timers TIM1-TIM14 | counting, prescaler, auto-reload with preload, update and compare interrupts, PWM and other output modes on all channels, MOE on TIM1/TIM8, one-pulse, encoder mode on TIM2/3/4/5 |
| USART1-6 | baud rate from BRR and the bus clock, real byte timing, TXE/TC/RXNE/IDLE/ORE flags, interrupts, DMA, overruns when bytes aren't read in time |
| I2C1-3 | master mode with the ST HAL's flag sequence (SB, ADDR, TXE, BTF, RXNE, AF, STOP), 100/400 kHz byte timing, interrupts and DMA |
| DMA1, DMA2 | all streams and channels for the simulated peripherals, normal, circular and double-buffer modes, half/full transfer interrupts |
| ADC1-3 | single, scan and continuous conversions with sampling time, internal channels; PB0 reads the battery divider |
| IWDG | the watchdog resets the chip if not refreshed in time (LSI 32 kHz) |
| CRC, RNG, DWT cycle counter, ITM | yes |

Not simulated: SPI (registers only), CAN, USB OTG, Ethernet, FSMC, RTC, DAC,
timer input capture, timer master/slave triggers (except encoder mode),
timer DMA, ADC conversions started by a timer, writing to flash, low-power
modes beyond WFI. If your firmware uses one of these you get a warning
saying so, and the peripheral behaves like plain memory.

### The board and the testbed

| Part | How it is simulated |
|---|---|
| Wiring | the Hiwonder RRC v1.2 pin map: USB port 1 = USART1 PA9/PA10, USB port 2 = USART3 PD8/PD9, QMI8658 on I2C2 PB10/PB11 (INT on PB12), KEY1 PE1, KEY2 PE0, LED PE10, buzzer PA8, battery voltage on PB0 through the 1:11 divider |
| ESCs (x4, on PA11, PB4, PC8, PC9) | read the pulse width of the PWM on their pin; arm after seeing a low pulse for a while; ignore pulses outside the valid range or too fast; stop the motor when the signal is lost; start-up delay from standstill; throttle range |
| Motors (Cobra 2204 1960 KV) | electric motor model: current from duty x bus voltage minus back-EMF over the winding resistance, torque from current, prop drag ~ speed squared, rotor inertia. This gives the real spin-up lag |
| Props | thrust and drag torque proportional to speed squared |
| Power supply | constant voltage up to the current limit, then the voltage sags (as a bench supply does) |
| Frame on the 2 joints | rigid body dynamics on a yaw joint and a pitch joint (gravity with the centre of mass offset, the motors' thrust and reaction torques, rotor gyroscopic effects, coupling between yaw and pitch), joint friction with stiction, end stops |
| Encoders (E8T-720, x2) | quadrature A/B signals into TIM5 (yaw) and TIM2 (pitch), 2880 counts per turn |
| QMI8658 IMU | register-level: WHO_AM_I, reset, CTRL1-CTRL9, ranges, output rates, low-pass filters, data-ready flags and INT pulse, burst reads with auto-increment, turn-on time. Readings come from the frame's motion (gravity, rotation, centripetal acceleration at the chip position) plus noise at the datasheet noise density, bias, scale error, slow bias drift and motor vibration |
| Keys, LED, buzzer | keys with the board pull-ups; LED and buzzer state reported by the simulator |

## 3. How close to the hardware is it?

What matches the real board closely:

* Your code runs as compiled, with the real HAL. Logic bugs, wrong register
  settings, wrong pins, interrupt priority mistakes, stack overflows, DMA
  buffer mistakes, watchdog resets and HardFaults show up here as on the board.
* Peripheral timing: UART bytes take 10 bit times, I2C bytes 9 clocks, timer
  periods come from the real prescaler and clock tree, so a wrong clock
  setting gives the wrong baud rate or PWM frequency here too (the simulator
  prints the baud rate it sees, for example `USART3 running at 1000000 baud`).
* The same run gives the same result every time when you drive it from
  Python (noise comes from a seeded random generator; change `sim.seed` for
  a different noise sequence). With a live PC program on the TCP port, the
  PC's own timing varies a little from run to run, as with the real board.

What is approximate:

* **CPU speed.** Every instruction counts as one clock cycle. A real Cortex-M4
  takes 1 to ~3 cycles for most instructions, plus flash wait states that the
  ART accelerator mostly hides. So code runs a bit faster in the simulator
  than on the board, maybe up to 1.5x for heavy maths. Timer, UART and I2C
  timing don't depend on this; only how long your own computations take does.
* **Bus timing.** Register accesses cost 4 cycles, interrupt entry and exit
  12 cycles each. DMA moves are instant per item.
* **The physical numbers.** The parts marked `[placeholder]` in
  `default_config.toml` are educated guesses (prop thrust, inertias, friction,
  ESC curve...). The behaviour has the right shape (motor lag, saturation,
  stiction, end stops, voltage sag) but the exact numbers will differ from the
  testbed until you measure them (section 6).
* **ESC firmware.** The real ESC has no datasheet. The model behaves like a
  typical BLHeli_S ESC (arming, throttle range, signal timeout). Beeps,
  calibration mode and ESC settings menus are not simulated.
* **Aerodynamics.** No airflow between the props, no ground or wall effects,
  no turbulence. Prop thrust is a clean k x speed squared.
* **Sensors.** No temperature drift or cross-axis sensitivity on the IMU; the
  encoders never miss counts; ADC readings have no noise.

A good rule: if something fails in the simulator, it will fail on the board.
If it works in the simulator, it will very likely work on the board, but
tune gains and limits on the real rig.

## 4. Time and speed

The simulator keeps its own clock. When the firmware is waiting (polling a
flag, `HAL_Delay`, `while (!done) {}`, `WFI`), the simulator notices that
nothing changes and jumps straight to the next event (the next UART byte, the
next timer tick, the next physics step). That is why it can keep up with real
time even though emulating a CPU in Python is slow.

* Optimised builds (CubeIDE `Release`, or `-O2`/`-Os`): faster than real time
  on a normal laptop.
* CubeIDE's default `Debug` build (`-O0`) makes the CPU do 5-10x more work:
  the simulator then runs about 1.5x slower than real time. That's fine for
  testing logic (the simulated timing stays exact, the run just takes longer
  on your clock). For long runs, use `-Og` (Project > Properties > C/C++
  Build > Settings > MCU GCC Compiler > Optimization: "Optimize for debug")
  or the Release configuration.

When a PC program is connected (`socket://localhost:5333`), the simulator
waits for the wall clock so the program sees the board at its real speed.
If the PC can't keep up, it prints `running at 0.9x real time`: the program
then just sees a slightly slower board. `--fast` turns the waiting off.

## 5. Settings: default_config.toml

All physical numbers are in [`sim/testbed_sim/default_config.toml`](../sim/testbed_sim/default_config.toml),
with units and a tag:

* `[datasheet]` from a datasheet in `docs/datasheets` or the part's label/spec (see `docs/hardware`)
* `[project]` decided or confirmed in the project (pin map, wiring, PSU setting)
* `[placeholder]` a guess, to be measured

Put your measured values in your own small `.toml` (only the lines you
change) and pass it with `--config`. Unknown keys are an error, so a typo
doesn't go unnoticed.

## 6. Measuring the placeholder values

You don't need all of these. The ones that matter most for control work are
marked **(important)**. Use the bench app or tbbridge on the real rig for the
motor ones, and the encoder readings (in telemetry, q1/q2) for the angles.

### Power

* `power.current_limit_a`: what you set on the bench supply's current knob.
* `power.board_current_a`: the current the supply shows with the board
  powered and the ESC unplugged.

### ESC

* `esc.throttle_min_us` **(important)**: with a prop on and the rig held
  still, raise the pulse from 1000 us in 10 us steps; note the first value
  where the motor keeps spinning. Do all four.
* `esc.throttle_max_us`: usually 1950-2000 us. Leave it unless you calibrated
  the ESC's range.
* `esc.arm_below_us`, `esc.arm_time_s`: note how long after power-up (with
  1000 us pulses) the "ready" beeps come.
* `esc.signal_timeout_s`: unplug the signal wire while a motor spins slowly
  (no prop!) and time how long it keeps spinning.
* `esc.startup_delay_s`: from a stop, step to a low throttle and film the
  motor (phone slow-motion) to see how long it takes to start.
* `esc.active_braking`: if the motor stops much faster when you drop the
  throttle than when you cut the power, braking is on.

### Motors and props

* `motor.resistance_ohm`: phase-to-phase resistance with a good multimeter,
  divided by 2. It is tiny (~0.1 ohm): subtract the reading with the probe
  tips shorted.
* `motor.no_load_current_a`: run one motor without its prop at about half
  throttle; the supply current minus `board_current_a`.
* `prop.thrust_coeff` **(important)**: put the frame (or one arm) on a kitchen
  scale, run one motor at a few throttles and note the scale change (grams x
  0.00981 = newtons) and the motor speed (an optical tachometer or a phone
  strobe app; rad/s = rpm x 0.1047). Then `thrust_coeff = thrust / speed^2`.
  Without a speed meter: tune the value until the simulated motor at the same
  pulse gives the same thrust.
* `prop.torque_coeff`: for small props it's about 1/60 of `thrust_coeff`
  (newton metres per newton is ~ 1.6 cm). For a better value, use the yaw
  test below with all four motors in a yaw pattern.
* `motor.rotor_inertia`: prop mass m (kg) and radius r (m): about m r^2 / 3,
  plus ~2e-6 for the motor bell.
* `motor.positions`, `motor.spin` **(important)**: spin each ESC output alone
  (bench app) and write down which motor turns and which way, seen from above
  (anticlockwise = +1). Positions in metres from the pitch axis, x forward,
  y left.

### Frame and joints

* `frame.mass_kg`: weigh everything that tilts with the pitch joint.
* `frame.cg_below_pitch_axis_m` **(important)**: lock yaw, motors off. Tie a
  small known weight W (newtons = grams x 0.00981) at a known distance d (m)
  from the pitch axis on the nose and read the angle th it settles at
  (encoder). Then `m g l = W d / sin(th)`, so `l = W d / (mass_kg x 9.81 x
  sin(th))`. If the frame falls over to one side by itself, l is negative
  (top-heavy).
* `frame.inertia_yy` **(important)**: give the frame a small tilt and let it
  swing (motors off, yaw locked). Measure the period T of the swing from the
  encoder log. Then `inertia_yy = m g l (T / 2 pi)^2` with the m g l above.
  (Only works if the frame swings, i.e. l > 0.)
* `frame.inertia_zz` + `frame.yaw_arm_inertia` **(important)**: run the yaw
  test: from rest, run a yaw motor pattern (ESC1+ESC4 vs ESC2+ESC3) and
  measure the angular acceleration (second derivative of q1, or the slope of
  gz) right after the motors are up to speed. Easier: compare the simulated
  and measured yaw response for the same commands and adjust the two
  inertias together.
* `frame.inertia_xx`: from a CAD model, or set it to `inertia_yy`.
* `joints.*.coulomb`, `joints.*.viscous`: give the joint a push by hand
  (motors off) and log the encoder while it coasts to a stop. The slowing
  down is `(coulomb + viscous x speed) / inertia`: a straight-line decline in
  speed means mostly coulomb, an exponential one mostly viscous. Fit both
  with a few lines of numpy.
* `joints.*.stiction`: hang small weights on a string at a known radius until
  the joint just starts to move; torque = weight x radius.
* `joints.*.limits`: move the joint by hand to each stop and read the
  encoder angle. Empty `[]` means it turns freely.

### Sensors

* `encoders.yaw_sign`, `encoders.pitch_sign`: turn yaw anticlockwise seen
  from above, and pitch nose-down; the encoder count should go up. If it goes
  down, set the sign to -1.
* `imu.rotation`, `imu.position`: look at the axis arrows printed next to the
  QMI8658 on the board and how the board is mounted on the frame. Each row of
  `rotation` says which body axis (x forward, y left, z up) each chip axis
  points along. `position` is the chip's distance from the pivot in metres.
* `imu.accel_bias_mg`, `imu.gyro_bias_dps`: with the rig level and still,
  average 10 s of readings. Gyro average = gyro bias; accel average minus
  (0, 0, 1 g) = accel bias (in the chip axes).
* `imu.vibration_ms2`: run all motors at a constant pulse and compute the
  standard deviation of the accelerometer readings; tune the value until the
  simulated spread matches.
* `board.lsi_hz`: the watchdog's clock varies from chip to chip (17-47 kHz).
  Only matters if your watchdog timeout is tight.

## 7. Troubleshooting

| You see | Meaning |
|---|---|
| `ESC1 won't arm: its pulse is 1500 us` | the ESC must see a low pulse (< `arm_below_us`) for `arm_time_s` after power-up before it accepts throttle, like the real one |
| `firmware read TIM3 while its clock is off` | you forgot `__HAL_RCC_TIM3_CLK_ENABLE()` (CubeMX normally adds it in the MSP init) |
| `ESC2 pin PB4 is set to alternate function AF0, which is no timer channel on PB4` | the pin's alternate function is wrong (PB4 needs AF2 for TIM3_CH1); compare with the pin map |
| `ESC1 pin PA11 is TIM1 CH4, but ...` | the pin is right but the timer isn't producing PWM (not started, MOE off, CCR 0...) |
| `ESC1 ignores its signal: 1500 us pulses at 2000 Hz` | the PWM is faster than the ESC accepts (or the pulse is out of range): check PSC/ARR |
| `USART3 running at 115200 baud` when you expected 1000000 | the baud rate the firmware actually set (check the clock tree and BRR) |
| `USART3 RX overrun` | bytes arrived while the firmware wasn't reading (the same would happen on the board) |
| `HardFault: ... PC=0x080012A4 in my_function()` | a crash, with the function where it happened (use the .elf to get the name) |
| `IWDG watchdog expired` | the main loop stopped refreshing the watchdog in time |
| `... is not modelled; it behaves like plain memory` | that peripheral isn't simulated (section 2) |
| `running at 0.8x real time` | the PC is slower than the simulated board; results are still exact |
| `Address already in use` | another simulator is still running; close it or use `--port2 5335` |
| tbbridge says `no HELLO from board` | the firmware doesn't answer on USART3 yet (still booting?), or the port name is wrong |
| `a Python package is missing` | in the `sim` folder run `pip install -r requirements.txt` |
| something else doesn't work | run `python -m testbed_sim --check`: it tests the installation step by step and says what to fix |
