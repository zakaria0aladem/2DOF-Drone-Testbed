# Testbed simulator

Run the real board firmware on your PC, with no board and no testbed.

The simulator pretends to be the Hiwonder board (STM32F407 at 168 MHz) wired
to the testbed: 4 ESCs and motors, the 2 joint encoders and the QMI8658 IMU.
You give it the same `.bin` or `.elf` file that STM32CubeIDE makes for the
real board. It runs that file instruction by instruction, so your code, the
HAL and the interrupts behave like on the board. The motors push the frame,
the frame moves, the encoders count and the IMU feels it.

How it works, what is and isn't simulated, and how to measure the numbers it
still guesses: [docs/SIMULATION.md](../docs/SIMULATION.md).

## Quick start (Windows)

* **Double-click `start_simulator.bat`** (in this `sim` folder). The first time
  it installs what it needs, then the simulator window opens with the demo
  firmware.
* **Drag your `.elf` or `.bin` onto `start_simulator.bat`** to run your own
  firmware in the window.

On Linux or macOS, or from a terminal: `python -m testbed_sim --gui`.

## 1. Install (once)

You need Python 3.10 or newer. In a terminal, in this `sim` folder:

```
pip install -r requirements.txt
python -m testbed_sim --check
```

`--check` tests the installation step by step (Python version, packages,
a short run of the demo firmware) and tells you how to fix what is missing.

## 2. Run the demo firmware

```
python -m testbed_sim --gui
```

With no firmware file, the simulator runs the demo firmware
(`firmware_demo/prebuilt/testbed_demo.bin`). `--gui` opens a window with:

* a **top view** of the frame turning on the yaw joint and a **side view** of
  it tilting on the pitch joint. The props change colour with throttle, and
  arrows show the thrust
* a **plot** of yaw and pitch over the last 10 s
* the four **ESCs**: pulse width, PWM rate, state (arming, armed, no-signal),
  motor rpm and thrust
* bus voltage and current, the LED, the buzzer, and whether the CPU is running
* buttons for **KEY1, KEY2** (hold them down with the mouse, like the real
  keys), **RESET**, pushing the frame by hand, **Pause** and the **speed**
* the text the firmware prints on USB port 1, and a box to type the
  commands below

Without `--gui` it runs in the terminal only. You see what the board prints
on USB port 1 (USART1), like a serial monitor:

```
RRC v1.2 testbed demo, SYSCLK 168 MHz
QMI8658 found, calibrating gyro (keep the rig still)
[sim] t=1.014 s: ESC1 armed (beep beep)
...
t=1 s bat=12002 mV enc0=0 enc1=0 disarmed
```

Lines starting with `[sim]` come from the simulator, not from your firmware.
While it runs you can type a command and press Enter (in the window: in the
box at the bottom):

| Command | What it does |
|---|---|
| `k1` | press KEY1 for 0.2 s (`k2` for KEY2) |
| `hold k1` / `release k1` | keep KEY1 pressed / let go |
| `reset` | press the RESET button |
| `push yaw 0.05` | push the frame by hand: 0.05 N m on the yaw joint for 0.1 s (`push pitch -0.02`) |
| `state` | print the true angles, motor speeds, ESC pulses, bus voltage... |
| `pause` / `resume` | stop and restart the simulated clock |
| `speed 0.5` | half real time (`speed 1` = real time, `speed 0` = as fast as possible) |
| `quit` | stop (Ctrl+C works too) |

## 3. Run your own firmware

Build in CubeIDE as usual, then point the simulator at the file in `Debug/`:

```
python -m testbed_sim "C:\path\to\drone_fw\Debug\drone_fw.elf" --gui
```

or drag the file onto `start_simulator.bat`.

Use the `.elf` when you can: then a crash is reported with the function name.
The simulator warns you when the firmware does something the real board
wouldn't like (a pin set to the wrong alternate function, a peripheral used
before its clock is on, ESC pulses the ESC won't accept...).

Useful options (`python -m testbed_sim --help` lists all):

| Option | Meaning |
|---|---|
| `--gui` | open the simulator window |
| `--check` | check the installation and stop |
| `--config my_rig.toml` | your measured values instead of the guesses (see below) |
| `--set power.volts=11.1` | change one value from the command line |
| `--duration 10` | stop after 10 simulated seconds |
| `--fast` | don't wait for the clock: run as fast as the PC can |
| `--csv run.csv` | save the true rig state 1000 times a second |
| `--port1 5334` | also share USB port 1 over TCP (then it isn't printed) |

## 4. Talk to it from a PC program (tbbridge, your controller)

USB port 2 (USART3, the Raspberry Pi / PC link) is shared on your computer
at `socket://localhost:5333`. Any Python program using pyserial can open it
by using that name instead of `COM9`. The telemetry tools are in `tools/`:

```
cd tools
python tbbridge.py log --port socket://localhost:5333 --arm --excite prbs --base 1250 --amp 80 --duration 10 --out runs/prbs.csv
```

That is exactly how you would log the real rig, so a script that works here
works on the testbed. To close the loop, give tbbridge a controller. The
example PID turns the yaw joint to 0.5 rad (watch it in the `--gui` window):

```
python tbbridge.py run --port socket://localhost:5333 --arm --controller examples/controller_template.py:HoldAngle --duration 10
```

Copy `examples/controller_template.py` to start your own controller. In your
own code:

```python
import serial
ser = serial.serial_for_url("socket://localhost:5333", timeout=0.01)   # or "COM9" on the real board
```

On Linux and macOS, `--pty` also makes a pseudo serial port (like
`/dev/pts/5`) for programs that can't use `socket://`.

## 5. Use it from Python

```python
from testbed_sim import Testbed
from testbed_sim import tbproto as P

tb = Testbed("firmware_demo/prebuilt/testbed_demo.bin")
tb.run(3.3)                                   # boot, ESCs arm
tb.write("USART3", P.encode(P.T_ARM))
for k in range(1000):                         # 2 s of motor commands, 2 ms apart
    tb.write("USART3", P.motor_cmd(k, [1300, 1200, 1200, 1300]))
    tb.run(0.002)
print(tb.state())                             # true yaw, pitch, motor rpm, ESC pulses...
print(tb.read_text("USART1"))                 # what the firmware printed
```

`examples/run_firmware.py` is a complete version of this.

## 6. For the AI course: the rig without firmware

`testbed_sim.plant.RigEnv` is the same physics and the same sensor errors,
but you set the four ESC pulses directly from Python. It runs about 16x
faster than real time, so it is the place to generate training data, test a
PIML model or a DeepC controller before trying them on the real rig.

```python
from testbed_sim.plant import RigEnv

env = RigEnv(dt=0.002)                 # 500 Hz, like the board's telemetry
obs = env.reset()
for k in range(5000):
    obs = env.step([1250, 1300, 1300, 1250])   # us for ESC1..ESC4
print(obs.q, obs.dq, obs.acc, obs.gyro)        # what the board would measure
print(obs.true_q, obs.true_dq)                 # the truth, to check a model
```

`examples/plant_prbs.py` records a system-identification run to a CSV with the
same column names as `tbbridge.py` (`t, q1, q2, dq1, dq2, ax..gz, u1..u4`), so
the same training code reads simulated and real data.

## 7. Make it match your rig

Every physical number is in `testbed_sim/default_config.toml`, marked
`[datasheet]`, `[project]` or `[placeholder]` (a guess). Copy the values you
measure into your own file, for example `my_rig.toml`:

```toml
[power]
current_limit_a = 3.0

[frame]
mass_kg = 0.52
```

and run with `--config my_rig.toml` (or `Testbed(fw, config="my_rig.toml")`,
`RigEnv(config="my_rig.toml")`). How to measure each guess is in
[docs/SIMULATION.md, section 6](../docs/SIMULATION.md#6-measuring-the-placeholder-values).

## 8. Check the simulator itself

```
python -m unittest discover tests
```

10 tests, about 15 seconds: boot, telemetry timing, closed-loop yaw and pitch,
command timeout, KEY1 kill, watchdog reset, crash report, the firmware-free
model, the console commands and the example controller.

## Folder map

```
start_simulator.bat double-click to open the simulator window (Windows)
testbed_sim/        the simulator (Python package)
  core.py           CPU, memory, time, interrupts
  periph/           STM32F407 peripherals: RCC, GPIO, timers, UART, I2C, DMA, ADC, NVIC...
  devices.py        QMI8658 IMU model
  rig.py            physics: ESCs, motors, props, joints, power supply
  board.py          Hiwonder board wiring (which pin goes where)
  plant.py          the rig without firmware, for the AI work
  api.py            Testbed class
  bridge.py         USB ports to TCP / pty, real-time pacing, console commands
  gui.py            the --gui window
  default_config.toml  every physical number
tools/              tbbridge.py, tbproto.py and the controller template, set up for the simulator
examples/           short scripts to start from
firmware_demo/      small CubeMX-style test firmware the simulator is checked against
tests/              automated checks
```
