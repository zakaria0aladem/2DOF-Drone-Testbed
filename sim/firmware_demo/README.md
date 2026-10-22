# Demo firmware

A small CubeMX-style program for the Hiwonder board, used to check the
simulator. It sets the clock to 168 MHz, starts the four ESC PWM outputs,
the two encoder timers, the QMI8658 on I2C2, the battery ADC and the
watchdog, prints a status line on USB port 1 every second, and runs the
telemetry protocol on USB port 2 (USART3, 1 Mbaud, DMA): 500 Hz telemetry,
ARM / DISARM / MOTOR_CMD from the PC, 3 s ESC arming wait, 100 ms command
timeout, KEY1 kill switch.

`prebuilt/testbed_demo.bin` is ready to run:

    python -m testbed_sim firmware_demo/prebuilt/testbed_demo.bin

It was written for the simulator tests and has not been tried on the real
board: use your own CubeIDE project for the testbed.

To rebuild it without CubeIDE: `pip install ziglang`, then `python build.py`
(`--debug` for -O0). The first build downloads ST's HAL and CMSIS into `_deps/`.
