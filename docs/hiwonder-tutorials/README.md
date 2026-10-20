# Hiwonder tutorials

Hiwonder's lessons for the ROS Robot Controller. They use **Keil + FreeRTOS**; we use **STM32CubeIDE**, so read them for board facts and pin usage, not for tool steps.

- `1-stm32-basics/` – setup, first LED program, compile and download, SWD debugging.
- `2-program-analysis/` – one lesson per board feature. The ones that matter for the testbed:
  Lesson 2 (LED, buzzer), 3 (keys), 4 (battery ADC), 5 (QMI8658 IMU), 6–7 (PWM, same idea as ESC signals), 10 (encoders), 17 and the host-computer lesson in extras (serial link).
- `3-extras/` – bus servos, PS2 receiver, LCD/LVGL, host-computer protocol, battery safety, servo manual. Mostly not used on the testbed.

Some downloads came without lesson numbers, so those sit in `3-extras/`. The lesson source code (.rar/.zip) is not stored here.
