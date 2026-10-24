# Flashing the board with stm32loader

Flashing goes over **USB port 1** (USART1, PA9/PA10) into the F407's built-in bootloader.
The board wires the USB chip's DTR/RTS lines to RESET and BOOT0, and `stm32loader` uses them to
put the chip into the bootloader by itself. STM32CubeProgrammer doesn't do this ("Activating device: KO"),
and FlyMcu turns on read protection, so use neither.

## Setup (once)

1. Install Python 3, then: `pip install stm32loader`
2. Plug a USB-C cable into **port 1**. In Device Manager → Ports (COM & LPT) note the
   `CH9102` COM port (COM9 on Zakaria's laptop). No port? Install WCH's CH343 driver.
3. In CubeIDE: **DRONE → Properties → C/C++ Build → Settings → MCU/MPU Post build outputs**, tick
   **Convert to binary file (-O binary)**. Builds now also produce `Debug\DRONE.bin`.

## Flash (every time)

1. Build in CubeIDE (hammer, `0 errors`).
2. Close anything holding the COM port (serial monitor, CubeProgrammer).
3. Unplug and replug USB port 1, then:

```
cd firmware\DRONE\Debug
stm32loader -p COM9 -f F4 -R -B -e -w -v -g 0x08000000 DRONE.bin
```

4. Unplug and replug (or press reset): the chip can stay in the bootloader afterwards.

Flags: `-f F4` chip family, `-R -B` DTR/RTS polarity for this board's RESET/BOOT0 wiring,
`-e` erase, `-w` write, `-v` verify, `-g 0x08000000` run from the start of flash.

## If it fails

| Message | Fix |
|---|---|
| `could not open port` / access denied | another program has the port open: close it |
| no reply / timeout | replug USB, check it's port 1 and the right COM number |
| `No such file` | build first, and check the .bin option above |
