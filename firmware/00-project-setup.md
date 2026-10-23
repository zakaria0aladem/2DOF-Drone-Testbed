# 00. Project setup: from nothing to a flashed, empty program

By the end of this chapter you have a CubeIDE project that runs the chip at 168 MHz, builds a `.bin`
file, and flashes it onto the board over USB. Do this once. Every later chapter builds on it.

## 1. Create the project

1. In STM32CubeIDE: **File → New → STM32 Project**.
2. In **Commercial Part Number** type `STM32F407VET6`, click the line **STM32F407VETx**, press **Next**.
3. Name it `drone_fw`. Untick "Use default location" and put it in the repo's `firmware/` folder
   (inside `Documents\GitHub\2DOF-Drone-Testbed`, never inside OneDrive). Leave Language = C,
   Binary Type = Executable, Project Type = STM32Cube. Press **Finish**, then **Yes** to open the
   Device Configuration view.

You now see a picture of the chip. That picture is the **`.ioc` file**: it stores every setting you
click, and CubeIDE turns it into C code when you save.

## 2. Debug = Serial Wire

**System Core → SYS → Debug = `Serial Wire`.**

The chip's debug port can use 5 pins (JTAG) or 2 pins (SWD, Serial Wire Debug). Serial Wire keeps
only PA13/PA14 for debugging and frees **PA15, PB3 and PB4**. We need those: PA15/PB3 are a joint
encoder and PB4 is an ESC output. PA13 and PA14 turn green.

## 3. Use the crystal

**System Core → RCC → High Speed Clock (HSE) = `Crystal/Ceramic Resonator`.**

The chip has a built-in 16 MHz clock (HSI), but it drifts by about 1%. The board has a 16 MHz quartz
crystal that is about 200× more accurate, and the 1 Mbaud serial link to the Pi needs that accuracy.
PH0 and PH1 (the crystal's two legs) turn green.

## 4. Run at 168 MHz

Open the **Clock Configuration** tab:

1. **Input frequency** (far left): `16`.
2. **PLL Source Mux**: `HSE`.
3. **System Clock Mux**: `PLLCLK`.
4. **HCLK (MHz)**: type `168`, press Enter, accept the automatic solve.

Check the PLL boxes: **/M = 8, ×N = 168, /P = 2** (16 ÷ 8 × 168 ÷ 2 = 168 MHz). You should also see
APB1 = 42 MHz, APB2 = 84 MHz, APB1 timer clocks = 84 MHz, APB2 timer clocks = 168 MHz. The timer
numbers matter later: a timer at 168 MHz needs prescaler 167 to tick once per microsecond, and one
at 84 MHz needs 83.

## 5. Generate the code

Press **Ctrl+S** and answer **Yes** to "generate code". Open `Core/Src/main.c`. `main()` runs:

1. `HAL_Init()`: starts ST's HAL library and the 1 ms SysTick timer (behind `HAL_Delay` and `HAL_GetTick`).
2. `SystemClock_Config()`: your clock settings (crystal on, PLL to 168 MHz).
3. `MX_GPIO_Init()`: your pin settings.
4. `while (1) { }`: runs forever. Your code goes here.

**Rule:** only type between `/* USER CODE BEGIN ... */` and `/* USER CODE END ... */`. Everything
else is rewritten each time you save the `.ioc`.

## 6. Make the build produce a .bin file

Right-click **drone_fw → Properties → C/C++ Build → Settings → Tool Settings → MCU/MPU Post build
outputs**. Tick **Convert to binary file (-O binary)** (and Intel Hex if you like). **Apply and Close.**

Build with the **hammer** icon. The Console must end with `0 errors`, and `Debug/drone_fw.bin`
appears.

## 7. Flash it over USB

Do **not** use FlyMcu on this board: it mistakes the F407 for an older chip and turns on read
protection. Use `stm32loader`:

1. Once: install Python 3, then `pip install stm32loader`.
2. Plug a USB-C cable into **port 1** (USART1). In Device Manager, note the COM port under
   "Ports (COM & LPT)". No port? Install WCH's CH343 driver for the CH9102 chip.
3. Unplug and replug the USB, then in a terminal:

```
cd <path to>\drone_fw\Debug
stm32loader -p COM9 -f F4 -R -B -e -w -v -g 0x08000000 drone_fw.bin
```

Replace `COM9` with your port. The flags mean: `-f F4` chip family, `-R -B` set the polarity of the
USB chip's DTR/RTS lines, which this board wires to RESET and BOOT0 so the PC can restart the chip
into its built-in bootloader, `-e` erase, `-w` write, `-v` verify, and
`-g 0x08000000` start the program from the start of flash.

4. **Unplug and replug** (or press reset) afterwards: the chip can stay in the bootloader after
   `stm32loader` exits.

**Check:** the board ships with Hiwonder's program, which blinks the LED. After flashing your empty
program, **the LED stops blinking**. That proves your code is running.

Next: [01. LED module](01-led.md).
