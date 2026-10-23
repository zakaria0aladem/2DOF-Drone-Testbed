# 01. LED module (`bsp_led`)

The board's status LED will tell us what the firmware is doing (booting, armed, fault) without a
screen. This chapter puts it in its own module, and introduces the pattern every later module uses.

**You need:** the project from [00. Project setup](00-project-setup.md).

## 1. How the LED is wired: active-low

On this board the LED lights when pin **PE10** is **LOW (0 V)**, as Hiwonder's own code confirms. The
usual wiring for that is LED + resistor between **3.3 V** and the pin: with the pin at 0 V current
flows through the LED; with the pin at 3.3 V both ends are equal and nothing flows.

| Code writes PE10 | Pin voltage | LED |
|---|---|---|
| `GPIO_PIN_RESET` (0) | 0 V | **on** |
| `GPIO_PIN_SET` (1) | 3.3 V | off |

"On when the pin is low" is called **active-low**. It's easy to get backwards, which is a good reason
to hide it inside the module: the rest of the code just calls `LED_On()`.

## 2. Set up PE10 in the .ioc

1. In the chip picture, left-click **PE10** and choose **`GPIO_Output`**.
2. **System Core → GPIO → PE10**:
   - **GPIO output level = `High`**: the pin starts HIGH, so the LED starts **off** (active-low).
   - **GPIO mode = `Output Push Pull`**: the pin actively drives both 3.3 V and 0 V.
   - **Pull-up/Pull-down = `No pull-up and no pull-down`**: pulls are for inputs.
   - **Maximum output speed = `Low`**: gentle edges, less electrical noise. An LED doesn't need more.
   - **User Label = `LED`**.
3. **Ctrl+S**, **Yes** to generate code.

The label makes CubeMX write two names into `Core/Inc/main.h`:

```c
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOE
```

## 3. Create the BSP folder (once per project)

1. Right-click the project **drone_fw → New → Source Folder**, name it `BSP`, **Finish**.
   A *source folder* is one whose `.c` files CubeIDE compiles. A plain folder would be ignored.
2. Right-click **drone_fw → Properties → C/C++ Build → Settings → Tool Settings → MCU GCC Compiler →
   Include paths**, click the **+** icon, type `../BSP`, OK, **Apply and Close**.
   This lets `#include "bsp_led.h"` find headers in BSP.

If you forget step 1 you get `undefined reference to 'LED_Init'`. If you forget step 2 you get
`bsp_led.h: No such file or directory`.

## 4. `BSP/bsp_led.h`: the menu

Right-click **BSP → New → Header File**, name it `bsp_led.h`:

```c
#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>

void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void LED_Blink(uint32_t period_ms);
void LED_Update(void);

#endif
```

- `#ifndef BSP_LED_H / #define BSP_LED_H / #endif` is an **include guard**. If two files include this
  header, its contents are still only pasted in once (otherwise: "redefinition" errors).
- `#include <stdint.h>` brings in fixed-size number types: `uint32_t` is an unsigned 32-bit
  whole number (0 to about 4.29 billion).
- The six lines are **prototypes**: they announce each function's name, inputs and output so other
  files can call them. `void` = returns nothing / takes nothing.
- Notice what is **not** here: no PE10, no active-low. Callers don't need to know.

## 5. `BSP/bsp_led.c`: the kitchen

Right-click **BSP → New → Source File**, name it `bsp_led.c`:

```c
#include "bsp_led.h"
#include "main.h"

/* The LED on PE10 is active-low: pin LOW = LED on. */
#define LED_ON_LEVEL   GPIO_PIN_RESET
#define LED_OFF_LEVEL  GPIO_PIN_SET

static uint32_t blink_period_ms = 0;   /* 0 = not blinking */
static uint32_t last_toggle_ms  = 0;

void LED_Init(void)
{
    blink_period_ms = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_OFF_LEVEL);
}

void LED_On(void)
{
    blink_period_ms = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_ON_LEVEL);
}

void LED_Off(void)
{
    blink_period_ms = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_OFF_LEVEL);
}

void LED_Toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void LED_Blink(uint32_t period_ms)
{
    blink_period_ms = period_ms;
    last_toggle_ms  = HAL_GetTick();
}

void LED_Update(void)
{
    if (blink_period_ms == 0) {
        return;
    }
    uint32_t now = HAL_GetTick();
    if (now - last_toggle_ms >= blink_period_ms / 2) {
        last_toggle_ms = now;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}
```

Line by line:

- `#include "main.h"`: gives us `LED_Pin`, `LED_GPIO_Port` and the whole HAL library. Only the `.c`
  includes it, so the hardware details stay out of the header.
- `#define LED_ON_LEVEL GPIO_PIN_RESET`: the **one place** that knows the LED is active-low. If a
  future board wires it active-high, you swap these two lines and nothing else.
- `static uint32_t blink_period_ms`: a variable that remembers its value between calls. `static`
  here means **private to this file**: `main.c` can't touch it, only this module's functions can.
- `HAL_GPIO_WritePin(port, pin, level)`: sets the pin to 0 V or 3.3 V.
- `HAL_GPIO_TogglePin(port, pin)`: flips it, whatever it was.
- `LED_On()` / `LED_Off()` also set `blink_period_ms = 0`, so a steady command cancels any blinking.
- `LED_Blink(1000)`: blink with a 1000 ms cycle (500 ms on, 500 ms off). It only **stores** the period
  and the start time. It returns immediately.
- `HAL_GetTick()`: milliseconds since power-up, counted by the SysTick interrupt that `HAL_Init()` started.
- `LED_Update()`: the part that does the blinking. Each time it's called it asks "has half a period
  passed since the last toggle?" If yes, it toggles and remembers the time; if not, it returns at
  once. It **never waits**, which is why it can share the main loop with the ESCs, encoders and
  serial link later. (With `HAL_Delay(500)` instead, the whole drone would freeze for half a second
  on every blink.)
- `now - last_toggle_ms`: subtracting like this still works when the millisecond counter wraps
  back to 0 after about 49 days, because unsigned subtraction wraps the same way.

## 6. Use it from `main.c`

Only three lines, each inside its USER CODE section:

```c
/* USER CODE BEGIN Includes */
#include "bsp_led.h"
/* USER CODE END Includes */
```

```c
  /* USER CODE BEGIN 2 */
  LED_Init();
  LED_Blink(1000);
  /* USER CODE END 2 */
```

```c
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LED_Update();
  }
  /* USER CODE END 3 */
```

`USER CODE BEGIN 2` runs once after the setup functions. `USER CODE BEGIN 3` is inside the
`while (1)` loop, so `LED_Update()` runs over and over, thousands of times per second.

## 7. Build, flash, check

Build (hammer, `0 errors`), flash with `stm32loader` as in chapter 00, then replug.

**Check:** the LED blinks slowly: 0.5 s on, 0.5 s off.

## Try it

1. Change `LED_Blink(1000)` to `LED_Blink(200)`. What do you see?
2. Replace the two lines in `USER CODE BEGIN 2` with just `LED_Init(); LED_On();`. The LED should
   stay on. Why does `LED_Update()` now do nothing?
3. Hard: add a function `LED_SetPattern(uint8_t flashes)` that blinks the LED `flashes` times quickly,
   then pauses for 1 s, using only `HAL_GetTick()` (no `HAL_Delay`). This is how the firmware will
   signal error codes later.
