/*
 * app.c - Application layer for the 2DOF drone rig
 *
 * Current stage: IMU bench test.
 *   App_Init(): finds and configures the on-board IMU (QMI8658 or MPU6050).
 *   App_Loop(): prints acceleration, gyro rate, roll and pitch 10 times a second
 *               on the "UART2" USB-C port (USART3, 115200 baud) and blinks the LED.
 */
#include "app.h"
#include "imu.h"
#include <stdio.h>
#include <math.h>

/* Peripheral handles created by CubeMX (in main.c, or in i2c.c / usart.c) */
extern I2C_HandleTypeDef hi2c2;

/* Board LED: PE10, lit when the pin is LOW */
#define LED_PORT  GPIOE
#define LED_PIN   GPIO_PIN_10

#define RAD_TO_DEG       57.2958f
#define PRINT_PERIOD_MS  100U     /* 10 lines per second */

void App_Init(void)
{
    HAL_Delay(200);                         /* let the IMU power up */
    printf("\r\n=== IMU test ===\r\n");

    if (IMU_Init(&hi2c2) == IMU_NONE) {
        printf("No IMU recognised. Check I2C2 is on PB10/PB11.\r\n");
    } else {
        printf("IMU found: %s\r\n", IMU_Name());
    }
}

void App_Loop(void)
{
    imu_data_t d;

    if (IMU_Read(&d) == HAL_OK) {
        /* Angles from gravity alone (valid while the board is not shaking) */
        float roll  = atan2f(d.ay, d.az) * RAD_TO_DEG;
        float pitch = atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * RAD_TO_DEG;

        printf("acc[g] %6.2f %6.2f %6.2f | gyro[dps] %7.1f %7.1f %7.1f | roll %6.1f  pitch %6.1f\r\n",
               d.ax, d.ay, d.az, d.gx, d.gy, d.gz, roll, pitch);
    } else {
        printf("IMU read error\r\n");
    }

    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);  /* blinking LED = program running */
    HAL_Delay(PRINT_PERIOD_MS);
}
