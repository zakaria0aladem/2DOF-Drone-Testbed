/*
 * retarget.c - Sends printf() output to USART3
 *
 * USART3 (PD8 TX / PD9 RX) is connected to the board's USB-serial chip on the
 * USB-C port printed "UART2". Open that port in a serial terminal at 115200.
 *
 * CubeIDE's syscalls.c calls __io_putchar() for every character printf writes.
 */
#include "main.h"

extern UART_HandleTypeDef huart3;

int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
    return ch;
}
