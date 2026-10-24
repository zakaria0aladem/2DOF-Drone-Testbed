/*
 * bsp_led.h
 *
 *  Created on: Oct 24, 2026
 *      Author: zakar
 */

#ifndef BSP_LED_H_
#define BSP_LED_H_

#include <stdint.h>

void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
void LED_Blink(uint32_t period_ms);
void LED_Update(void);




#endif /* BSP_LED_H_ */
