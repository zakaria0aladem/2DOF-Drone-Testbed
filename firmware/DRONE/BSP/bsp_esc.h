/*
 * bsp_esc.h
 *
 *  Created on: Oct 24, 2026
 *      Author: zakar
 */

#ifndef BSP_ESC_H_
#define BSP_ESC_H_

#include <stdint.h>

void ESC_Init(void);
void ESC_Set(uint8_t motor, uint16_t pulse_ms);
void ESC_StopAll(void);
uint8_t ESC_IsArmed(void);
void ESC_Update(void);




#endif /* BSP_ESC_H_ */
