/*
 * bsp_buzzer.h
 *
 *  Created on: Nov 7, 2026
 *      Author: zakar
 */

#ifndef BSP_BUZZER_H_
#define BSP_BUZZER_H_


#include <stdint.h>

void BUZZER_Init(void);
void BUZZER_On(void);
void BUZZER_Off(void);
void BUZZER_Beep(uint32_t length_ms);
void BUZZER_Update(void);


#endif /* BSP_BUZZER_H_ */
