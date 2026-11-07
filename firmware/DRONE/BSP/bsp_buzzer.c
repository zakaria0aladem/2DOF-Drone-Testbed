/*
 * bsp_buzzer.c
 *
 *  Created on: Nov 7, 2026
 *      Author: zakar
 */


#include "main.h"
#include "bsp_buzzer.h"

extern TIM_HandleTypeDef htim1;

/* passive buzzer on PA8 = TIM1 CH1, 2 kHz (period 500 us) */
#define BUZZER_TONE   250   /* half of 500 = loudest tone */
#define BUZZER_SILENT 0

static uint32_t beep_start_ms = 0;
static uint32_t beep_length_ms = 0;   /* 0 = no beep running */

void BUZZER_Init(void){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BUZZER_SILENT);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	beep_length_ms = 0;
}

void BUZZER_On(void){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BUZZER_TONE);
}

void BUZZER_Off(void){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BUZZER_SILENT);
	beep_length_ms = 0;
}

/* starts a beep, BUZZER_Update ends it */
void BUZZER_Beep(uint32_t length_ms){
	BUZZER_On();
	beep_start_ms = HAL_GetTick();
	beep_length_ms = length_ms;
}

/* call every loop */
void BUZZER_Update(void){
	if(beep_length_ms != 0 && HAL_GetTick() - beep_start_ms >= beep_length_ms){
		BUZZER_Off();
	}
}
