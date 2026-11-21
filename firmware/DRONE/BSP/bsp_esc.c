/*
 * bsp_esc.c
 *
 *  Created on: Oct 24, 2026
 *      Author: zakar
 */


#include "main.h"
#include "bsp_esc.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define MIN 1000
#define MAX 2000
#define ARM 6000

static uint32_t arm_start_time_ms = 0;
static uint8_t arm_state = 0;
static uint8_t arm_requested = 0;

void ESC_Arm(void){
	if(arm_state ==0 ){
		arm_requested = 1;
		arm_start_time_ms = HAL_GetTick();
	}
}

/* stops all motors and cancels arming; K1 arms again with the normal countdown.
   Used by the PID after an angle cutoff or an IMU fault */
void ESC_Disarm(void){
	arm_state = 0;
	arm_requested = 0;
	ESC_StopAll();
}

void ESC_Init(void){
	arm_state = 0;
	arm_requested = 0;
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,MIN);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,MIN);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MIN);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MIN);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);


}

void ESC_Set(uint8_t motor, uint16_t pulse_us){
	if(pulse_us>= MAX){
		pulse_us = MAX;
	}
	else if(pulse_us<=MIN){
		pulse_us = MIN;
	}
	if(arm_state==1){
		if(motor == 1){
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,pulse_us);
		}
		else if(motor == 2){
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pulse_us);
		}
		else if(motor==3){
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pulse_us);
		}
		else if(motor==4){
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pulse_us);
		}
		else{
			return;
		}
	}
	else{
		return;
	}

}


uint8_t ESC_IsArmed(void){
	return arm_state;
}


void ESC_Update(void){
	if(arm_requested == 1 && HAL_GetTick()-arm_start_time_ms >=ARM){
		arm_state = 1;
	}
	else{
		return;
	}
}

void ESC_StopAll(void){
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,MIN);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,MIN);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,MIN);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,MIN);
}

/* 1 while K1 was pressed but the countdown is not finished yet */
uint8_t ESC_IsCountingDown(void){
	if(arm_requested == 1 && arm_state == 0){
		return 1;
	}
	return 0;
}

uint16_t ESC_GetPulse(uint8_t motor){
	if(motor == 1){
		return __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_1);
	}
	else if(motor == 2){
		return __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);
	}
	else if(motor == 3){
		return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
	}
	else if(motor == 4){
		return __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
	}
	else{
		return 0;
	}
}


