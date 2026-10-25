/*
 * bsp_enc.c
 *
 *  Created on: Oct 25, 2026
 *      Author: zakar
 */


#include "main.h"
#include "bsp_enc.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

#define ENC_COUNTS_PER_REV 2880

void ENC_Init(void){
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	__HAL_TIM_SET_COUNTER(&htim2, 0);


}

int32_t ENC_GetCount(uint8_t axis){
	if(axis == ENC_YAW){
		return (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

	}
	else if(axis == ENC_PITCH){
		return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
	}
	else{
		return 0;
	}
}


float ENC_GetDeg(uint8_t axis){

	return ENC_GetCount(axis) * 360.0f / ENC_COUNTS_PER_REV;
}


void ENC_Zero(uint8_t axis){
	if(axis == ENC_YAW){
		__HAL_TIM_SET_COUNTER(&htim5,0);

	}
	else if(axis == ENC_PITCH){
		__HAL_TIM_SET_COUNTER(&htim2,0);
	}
	else{
		return;
	}
}
