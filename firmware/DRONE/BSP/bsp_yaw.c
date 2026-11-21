/*
 * bsp_yaw.c
 *
 *  Created on: Nov 20, 2026
 *      Author: zakar
 */

#include "main.h"
#include "bsp_yaw.h"
#include "bsp_enc.h"
#include "bsp_imu.h"


#define YAW_KP 3.0f
#define YAW_KD 0.5f
#define YAW_LIMIT 75.0f
#define YAW_ENABLE 1

static float target;
static float output;

static float YAW_Clamp(float value, float min, float max){
	if(value > max){
		return max;
	}
	else if(value< min){
		return min;

	}
	else{
		return value;
	}
}

void YAW_Init(void){
	 target = 0.0f;
	 output = 0.0f;

}

void YAW_SetTarget(float target_deg){
	target = target_deg;
}

void YAW_Update(void){
	float yaw_deg = ENC_GetDeg(ENC_YAW);
	float yaw_rate = IMU_GetData()->gz;
	float error = target - yaw_deg;
	float C_out = YAW_KP * error - YAW_KD * yaw_rate;
	output = YAW_Clamp(C_out, -YAW_LIMIT, YAW_LIMIT);
	if (YAW_ENABLE == 0){
		output = 0.0f;
	}
}

float YAW_GetOutput(void){
	return output;
}
float YAW_GetTarget(void){
	return target;
}
