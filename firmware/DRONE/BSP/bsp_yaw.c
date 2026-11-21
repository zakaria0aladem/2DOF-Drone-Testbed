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
#include "bsp_esc.h"


#define YAW_KP_START 3.0f
#define YAW_KD_START 0.5f
#define YAW_KI_START 0.0f
#define YAW_LIMIT 75.0f
#define YAW_ENABLE 1
#define YAW_I_LIMIT 30.0f
#define YAW_DT_S 0.002f

static float target;
static float output;
static float kp = YAW_KP_START;
static float kd = YAW_KD_START;
static float ki = YAW_KI_START;
static float integral = 0.0f;

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
	 integral = 0.0f;
}

void YAW_SetTarget(float target_deg){
	target = target_deg;
}

void YAW_Update(void){
	float yaw_deg = ENC_GetDeg(ENC_YAW);
	float yaw_rate = IMU_GetData()->gz;
	float error = target - yaw_deg;
	integral = YAW_Clamp(integral + ki * error * YAW_DT_S, -YAW_I_LIMIT, YAW_I_LIMIT);
	if (ESC_IsArmed() == 0){
		integral =0.0f;
	}
	float C_out = kp * error + integral - kd * yaw_rate;
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

void YAW_SetGains(float new_kp, float new_ki, float new_kd){
	if(new_kp<0 || new_kp>10){
		return ;
	}
	if(new_ki<0 || new_ki>5){
		return ;
	}
	if(new_kd<0 || new_kd>2){
		return ;
	}
	kp = new_kp;
	kd = new_kd;
	ki = new_ki;
	integral = 0.0f;

}

float YAW_GetKp(void){
	return kp;
}

float YAW_GetKd(void){
	return kd;
}

float YAW_GetKi(void){
	return ki;
}
