/*
 * bsp_pid.c
 *
 *  Created on: Oct 25, 2026
 *      Author: amr
 */


#include "main.h"
#include "bsp_pid.h"
#include "bsp_imu.h"
#include "bsp_esc.h"
#include <stdio.h>

/* gains: tune with P only first, then add D, then I */
#define KP				2.0f	/* us per deg of error           */
#define KI				0.0f	/* us per deg*s of error         */
#define KD				0.0f	/* us per deg/s of rotation rate */

#define DT_S			0.002f	/* PID_Update runs from TIM6 at 500 Hz       */
#define BASE_US			1150	/* throttle both sides get before correction */
#define OUT_LIMIT		150.0f	/* largest correction [us]                   */
#define I_LIMIT			100.0f	/* largest integral part [us]                */
#define MAX_US			1400	/* ceiling for any motor while tuning        */
#define CUTOFF_DEG		40.0f	/* motors stop beyond this angle             */

/* motors on each side of the pitch axis, numbered as in ESC_Set();
   if the rig tilts away from the target, swap the two sides */
#define MOTOR_FRONT_1	1
#define MOTOR_FRONT_2	2
#define MOTOR_BACK_1	3
#define MOTOR_BACK_2	4

/* PD3 level when the "Motor ON/OFF" switch is ON, check with PID_Print */
#define SWITCH_ON_LEVEL	GPIO_PIN_RESET

#define PRINT_PERIOD_MS	100

static float target = 0.0f;
static float angle = 0.0f;
static float output = 0.0f;
static float integral = 0.0f;
static uint8_t switch_on = 0;
static uint8_t cutoff = 0;
static volatile uint32_t loop_count = 0;
static uint32_t last_count = 0;
static uint32_t last_print_ms = 0;

static float PID_Clamp(float value, float min, float max){
	if(value > max){
		return max;
	}
	if(value < min){
		return min;
	}
	return value;
}

static void PID_StopMotors(void){
	integral = 0.0f;
	output = 0.0f;
	ESC_StopAll();
}

void PID_Init(void){
	target = 0.0f;
	cutoff = 0;
	PID_StopMotors();
}

void PID_SetTarget(float target_deg){
	target = target_deg;
}

/*one control step, called from the TIM6 interrupt every 2 ms*/
void PID_Update(void){
	const IMU_Data_t *imu = IMU_GetData();
	float error, rate, front, back;

	loop_count++;

	/* rig pitch from the IMU: if tilting the rig's pitch axis changes
	   roll instead, use imu->roll and imu->gx here */
	angle = imu->pitch;
	rate  = imu->gy;

	/* switch OFF: motors stopped, and the angle cutoff is reset */
	switch_on = (HAL_GPIO_ReadPin(ARM_SW_GPIO_Port, ARM_SW_Pin) == SWITCH_ON_LEVEL);
	if(switch_on == 0){
		cutoff = 0;
		PID_StopMotors();
		return;
	}

	/* tilted too far: motors stay stopped until the switch is turned off */
	if(angle > CUTOFF_DEG || angle < -CUTOFF_DEG){
		cutoff = 1;
	}
	if(cutoff == 1){
		PID_StopMotors();
		return;
	}

	error = target - angle;
	integral = PID_Clamp(integral + KI * error * DT_S, -I_LIMIT, I_LIMIT);
	output = PID_Clamp(KP * error + integral - KD * rate, -OUT_LIMIT, OUT_LIMIT);

	front = PID_Clamp(BASE_US + output, 1000.0f, MAX_US);
	back  = PID_Clamp(BASE_US - output, 1000.0f, MAX_US);

	ESC_Set(MOTOR_FRONT_1, (uint16_t)front);
	ESC_Set(MOTOR_FRONT_2, (uint16_t)front);
	ESC_Set(MOTOR_BACK_1, (uint16_t)back);
	ESC_Set(MOTOR_BACK_2, (uint16_t)back);
}

/*prints target, angle, output and loop rate, at most once every PRINT_PERIOD_MS*/
void PID_Print(void){
	uint32_t now = HAL_GetTick();
	uint32_t count, loop_hz;

	if(now - last_print_ms < PRINT_PERIOD_MS){
		return;
	}
	count = loop_count;
	loop_hz = (count - last_count) * 1000U / (now - last_print_ms);
	last_count = count;
	last_print_ms = now;

	printf("target %5.1f  angle %6.1f  output %6.1f | switch %s  %s | loop %lu Hz\r\n",
			target, angle, output,
			switch_on ? "ON " : "OFF",
			cutoff ? "CUTOFF" : "ok",
			(unsigned long)loop_hz);
}
