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

/* PD controller for the props-off demo: KI stays 0.
   KP 10: a 20 deg tilt already gives the full correction, so the
   difference between the two sides is easy to see and hear */
#define KP				10.0f	/* us per deg of error           */
#define KI				0.0f	/* us per deg*s of error (0 = PD) */
#define KD				0.5f	/* us per deg/s of rotation rate */

#define DT_S			0.002f	/* PID_Update runs from TIM6 at 500 Hz       */
#define BASE_US			1200	/* throttle both sides get before correction */
#define OUT_LIMIT		200.0f	/* largest correction [us]                   */
#define I_LIMIT			100.0f	/* largest integral part [us]                */
#define MAX_US			1400	/* ceiling for any motor while tuning        */
#define MIN_US			1000	/* slow side stops fully at about 20 deg tilt (1000 = stop) */
#define CUTOFF_DEG		60.0f	/* motors stop beyond this angle             */

/* motor test: 1 = spin the motors one at a time in order 1, 2, 3, 4
   (2 s each, 1 s pause, repeating) instead of running the controller.
   Use it to see which physical motor has which number, then set it to 0 */
#define MOTOR_ID_TEST	0
#define ID_TEST_US		1250	/* speed of the motor under test */

/* motors on each side of the pitch shaft, numbered as in ESC_Set().
   Found with MOTOR_ID_TEST, looking along the pitch shaft at the board:
     motor 2 top-left     motor 1 top-right
     motor 3 bottom-left  motor 4 bottom-right
   LEFT gets base + output, RIGHT gets base - output.
   If tilting makes the side that RISES speed up, swap LEFT and RIGHT */
#define MOTOR_LEFT_1	2
#define MOTOR_LEFT_2	3
#define MOTOR_RIGHT_1	1
#define MOTOR_RIGHT_2	4

/* per-motor trim [us], added to every speed the controller sends.
   The ESC channels and motors don't all stop at the same signal: raise a
   motor's trim if it stops while its partner still turns (index = motor) */
static const int16_t motor_trim_us[5] = {
	0,		/* unused */
	0,		/* motor 1 top-right    */
	0,		/* motor 2 top-left     */
	20,		/* motor 3 bottom-left: stalls earlier than motor 2 (demo trim) */
	0,		/* motor 4 bottom-right */
};

/* PD3 level when the "Motor ON/OFF" switch is ON, check with PID_Print.
   PD3 is not actually wired to the switch, so PID_Init gives it a pull-up:
   it then always reads ON instead of floating and flipping at random */
#define SWITCH_ON_LEVEL	GPIO_PIN_SET

#define PRINT_PERIOD_MS	100

static float target = 0.0f;
static float angle = 0.0f;
static float output = 0.0f;
static float left_us = 0.0f;
static float right_us = 0.0f;
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
	left_us = 0.0f;
	right_us = 0.0f;
	ESC_StopAll();
}

#if MOTOR_ID_TEST
/* spins one motor at a time: motor 1 for 2 s, pause 1 s, motor 2, ... */
static void PID_MotorIdTest(void){
	static uint32_t start_ms = 0;
	uint32_t t;
	uint8_t motor, m;

	if(ESC_IsArmed() == 0){
		return;
	}
	if(start_ms == 0){
		start_ms = HAL_GetTick();
	}
	t = (HAL_GetTick() - start_ms) % 12000U;	/* 4 motors x 3 s */
	motor = (uint8_t)(t / 3000U + 1U);

	for(m = 1; m <= 4; m++){
		if(m == motor && (t % 3000U) < 2000U){
			ESC_Set(m, ID_TEST_US);
		}
		else{
			ESC_Set(m, 1000);
		}
	}
}
#endif

/* sends one motor its speed plus its trim, kept inside MIN_US..MAX_US */
static void PID_SetMotor(uint8_t motor, float pulse_us){
	pulse_us = PID_Clamp(pulse_us + motor_trim_us[motor], MIN_US, MAX_US);
	ESC_Set(motor, (uint16_t)pulse_us);
}

void PID_Init(void){
	GPIO_InitTypeDef gpio = {0};

	/* PD3 floats (nothing connected): pull it up so it reads a steady level */
	gpio.Pin = ARM_SW_Pin;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ARM_SW_GPIO_Port, &gpio);

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
	float error, rate;

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

#if MOTOR_ID_TEST
	PID_MotorIdTest();
	return;
#endif

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

	left_us = PID_Clamp(BASE_US + output, MIN_US, MAX_US);
	right_us  = PID_Clamp(BASE_US - output, MIN_US, MAX_US);

	PID_SetMotor(MOTOR_LEFT_1, left_us);
	PID_SetMotor(MOTOR_LEFT_2, left_us);
	PID_SetMotor(MOTOR_RIGHT_1, right_us);
	PID_SetMotor(MOTOR_RIGHT_2, right_us);
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

	printf("target %5.1f  angle %6.1f  output %6.1f | left %4.0f  right %4.0f us | switch %s  %s | loop %lu Hz\r\n",
			target, angle, output, left_us, right_us,
			switch_on ? "ON " : "OFF",
			cutoff ? "CUTOFF" : "ok",
			(unsigned long)loop_hz);
}
