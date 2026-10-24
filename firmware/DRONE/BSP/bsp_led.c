/*
 * bsp_led.c
 *
 *  Created on: Oct 24, 2026
 *      Author: zakar
 */


#include "bsp_led.h"
#include "main.h"


/* The LED is active low*/
#define LED_ON_LEVEL GPIO_PIN_RESET
#define LED_OFF_LEVEL GPIO_PIN_SET

static uint32_t blink_period_ms = 0;
static uint32_t last_toggle_ms = 0;

void LED_Init(void){
	blink_period_ms =0;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_OFF_LEVEL);
}

void LED_On(void){
	blink_period_ms =0;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_ON_LEVEL);
}

void LED_Off(void){
	blink_period_ms =0;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_OFF_LEVEL);
}

void LED_Toggle(void){
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/*init blinking period and needed variables*/
void LED_Blink(uint32_t period_ms){
	blink_period_ms = period_ms;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, LED_ON_LEVEL);
	last_toggle_ms = HAL_GetTick();
}

/*actual blinking function that checks each cycle*/
void LED_Update(void){
	uint32_t time_ms =HAL_GetTick() - last_toggle_ms;
	if(blink_period_ms==0){
		return;
	}
	else if(time_ms >= blink_period_ms/2){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		last_toggle_ms = HAL_GetTick();

	}
	else{
		return;
	}
}















