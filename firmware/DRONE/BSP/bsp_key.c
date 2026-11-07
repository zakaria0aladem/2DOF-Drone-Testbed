/*
 * bsp_key.c
 *
 *  Created on: Nov 7, 2026
 *      Author: zakar
 */
#include "bsp_key.h"
#include <stdint.h>
#include "main.h"

uint8_t KEY1_IsPressed(void){
	if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET){
		return 1;
	}
	else{
		return 0;
	}

}
