/*
 * bsp_comm.c
 *
 *  Created on: Nov 8, 2026
 *      Author: amr
 *
 *  Raspberry Pi link on USART3 (message format in bsp_comm.h).
 *  - Receiving: the USART3 interrupt only stores each byte in a buffer.
 *    COMM_Update (main loop) builds lines and checks them, so nothing
 *    slow ever runs inside an interrupt and TIM6 keeps its 500 Hz.
 *  - Sending: COMM_Update sends one $TEL line every TEL_PERIOD_MS.
 *  - Checks before a target is used: checksum, valid number, clamp to
 *    +/- TARGET_LIMIT_DEG. Anything else is thrown away and counted.
 *  - Link loss: once the Pi has sent a valid command, if nothing valid
 *    arrives for LINK_TIMEOUT_MS the target goes back to 0 (level).
 */

#include "main.h"
#include "bsp_comm.h"
#include "bsp_imu.h"
#include "bsp_enc.h"
#include "bsp_pid.h"
#include "bsp_esc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define TEL_PERIOD_MS		20		/* telemetry rate: 50 Hz                    */
#define TARGET_LIMIT_DEG	20.0f	/* largest target the Pi may ask for        */
#define LINK_TIMEOUT_MS		500		/* no valid command this long = link lost   */
#define RX_BUF_SIZE			128		/* bytes waiting between interrupt and loop */
#define LINE_MAX			32		/* longest command line accepted            */

/* filled by the interrupt, emptied by COMM_Update */
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static char line[LINE_MAX];
static uint8_t line_len = 0;
static uint8_t line_too_long = 0;

static uint32_t last_tel_ms = 0;
static uint32_t last_rx_ms = 0;
static uint8_t link_used = 0;		/* 1 after the first valid command */
static uint8_t link_lost = 0;
static uint32_t bad_count = 0;		/* messages thrown away */

/* XOR of every character, sent as 2 hex digits after the '*' */
static uint8_t COMM_Checksum(const char *text, uint32_t len){
	uint8_t sum = 0;
	uint32_t i;

	for(i = 0; i < len; i++){
		sum ^= (uint8_t)text[i];
	}
	return sum;
}

/* checks one "$SET,10.0*CS" line, returns 1 and the target if it is valid */
static uint8_t COMM_ParseSet(char *msg, float *target_deg){
	char *star, *end;
	uint32_t sent_sum;
	float value;

	if(msg[0] != '$'){
		return 0;
	}
	star = strchr(msg, '*');
	if(star == NULL || strlen(star) != 3){
		return 0;
	}
	sent_sum = strtoul(star + 1, &end, 16);
	if(*end != '\0' || sent_sum != COMM_Checksum(msg + 1, (uint32_t)(star - msg - 1))){
		return 0;
	}
	*star = '\0';	/* cut the checksum off, leaves "$SET,10.0" */

	if(strncmp(msg + 1, "SET,", 4) != 0){
		return 0;
	}
	value = strtof(msg + 5, &end);
	if(end == msg + 5 || *end != '\0' || isfinite(value) == 0){
		return 0;
	}

	if(value > TARGET_LIMIT_DEG){
		value = TARGET_LIMIT_DEG;
	}
	if(value < -TARGET_LIMIT_DEG){
		value = -TARGET_LIMIT_DEG;
	}
	*target_deg = value;
	return 1;
}

static void COMM_HandleLine(uint32_t now){
	float target_deg;

	if(COMM_ParseSet(line, &target_deg)){
		PID_SetTarget(target_deg);
		last_rx_ms = now;
		link_used = 1;
		link_lost = 0;
	}
	else{
		bad_count++;
	}
}

static void COMM_SendTelemetry(uint32_t now){
	const IMU_Data_t *imu = IMU_GetData();
	char msg[96];
	int len;

	len = snprintf(msg, sizeof(msg), "TEL,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%lu",
			(unsigned long)now,
			imu->pitch,
			ENC_GetDeg(ENC_PITCH),
			ENC_GetDeg(ENC_YAW),
			PID_GetTarget(),
			PID_GetOutput(),
			(unsigned)PID_GetLeft(),
			(unsigned)PID_GetRight(),
			(unsigned)ESC_IsArmed(),
			(unsigned)link_lost,
			(unsigned long)bad_count);
	if(len <= 0 || len >= (int)sizeof(msg)){
		return;
	}
	printf("$%s*%02X\r\n", msg, COMM_Checksum(msg, (uint32_t)len));
}

void COMM_Init(void){
	/* priority 0: above TIM6 (priority 1). The IMU read inside TIM6 takes about
	   0.4 ms, longer than one byte at 115200 (0.09 ms), so a lower priority
	   lost bytes. This handler only stores one byte (about 1 us), so the
	   500 Hz control loop is not affected */
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
	SET_BIT(USART3->CR1, USART_CR1_RXNEIE);	/* interrupt on every received byte */
	last_tel_ms = HAL_GetTick();
}

/* main loop: read the Pi's commands, check the link, send telemetry */
void COMM_Update(void){
	uint32_t now = HAL_GetTick();
	char c;

	while(rx_tail != rx_head){
		c = (char)rx_buf[rx_tail];
		rx_tail = (uint16_t)((rx_tail + 1U) % RX_BUF_SIZE);

		if(c == '$'){				/* a new message always starts over */
			line_len = 0;
			line_too_long = 0;
		}
		if(c == '\r' || c == '\n'){
			if(line_too_long){
				bad_count++;
			}
			else if(line_len > 0){
				line[line_len] = '\0';
				COMM_HandleLine(now);
			}
			line_len = 0;
			line_too_long = 0;
		}
		else if(line_len < LINE_MAX - 1){
			line[line_len++] = c;
		}
		else{
			line_too_long = 1;
		}
	}

	/* link loss: go back to level and wait for the Pi to come back */
	if(link_used && link_lost == 0 && now - last_rx_ms > LINK_TIMEOUT_MS){
		link_lost = 1;
		PID_SetTarget(0.0f);
	}

	if(now - last_tel_ms >= TEL_PERIOD_MS){
		last_tel_ms = now;
		COMM_SendTelemetry(now);
	}
}

uint8_t COMM_IsLinkLost(void){
	return link_lost;
}

/* USART3 interrupt: only stores the byte. Reading SR then DR also clears
   an overrun. Note: if "USART3 global interrupt" is ever ticked in CubeMX,
   delete this function, because CubeMX then generates its own one */
void USART3_IRQHandler(void){
	uint32_t sr = USART3->SR;
	uint8_t byte;
	uint16_t next;

	if(sr & (USART_SR_RXNE | USART_SR_ORE)){
		byte = (uint8_t)USART3->DR;
		next = (uint16_t)((rx_head + 1U) % RX_BUF_SIZE);
		if(next != rx_tail){		/* buffer full: the byte is dropped */
			rx_buf[rx_head] = byte;
			rx_head = next;
		}
	}
}
