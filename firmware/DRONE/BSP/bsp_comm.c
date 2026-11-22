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
 *
 *  BIG PICTURE (how a byte travels):
 *    Pi USB -> board UART2 USB-C -> USB-to-UART bridge -> USART3 RX pin (PD9)
 *    -> RXNE interrupt -> USART3_IRQHandler stores byte in ring buffer (~1 us)
 *    -> COMM_Update (main loop) builds a line -> COMM_HandleLine checks it
 *    -> PID_SetTarget / YAW_SetTarget / ESC_Disarm / SetGains
 *  And back: COMM_SendTelemetry -> printf -> __io_putchar -> USART3 TX (PD8) -> Pi
 *
 *  UART settings: 115200 baud, 8N1 = 10 bits per byte -> ~87 us per byte.
 *  "10 out, 50 back": the Pi sends commands at 10 Hz, the board sends $TEL at 50 Hz.
 */

/* each module only talks to others through their header functions (encapsulation) */
#include "main.h"
#include "bsp_comm.h"
#include "bsp_imu.h"      /* IMU_GetData, IMU_IsFault           -> telemetry, state     */
#include "bsp_enc.h"      /* ENC_GetDeg                         -> encoder angles       */
#include "bsp_pid.h"      /* PID_SetTarget, PID_SetGains, getters -> pitch controller   */
#include "bsp_esc.h"      /* ESC_Disarm, ESC_IsArmed, ESC_GetPulse -> motors            */
#include "bsp_yaw.h"      /* YAW_SetTarget, YAW_SetGains, getters -> yaw controller     */
#include <stdio.h>        /* snprintf, printf                                           */
#include <stdlib.h>       /* strtoul, strtof (text -> number)                           */
#include <string.h>       /* strchr, strlen, strncmp, strcmp                            */
#include <math.h>         /* isfinite (reject NaN / infinity)                           */

/* ---------------------------------------------------------------------------
 * SETTINGS. #define = a name for a number, replaced at compile time (no cost
 * at runtime). One place to change behaviour, and the code reads like words.
 * ------------------------------------------------------------------------- */
#define TEL_PERIOD_MS		20		/* telemetry rate: 50 Hz. Fast enough for plots, ~40% of the UART */
#define TARGET_LIMIT_DEG	35.0f	/* largest pitch target the Pi may ask for: CLAMP, not reject.
									   Stays well inside the +/-60 deg cutoff. f = float for the FPU */
#define YAW_LIMIT_DEG		3600.0f	/* largest yaw target: 10 turns, no cable limit since the rig runs on battery */
#define LINK_TIMEOUT_MS		500		/* no VALID command this long = link lost.
									   Pi sends every 100 ms -> 500 ms = 5 missed heartbeats */
#define RX_BUF_SIZE			128		/* ring buffer size: bytes waiting between interrupt and main loop.
									   Pi sends only ~a few hundred bytes/s, so plenty of margin */
#define LINE_MAX			48		/* longest command line accepted (was 32, gain lines are longer).
									   Longer lines are thrown away -> no buffer overflow */
#define GAIN_PERIOD_MS		1000	/* $GAIN line (gains in use): once a second  */

/* ---------------------------------------------------------------------------
 * VARIABLES
 * static (outside a function) = PRIVATE to this file + keeps its value forever.
 * Other modules can only use the public functions (COMM_Init/Update/IsLinkLost).
 * volatile = changed inside an interrupt -> compiler must re-read it from
 * memory every time (otherwise it could cache it in a register and never see
 * the ISR's change). volatile is NOT a lock.
 * ------------------------------------------------------------------------- */

/* GROUP 1: ring buffer, SHARED between the USART3 ISR and the main loop.
   filled by the interrupt, emptied by COMM_Update.
   head = where the ISR writes next   (ONLY the ISR moves head)
   tail = where the main loop reads next (ONLY the main loop moves tail)
   head == tail -> empty.  head+1 == tail -> full.
   One writer per index (single producer, single consumer) -> no lock needed.
   uint16_t is read/written in one instruction on Cortex-M -> never half-updated */
static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

/* GROUP 2: the line being built from the bytes (main loop only, no volatile) */
static char line[LINE_MAX];			/* e.g. "$SET,10.0*71" while it is assembled           */
static uint8_t line_len = 0;		/* characters stored so far                             */
static uint8_t line_too_long = 0;	/* 1 = overflowed: keep reading to its end, then reject */

/* GROUP 3: timing, link state and statistics (main loop only) */
static uint32_t last_tel_ms = 0;	/* when the last $TEL was sent   -> 50 Hz timer         */
static uint32_t last_gain_ms = 0;	/* when the last $GAIN was sent  -> 1 Hz timer          */
static uint8_t gain_due = 0;		/* 1 = send $GAIN soon (after a gain change) = confirmation */
static uint32_t last_rx_ms = 0;		/* when the last VALID command arrived -> link-loss timer */
static uint8_t link_used = 0;		/* 1 after the first valid command.
									   Before that no link-loss (board can run without a Pi) */
static uint8_t link_lost = 0;		/* 1 = no valid command for 500 ms ($TEL field 10)      */
static uint32_t bad_count = 0;		/* messages thrown away ($TEL field 11). This counter is
									   what showed the old interrupt-priority bug (~half bad) */

/* ===========================================================================
 * COMM_Checksum: XOR of every character, sent as 2 hex digits after the '*'
 *
 * static (function) = private helper, only this file can call it.
 * const char *text  = POINTER to the first character to include (its address).
 *                     const = the function may only READ the text, not change it.
 * len               = how many characters. Needed because only PART of the line
 *                     is checksummed: the characters BETWEEN '$' and '*'.
 * Returns one byte (0..255) = exactly 2 hex digits (00..FF).
 *
 * XOR (^) bit by bit: same -> 0, different -> 1.
 * Catches: any single flipped bit.
 * Misses : two flips in the same bit position, swapped characters (order doesn't
 *          matter for XOR). A CRC would catch those. Same function in protocol.py.
 * Example: "SET,10.0" -> 0x71 -> "$SET,10.0*71"
 * ========================================================================= */
static uint8_t COMM_Checksum(const char *text, uint32_t len){
	uint8_t sum = 0;		/* start at 0: x ^ 0 = x, so 0 changes nothing */
	uint32_t i;

	for(i = 0; i < len; i++){
		sum ^= (uint8_t)text[i];	/* sum = sum XOR this character.
									   (uint8_t) cast: char may be signed, cast keeps it a plain byte */
	}
	return sum;
}

/* ===========================================================================
 * COMM_CheckLine: INTEGRITY check, the same for every message type.
 * checks that a line is a valid message: starts with '$', ends with '*'
 * and 2 hex digits, and the checksum matches. On success the checksum is
 * cut off ("$SET,10.0*3A" becomes "$SET,10.0") and 1 is returned
 *
 * char *msg: pointer to the line. NOT const, because it modifies the line
 * (writes '\0' over the '*').
 * Integrity first, meaning later: no point reading data we know is damaged.
 *
 * Example line in memory:
 *   index: 0   1   2   3   4   5   6   7   8   9   10  11  12
 *          '$' 'S' 'E' 'T' ',' '1' '0' '.' '0' '*' '7' '1' '\0'
 * ========================================================================= */
static uint8_t COMM_CheckLine(char *msg){
	char *star, *end;
	uint32_t sent_sum;

	/* 1. every message starts with '$' */
	if(msg[0] != '$'){
		return 0;
	}
	/* 2. strchr finds the first '*' and returns a POINTER to it (index 9),
	      or NULL if there is none. strlen(star) counts '*','7','1' = 3,
	      so exactly 2 checksum characters, no more, no less */
	star = strchr(msg, '*');
	if(star == NULL || strlen(star) != 3){
		return 0;
	}
	/* 3. strtoul = "string to unsigned long", base 16 (hex): "71" -> 0x71.
	      end is set to the first character it could NOT convert.
	      *end != '\0' -> checksum was not valid hex (e.g. "7Z") -> reject.
	      Then recalculate the XOR over msg+1 (skip '$') for (star - msg - 1)
	      characters (pointer subtraction = distance = 8 here) and compare.
	      Sent != calculated -> corrupted on the way -> reject */
	sent_sum = strtoul(star + 1, &end, 16);
	if(*end != '\0' || sent_sum != COMM_Checksum(msg + 1, (uint32_t)(star - msg - 1))){
		return 0;
	}
	/* 4. overwrite '*' with the end-of-string marker: the string is now
	      "$SET,10.0". The checksum is no longer needed, and the number check
	      can simply demand that the number runs to the end of the string */
	*star = '\0';
	return 1;
}

/* ===========================================================================
 * COMM_ParseNumber: ONE number + clamp. Reused for pitch and yaw (the limit
 * is a parameter).
 * reads the number in text ("10.0"), clamps it to +/- limit.
 * Returns 1 and the value if it is a real number, 0 otherwise
 *
 * float *value = OUTPUT parameter: a C function returns only one value (here
 * 1/0 for "valid?"), so the caller passes the ADDRESS of its own float and the
 * result is written there with *value = v.
 * ========================================================================= */
static uint8_t COMM_ParseNumber(const char *text, float limit, float *value){
	char *end;
	float v = strtof(text, &end);	/* strtof = "string to float", end = first unused character */

	/* three ways to fail:
	   end == text      -> nothing converted, no number at all   ("$SET,abc")
	   *end != '\0'     -> junk after the number                 ("$SET,10xyz")
	   !isfinite(v)     -> NaN or infinity ("nan"/"inf" are accepted by strtof).
	                       A NaN target would make the PID output NaN and slip past
	                       every comparison (any compare with NaN is false) */
	if(end == text || *end != '\0' || isfinite(v) == 0){
		return 0;
	}
	/* CLAMP, don't reject: a valid message that asks for too much is limited */
	if(v > limit){
		v = limit;
	}
	if(v < -limit){
		v = -limit;
	}
	*value = v;		/* write the result into the caller's variable through the pointer */
	return 1;
}

/* ===========================================================================
 * COMM_ParseNumbers: EXACTLY "count" comma-separated numbers (used for gains).
 * reads "count" numbers separated by commas ("6,2.5,0.85") into values[].
 * Returns 1 only if there are exactly that many real numbers
 * ("6,2.5" = too few -> 0, "6,2.5,0.85,9" = too many -> 0).
 * No clamping here: range checks are done in PID_SetGains / YAW_SetGains.
 * ========================================================================= */
static uint8_t COMM_ParseNumbers(const char *text, float *values, uint8_t count){
	char *end;
	uint8_t i;

	for(i = 0; i < count; i++){
		values[i] = strtof(text, &end);		/* read one number */
		if(end == text || isfinite(values[i]) == 0){
			return 0;						/* no number, or NaN/infinity */
		}
		if(i < count - 1){
			/* not the last number: a ',' MUST follow, continue right after it */
			if(*end != ','){
				return 0;
			}
			text = end + 1;
		}
		else if(*end != '\0'){
			return 0;						/* last number: must be the end of the string */
		}
	}
	return 1;
}

/* ===========================================================================
 * COMM_HandleLine: COMMAND DISPATCHER. Called by COMM_Update each time a
 * '\r' or '\n' completes a line. now = time in ms, read once per update.
 *
 * one complete line from the Pi. Accepted commands:
 *   $SET,<deg>   pitch target (clamped to TARGET_LIMIT_DEG)
 *   $YAW,<deg>   yaw target   (clamped to YAW_LIMIT_DEG)
 *   $DIS         disarm the motors (arming stays on the K1 key)
 *   $GP,kp,ki,kd pitch gains (range checks in PID_SetGains)
 *   $GY,kp,ki,kd yaw gains   (range checks in YAW_SetGains)
 * Any valid command also counts as the heartbeat for link-loss
 *
 * Valid   -> act on it + reset the link-loss timer.
 * Invalid -> bad_count++ and NOTHING else (old target kept, timer NOT reset,
 *            so a link that only delivers garbage still counts as lost).
 * ========================================================================= */
static void COMM_HandleLine(uint32_t now){
	float value;
	float gains[3];

	/* integrity first, for every message type (checksum cut off on success) */
	if(COMM_CheckLine(line) == 0){
		bad_count++;
		return;
	}
	/* strncmp(a, b, n) compares the first n characters, 0 = equal.
	   The '$' is still in the line, so "$SET," is 5 characters.
	   The value starts right after the prefix (line + 5). */
	if(strncmp(line, "$SET,", 5) == 0 && COMM_ParseNumber(line + 5, TARGET_LIMIT_DEG, &value)){
		PID_SetTarget(value);		/* the ONLY way a Pi command reaches the pitch PID.
									   Used on the next TIM6 tick (within 2 ms) */
	}
	else if(strncmp(line, "$YAW,", 5) == 0 && COMM_ParseNumber(line + 5, YAW_LIMIT_DEG, &value)){
		YAW_SetTarget(value);		/* yaw target, same idea, own (larger) limit */
	}
	else if(strcmp(line, "$DIS") == 0){
		/* strcmp = whole string must match exactly: "$DIS" and nothing else.
		   REMOTE DISARM: stopping is allowed from anywhere (Pi / IoT dashboard),
		   but ARMING stays on the physical K1 key, so nobody can spin the props
		   remotely while someone stands next to the rig */
		ESC_Disarm();
	}
	else if(strncmp(line, "$GP,", 4) == 0 && COMM_ParseNumbers(line + 4, gains, 3)){
		/* live pitch gain tuning, no reflashing needed */
		PID_SetGains(gains[0], gains[1], gains[2]);
		gain_due = 1;		/* tell the Pi straight away what is in use now */
	}
	else if(strncmp(line, "$GY,", 4) == 0 && COMM_ParseNumbers(line + 4, gains, 3)){
		/* live yaw gain tuning */
		YAW_SetGains(gains[0], gains[1], gains[2]);
		gain_due = 1;		/* confirmation via $GAIN */
	}
	else{
		/* checksum was fine but unknown command or bad numbers -> reject */
		bad_count++;
		return;
	}
	/* reached only by a VALID command = HEARTBEAT */
	last_rx_ms = now;		/* reset the link-loss timer: "the Pi is alive"           */
	link_used = 1;			/* from now on, watch for link loss                       */
	link_lost = 0;			/* RECOVERY: clears a lost link, normal control resumes   */
}

/* ===========================================================================
 * COMM_GetState: one number for the system state (last $TEL field).
 * Checked in PRIORITY order: a fault is reported first, even if armed.
 *   3 = IMU fault   2 = armed   1 = counting down (K1 pressed)   0 = disarmed
 * (an angle cutoff disarms the motors, so it shows up as 0 here)
 * ========================================================================= */
static uint8_t COMM_GetState(void){
	if(IMU_IsFault()){
		return 3;
	}
	else if(ESC_IsArmed()){
		return 2;
	}
	else if(ESC_IsCountingDown()){
		return 1;
	}
	else{
		return 0;
	}
}

/* ===========================================================================
 * COMM_SendTelemetry: builds and sends one $TEL line. Called every 20 ms
 * (50 Hz) from COMM_Update, in the MAIN LOOP, never in an ISR.
 *
 * $TEL fields (18):
 *   1 ms          board time since power-up
 *   2 pitch       IMU pitch after the complementary filter (deg)
 *   3 enc_pitch   pitch encoder angle (deg)
 *   4 enc_yaw     yaw encoder angle (deg)
 *   5 target      pitch target the BOARD is really using (confirms clamp/link loss)
 *   6 output      pitch PID output u (us)
 *   7 left_us     left motor pair pulse (us)
 *   8 right_us    right motor pair pulse (us)
 *   9 armed       1 = motors armed
 *  10 link_lost   1 = no valid command for 500 ms
 *  11 bad         rejected messages since start
 *  12 yaw_target  yaw target in use (deg)
 *  13 yaw_output  yaw controller output
 *  14-17 m1..m4   each motor's final pulse (us). Needed now because yaw mixing
 *                 makes every motor different, left/right is no longer enough
 *  18 state       0 disarmed, 1 counting down, 2 armed, 3 IMU fault
 * Groups: time | sensors | controllers | motors | status
 *         (= measured / reference / status, as the IoT milestone asks)
 * ========================================================================= */
static void COMM_SendTelemetry(uint32_t now){
	/* pointer to the IMU module's data, no copy. const = read-only.
	   imu->pitch = the "pitch" field of the struct the pointer points to */
	const IMU_Data_t *imu = IMU_GetData();
	char msg[160];		/* text buffer for the body, bigger than any real $TEL */
	int len;

	/* snprintf = printf into a STRING. sizeof(msg) = the most it may write,
	   so it can NEVER overflow the buffer (plain sprintf has no limit).
	   %lu = unsigned long, %.1f = float with 1 decimal, %u = unsigned int.
	   Types must match exactly -> casts. (unsigned) on a float pulse cuts the
	   decimals (1199.7 -> 1199), us precision is enough.
	   Every value comes from ITS OWN module through a getter: no shared globals.
	   No '$' here: the checksum must not include it, it is added at the end.
	   Note: TIM6 can interrupt this function, so values may be from ticks
	   2 ms apart. Fine for monitoring/logging. */
	len = snprintf(msg, sizeof(msg), "TEL,%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%lu,%.1f,%.1f,%u,%u,%u,%u,%u",
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
			(unsigned long)bad_count,
			YAW_GetTarget(),
			YAW_GetOutput(),
			(unsigned)ESC_GetPulse(1),
			(unsigned)ESC_GetPulse(2),
			(unsigned)ESC_GetPulse(3),
			(unsigned)ESC_GetPulse(4),
			(unsigned)COMM_GetState());

	/* len <= 0 -> formatting error. len >= size -> text didn't fit and was cut
	   (snprintf returns the length it WANTED). Better to skip one message than
	   send a broken one */
	if(len <= 0 || len >= (int)sizeof(msg)){
		return;
	}
	/* final line: '$' + body + '*' + checksum as 2 uppercase hex digits (%02X,
	   leading zero) + "\r\n". printf -> __io_putchar -> HAL_UART_Transmit:
	   BLOCKING, ~90 bytes x 87 us = ~8 ms. OK because this is the main loop;
	   the 500 Hz TIM6 interrupt keeps running underneath.
	   (Float printing needs "Use float with printf" = -u _printf_float) */
	printf("$%s*%02X\r\n", msg, COMM_Checksum(msg, (uint32_t)len));
}

/* ===========================================================================
 * COMM_SendGains: one $GAIN line with the gains currently in use.
 * $GAIN,pitch_kp,pitch_ki,pitch_kd,yaw_kp,yaw_ki,yaw_kd: the gains in use
 * Sent once a second, and straight after a $GP/$GY change (confirmation).
 * %.3f = 3 decimals (gains like 0.850 need more precision than angles).
 * Same build pattern as $TEL: snprintf body, check length, add $ * CS \r\n.
 * ========================================================================= */
static void COMM_SendGains(void){
	char msg[96];
	int len;

	len = snprintf(msg, sizeof(msg), "GAIN,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
			PID_GetKp(), PID_GetKi(), PID_GetKd(),
			YAW_GetKp(), YAW_GetKi(), YAW_GetKd());
	if(len <= 0 || len >= (int)sizeof(msg)){
		return;
	}
	printf("$%s*%02X\r\n", msg, COMM_Checksum(msg, (uint32_t)len));
}

/* ===========================================================================
 * COMM_Init: switch on reception by interrupt. Called ONCE in main(), after
 * MX_USART3_UART_Init() already set 115200 8N1 and enabled TX/RX.
 *
 * An interrupt needs TWO switches ON:
 *   1. the PERIPHERAL asks   (USART3 CR1 RXNEIE bit)
 *   2. the NVIC (CPU) listens (HAL_NVIC_EnableIRQ, with a priority)
 * Done here (not in CubeMX) with our own minimal handler: HAL_UART_Receive_IT
 * is built for a KNOWN number of bytes, our data is an endless stream of lines
 * with unknown length, so "interrupt on every byte -> ring buffer" is simplest.
 * ========================================================================= */
void COMM_Init(void){
	/* priority 0: above TIM6 (priority 1). The IMU read inside TIM6 takes about
	   0.4 ms, longer than one byte at 115200 (0.09 ms), so a lower priority
	   lost bytes. This handler only stores one byte (about 1 us), so the
	   500 Hz control loop is not affected */
	/* (preemption priority 0 = highest, can interrupt TIM6 while it runs.
	    sub-priority 0 = only decides order between equal preemption levels.
	    Priorities: SysTick 0, USART3 0, TIM6 1. Lower number = higher priority.
	    It's about DEADLINES, not importance: UART must be served within 87 us) */
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);			/* switch 2: the CPU accepts USART3 interrupts */
	SET_BIT(USART3->CR1, USART_CR1_RXNEIE);	/* switch 1: interrupt on every received byte.
											   RXNE = "receive data register not empty",
											   RXNEIE = RXNE Interrupt Enable */
	last_tel_ms = HAL_GetTick();			/* first $TEL 20 ms after start-up */
}

/* ===========================================================================
 * COMM_Update: the MAIN-LOOP half of the module, called every pass of while(1).
 * main loop: read the Pi's commands, check the link, send telemetry
 *   Part 1: empty the ring buffer and assemble complete lines
 *   Part 2: link-loss check
 *   Part 3: send $TEL every 20 ms (or $GAIN when due)
 * Everything slow (parsing, checksum, snprintf, blocking printf) is HERE,
 * never in the ISR, so the 500 Hz control loop is never disturbed.
 * ========================================================================= */
void COMM_Update(void){
	uint32_t now = HAL_GetTick();	/* read the clock ONCE: same 'now' for every decision below */
	char c;

	/* ---- Part 1: empty the ring buffer, build lines -------------------- */
	/* while tail != head = there are unread bytes. Empties EVERYTHING that is
	   waiting, so even a burst of several messages is handled in one call */
	while(rx_tail != rx_head){
		c = (char)rx_buf[rx_tail];
		/* move tail forward. % RX_BUF_SIZE wraps 127 -> 0 = circular buffer.
		   1U = unsigned 1, keeps the arithmetic unsigned */
		rx_tail = (uint16_t)((rx_tail + 1U) % RX_BUF_SIZE);

		/* '$' = start of a new message -> start over. Leftovers of a broken line
		   (lost bytes, no '\n') are dropped: automatic RESYNC on the next '$'.
		   (the '$' itself is then stored as line[0] by the else-if below) */
		if(c == '$'){				/* a new message always starts over */
			line_len = 0;
			line_too_long = 0;
		}
		/* end of line -> process it */
		if(c == '\r' || c == '\n'){
			if(line_too_long){
				bad_count++;		/* overflowed line -> reject */
			}
			else if(line_len > 0){
				/* '\0' turns the characters into a proper C string, then check + use it.
				   line_len > 0: the Pi sends "\r\n", the '\r' processes the line and
				   the '\n' finds an empty line -> silently ignored, not counted bad */
				line[line_len] = '\0';
				COMM_HandleLine(now);
			}
			line_len = 0;			/* ready for the next line */
			line_too_long = 0;
		}
		/* normal character -> append. line_len++ stores at the current position,
		   then increments. LINE_MAX - 1: one slot always kept free for the '\0' */
		else if(line_len < LINE_MAX - 1){
			line[line_len++] = c;
		}
		/* line full but not ended -> stop storing, reject it when it ends.
		   = OVERFLOW PROTECTION (never write past the end of line[]) */
		else{
			line_too_long = 1;
		}
	}

	/* ---- Part 2: link loss --------------------------------------------- */
	/* link loss: go back to level and wait for the Pi to come back */
	/* all three must be true:
	   link_used      -> a valid command arrived at least once
	   link_lost == 0 -> not already lost, so this runs ONCE at the moment of loss
	   > 500 ms       -> no VALID command for 5 heartbeats
	   -> target 0 deg (LEVEL = known safe position). Motors stay armed, PID keeps
	   running (cutting motors would drop the rig). Recovery in COMM_HandleLine.
	   now - last_rx_ms stays correct even when the ms counter wraps (unsigned).
	   Note: only the PITCH target is reset here, the yaw target is kept. */
	if(link_used && link_lost == 0 && now - last_rx_ms > LINK_TIMEOUT_MS){
		link_lost = 1;
		PID_SetTarget(0.0f);
	}

	/* ---- Part 3: telemetry (50 Hz) and gains (1 Hz / after a change) ---- */
	/* NON-BLOCKING timer pattern: if the period has passed, update the timestamp
	   and act; otherwise do nothing and let the main loop continue. No HAL_Delay */
	if(now - last_tel_ms >= TEL_PERIOD_MS){
		last_tel_ms = now;
		COMM_SendTelemetry(now);
	}
	/* never in the same pass as $TEL, so one call never prints both lines */
	/* (else-if keeps each COMM_Update's blocking printf short) */
	else if(gain_due || now - last_gain_ms >= GAIN_PERIOD_MS){
		last_gain_ms = now;
		gain_due = 0;
		COMM_SendGains();
	}
}

/* public getter: lets other modules ask "is the link lost?" without touching
   the private variable (encapsulation) */
uint8_t COMM_IsLinkLost(void){
	return link_lost;
}

/* ===========================================================================
 * USART3_IRQHandler: runs on EVERY received byte (priority 0). Keep it TINY.
 * USART3 interrupt: only stores the byte. Reading SR then DR also clears
 * an overrun. Note: if "USART3 global interrupt" is ever ticked in CubeMX,
 * delete this function, because CubeMX then generates its own one
 *
 * Overrun (ORE) = a new byte finished before the old one was read (the UART
 * holds only ONE byte). That was the old bug when TIM6 had higher priority.
 * Takes ~1 us, so it barely delays the TIM6 control loop.
 * ========================================================================= */
void USART3_IRQHandler(void){
	uint32_t sr = USART3->SR;		/* status register: which event happened */
	uint8_t byte;
	uint16_t next;

	/* RXNE = a byte is waiting. ORE = overrun. Either way, read DR:
	   reading DR takes the byte AND (after SR) clears RXNE / ORE,
	   so reception can never get stuck */
	if(sr & (USART_SR_RXNE | USART_SR_ORE)){
		byte = (uint8_t)USART3->DR;	/* data register: the received byte */
		next = (uint16_t)((rx_head + 1U) % RX_BUF_SIZE);	/* where head would move (wraps) */
		/* if head would catch up with tail the buffer is full -> drop the byte.
		   The broken line then fails its checksum and is counted as bad */
		if(next != rx_tail){		/* buffer full: the byte is dropped */
			rx_buf[rx_head] = byte;	/* store at head ...                     */
			rx_head = next;			/* ... then move head (only the ISR does) */
		}
	}
}
