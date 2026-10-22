/*
 * telemetry.h - telemetry/command link for the 2-DOF testbed (STM32 HAL; F103 or F407).
 *
 * The link owns: UART framing, sample timing, the command watchdog, arming and
 * clamping. It does NOT own sensor or ESC drivers. It calls two port hooks
 * that the sensor and ESC code implement (see tlm_port_template.c):
 *
 *     tlm_port_read_sensors()  -> fill joint angles/rates and IMU
 *     tlm_port_write_motors()  -> push four pulse widths to the ESC driver
 *
 * Usage (main.c):
 *     tlm_init(&huart1);                      // after MX_USART1_UART_Init()
 *     while (1) {
 *         tlm_poll();                         // drain RX, handle commands
 *         if (tick_1khz_flag) { tick_1khz_flag = 0; tlm_tick(); }
 *     }
 * and set tick_1khz_flag from a 1 kHz timer interrupt (or SysTick).
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#if defined(STM32F407xx)
#include "stm32f4xx_hal.h"
#else
#include "stm32f1xx_hal.h"
#endif
#include "tlm_proto.h"

/* ---- configuration (override with -D in the build if needed) ---- */
#ifndef TLM_TICK_HZ
#define TLM_TICK_HZ           1000u   /* rate tlm_tick() is called at */
#endif
#ifndef TLM_DEFAULT_RATE_HZ
#define TLM_DEFAULT_RATE_HZ   500u    /* sample + telemetry rate, must divide TLM_TICK_HZ */
#endif
#ifndef TLM_CMD_TIMEOUT_MS
#define TLM_CMD_TIMEOUT_MS    100u    /* armed + no MOTOR_CMD for this long -> idle */
#endif
#ifndef TLM_US_MIN
#define TLM_US_MIN            1000u   /* motor stop / disarmed pulse */
#endif
#ifndef TLM_US_MAX
#define TLM_US_MAX            1400u   /* bench safety clamp, matches BENCH_MAX_US; raise once proven */
#endif
#ifndef TLM_RX_BUF
#define TLM_RX_BUF            256u    /* circular DMA RX buffer, power of two not required */
#endif

/* What the sensor driver hands to the link each sample. */
typedef struct {
    float   q[2];       /* rad */
    float   dq[2];      /* rad/s */
    float   acc[3];     /* m/s^2 */
    float   gyro[3];    /* rad/s */
    uint8_t enc_ok;     /* 1 if the encoder read succeeded */
    uint8_t imu_ok;     /* 1 if the IMU read succeeded */
} tlm_sensors_t;

/* ---- port hooks, implemented outside this module (weak stubs provided) ---- */
void tlm_port_read_sensors(tlm_sensors_t *s);
void tlm_port_write_motors(const uint16_t us[4]);
uint8_t tlm_port_can_arm(void);   /* 0 = refuse ARM (ESCs still arming, fault, killed) */
uint8_t tlm_port_killed(void);    /* 1 = hardware kill latched */

/* ---- API ---- */
void     tlm_init(UART_HandleTypeDef *huart);
void     tlm_poll(void);
void     tlm_tick(void);
void     tlm_text(const char *s);   /* send a debug string if the TX line is free */
uint32_t tlm_micros(void);
uint8_t  tlm_is_armed(void);
void     tlm_disarm(void);          /* e.g. from a hardware kill switch */

#endif /* TELEMETRY_H */
