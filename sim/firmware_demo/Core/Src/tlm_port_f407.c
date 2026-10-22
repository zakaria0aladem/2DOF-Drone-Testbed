/*
 * tlm_port_f407.c - telemetry port hooks for the RRC v1.2: joint encoders,
 * QMI8658, four ESC outputs, ESC arming delay and the KEY1 kill switch.
 */
#include "telemetry.h"
#include "qmi8658.h"

extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1, htim2, htim3, htim5, htim8;
void dbg(const char *s);

#define ENC_COUNTS_PER_REV  2880.0f          /* E8T-720: 720 CPR x4 */
#define TWO_PI              6.2831853f
#define ESC_ARM_MS          3000u            /* ESCs need ~3 s of 1000 us at power-up */
#define DQ_ALPHA            0.2f             /* low-pass on the joint-rate estimate */

static uint8_t  s_imu_ok;
static uint8_t  s_killed;
static uint32_t s_boot_ms;
static float    s_q_prev[2], s_dq[2];
static uint32_t s_t_prev_us;

static void esc_write(const uint16_t us[4])
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, us[0]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, us[1]);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, us[2]);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, us[3]);
}

void tlm_port_init(void)
{
    static const uint16_t idle[4] = { TLM_US_MIN, TLM_US_MIN, TLM_US_MIN, TLM_US_MIN };
    esc_write(idle);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    s_boot_ms = HAL_GetTick();

    s_imu_ok = (qmi8658_init(&hi2c2) == HAL_OK);
    if (s_imu_ok) {
        dbg("QMI8658 found, calibrating gyro (keep the rig still)\r\n");
        s_imu_ok = (qmi8658_calibrate_gyro(500) == HAL_OK);
    } else {
        dbg("QMI8658 NOT found on I2C2\r\n");
    }
}

/* Called from the main loop: kill switch and status LED. */
void tlm_port_service(void)
{
    if (tlm_is_armed() && HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1) == GPIO_PIN_RESET) {
        s_killed = 1;
        tlm_disarm();
        dbg("KEY1 kill\r\n");
    }
    /* LED: blink fast while ESCs arm, slow when ready, solid while armed. */
    uint32_t t = HAL_GetTick();
    GPIO_PinState on;
    if (tlm_is_armed())                  on = GPIO_PIN_RESET;
    else if (t - s_boot_ms < ESC_ARM_MS) on = ((t / 100u) & 1u) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    else                                 on = ((t / 500u) & 1u) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, on);
}

void tlm_port_read_sensors(tlm_sensors_t *s)
{
    int32_t c0 = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    int32_t c1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint32_t now = tlm_micros();
    float q[2] = { c0 * (TWO_PI / ENC_COUNTS_PER_REV), c1 * (TWO_PI / ENC_COUNTS_PER_REV) };
    float dt = (now - s_t_prev_us) * 1e-6f;
    if (s_t_prev_us != 0 && dt > 0.0f) {
        for (int i = 0; i < 2; i++)
            s_dq[i] += DQ_ALPHA * ((q[i] - s_q_prev[i]) / dt - s_dq[i]);
    }
    s_t_prev_us = now;
    for (int i = 0; i < 2; i++) {
        s_q_prev[i] = q[i];
        s->q[i] = q[i];
        s->dq[i] = s_dq[i];
    }
    s->enc_ok = 1;

    qmi8658_data_t d;
    s->imu_ok = s_imu_ok && qmi8658_read(&d) == HAL_OK;
    if (s->imu_ok) {
        for (int i = 0; i < 3; i++) { s->acc[i] = d.acc[i]; s->gyro[i] = d.gyro[i]; }
    }
}

void tlm_port_write_motors(const uint16_t us[4]) { esc_write(us); }

uint8_t tlm_port_can_arm(void)
{
    return !s_killed && s_imu_ok && (HAL_GetTick() - s_boot_ms) >= ESC_ARM_MS;
}

uint8_t tlm_port_killed(void) { return s_killed; }
