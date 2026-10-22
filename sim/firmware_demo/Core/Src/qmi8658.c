/*
 * qmi8658.c - minimal QMI8658 driver (I2C2, polling).
 *
 * Settings: accel +-8 g, gyro +-512 dps, both at ODR setting 0011
 * (896.8 Hz when both sensors run), little-endian, register auto-increment.
 */
#include "qmi8658.h"

#define G0          9.80665f
#define DEG2RAD     0.017453292f
#define ACC_LSB_G   4096.0f     /* +-8 g   -> 4096 LSB/g   (datasheet table 7) */
#define GYR_LSB_DPS 64.0f       /* +-512 dps -> 64 LSB/dps (datasheet table 8) */

static I2C_HandleTypeDef *s_i2c;
static float s_gyro_bias[3];

static HAL_StatusTypeDef wr(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(s_i2c, QMI8658_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
}

static HAL_StatusTypeDef rd(uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(s_i2c, QMI8658_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 10);
}

HAL_StatusTypeDef qmi8658_init(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;
    s_i2c = hi2c;

    wr(QMI_RESET, 0xB0);                 /* soft reset, then wait for it to boot */
    HAL_Delay(20);

    if (rd(QMI_WHO_AM_I, &id, 1) != HAL_OK || id != 0x05)
        return HAL_ERROR;

    if (wr(QMI_CTRL1, 0x40) != HAL_OK) return HAL_ERROR;   /* auto-increment, little-endian */
    if (wr(QMI_CTRL2, 0x23) != HAL_OK) return HAL_ERROR;   /* aFS=010 (+-8 g), aODR=0011 */
    if (wr(QMI_CTRL3, 0x53) != HAL_OK) return HAL_ERROR;   /* gFS=101 (+-512 dps), gODR=0011 */
    if (wr(QMI_CTRL5, 0x00) != HAL_OK) return HAL_ERROR;   /* on-chip low-pass filters off */
    if (wr(QMI_CTRL7, 0x03) != HAL_OK) return HAL_ERROR;   /* aEN + gEN */
    HAL_Delay(160);                       /* gyro turn-on time is 150 ms + 3/ODR */
    return HAL_OK;
}

static HAL_StatusTypeDef read_raw(int16_t raw[6])
{
    uint8_t b[12];
    if (rd(QMI_AX_L, b, sizeof b) != HAL_OK) return HAL_ERROR;
    for (int i = 0; i < 6; i++)
        raw[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
    return HAL_OK;
}

HAL_StatusTypeDef qmi8658_calibrate_gyro(uint16_t n)
{
    float sum[3] = { 0 };
    int16_t raw[6];
    for (uint16_t k = 0; k < n; k++) {
        if (read_raw(raw) != HAL_OK) return HAL_ERROR;
        for (int i = 0; i < 3; i++) sum[i] += raw[3 + i] / GYR_LSB_DPS * DEG2RAD;
        HAL_Delay(1);
    }
    for (int i = 0; i < 3; i++) s_gyro_bias[i] = sum[i] / n;
    return HAL_OK;
}

HAL_StatusTypeDef qmi8658_read(qmi8658_data_t *out)
{
    int16_t raw[6];
    if (read_raw(raw) != HAL_OK) return HAL_ERROR;
    for (int i = 0; i < 3; i++) {
        out->acc[i]  = raw[i] / ACC_LSB_G * G0;
        out->gyro[i] = raw[3 + i] / GYR_LSB_DPS * DEG2RAD - s_gyro_bias[i];
    }
    return HAL_OK;
}
