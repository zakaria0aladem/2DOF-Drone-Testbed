/*
 * imu.c - IMU driver for the Hiwonder ROS Robot Controller V1.2
 * Supports MPU6050 and QMI8658 on I2C2 (PB10 SCL / PB11 SDA).
 */
#include "imu.h"
#include <stdio.h>

/* ---------------- MPU6050 registers ---------------- */
#define MPU_WHO_AM_I      0x75   /* returns 0x68 (0x70/0x71/0x72 on some clones) */
#define MPU_PWR_MGMT_1    0x6B
#define MPU_CONFIG        0x1A
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_ACCEL_XOUT_H  0x3B   /* 14 bytes: AX AY AZ TEMP GX GY GZ, big-endian */
#define MPU_ACC_LSB_PER_G   16384.0f  /* +-2 g      */
#define MPU_GYR_LSB_PER_DPS 131.0f    /* +-250 dps  */

/* ---------------- QMI8658 registers ---------------- */
#define QMI_WHO_AM_I      0x00   /* returns 0x05 */
#define QMI_CTRL1         0x02
#define QMI_CTRL2         0x03   /* accelerometer range + data rate */
#define QMI_CTRL3         0x04   /* gyroscope range + data rate     */
#define QMI_CTRL7         0x08   /* sensor enable                   */
#define QMI_AX_L          0x35   /* 12 bytes: AX AY AZ GX GY GZ, little-endian */
#define QMI_ACC_LSB_PER_G   8192.0f   /* +-4 g      */
#define QMI_GYR_LSB_PER_DPS 64.0f     /* +-512 dps  */

static I2C_HandleTypeDef *bus;
static uint8_t     dev_addr;          /* 7-bit address */
static imu_type_t  dev_type = IMU_NONE;

static HAL_StatusTypeDef wr(uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(bus, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 50);
}

static HAL_StatusTypeDef rd(uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(bus, dev_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 50);
}

static int try_address(uint8_t addr, uint8_t reg, uint8_t *id)
{
    dev_addr = addr;
    return (rd(reg, id, 1) == HAL_OK);
}

imu_type_t IMU_Init(I2C_HandleTypeDef *hi2c)
{
    bus = hi2c;
    uint8_t id;

    /* 1. Bus scan: lists every device that answers */
    printf("I2C2 scan:");
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (HAL_I2C_IsDeviceReady(bus, a << 1, 2, 5) == HAL_OK) {
            printf(" 0x%02X", a);
            found++;
        }
    }
    printf(found ? "\r\n" : " nothing found\r\n");

    /* 2. MPU6050 at 0x68 / 0x69 */
    for (uint8_t a = 0x68; a <= 0x69; a++) {
        if (try_address(a, MPU_WHO_AM_I, &id)) {
            printf("Address 0x%02X, WHO_AM_I(0x75) = 0x%02X\r\n", a, id);
            if (id == 0x68 || id == 0x70 || id == 0x71 || id == 0x72) {
                wr(MPU_PWR_MGMT_1, 0x01);    /* wake up, PLL clock from gyro X */
                HAL_Delay(50);
                wr(MPU_CONFIG, 0x03);        /* digital low-pass ~44 Hz        */
                wr(MPU_GYRO_CONFIG, 0x00);   /* +-250 dps                      */
                wr(MPU_ACCEL_CONFIG, 0x00);  /* +-2 g                          */
                dev_type = IMU_MPU6050;
                return dev_type;
            }
        }
    }

    /* 3. QMI8658 at 0x6A / 0x6B */
    for (uint8_t a = 0x6A; a <= 0x6B; a++) {
        if (try_address(a, QMI_WHO_AM_I, &id)) {
            printf("Address 0x%02X, WHO_AM_I(0x00) = 0x%02X\r\n", a, id);
            if (id == 0x05) {
                wr(QMI_CTRL1, 0x40);         /* register auto-increment, little-endian */
                wr(QMI_CTRL2, 0x15);         /* accel +-4 g, ~250 Hz                   */
                wr(QMI_CTRL3, 0x55);         /* gyro +-512 dps, ~250 Hz                */
                wr(QMI_CTRL7, 0x03);         /* enable accel + gyro                    */
                HAL_Delay(50);
                dev_type = IMU_QMI8658;
                return dev_type;
            }
        }
    }

    dev_type = IMU_NONE;
    return dev_type;
}

HAL_StatusTypeDef IMU_Read(imu_data_t *d)
{
    uint8_t b[14];
    HAL_StatusTypeDef st;

    if (dev_type == IMU_MPU6050) {
        st = rd(MPU_ACCEL_XOUT_H, b, 14);
        if (st != HAL_OK) return st;
        int16_t ax = (int16_t)(b[0]  << 8 | b[1]);
        int16_t ay = (int16_t)(b[2]  << 8 | b[3]);
        int16_t az = (int16_t)(b[4]  << 8 | b[5]);
        int16_t gx = (int16_t)(b[8]  << 8 | b[9]);
        int16_t gy = (int16_t)(b[10] << 8 | b[11]);
        int16_t gz = (int16_t)(b[12] << 8 | b[13]);
        d->ax = ax / MPU_ACC_LSB_PER_G;   d->ay = ay / MPU_ACC_LSB_PER_G;   d->az = az / MPU_ACC_LSB_PER_G;
        d->gx = gx / MPU_GYR_LSB_PER_DPS; d->gy = gy / MPU_GYR_LSB_PER_DPS; d->gz = gz / MPU_GYR_LSB_PER_DPS;
        return HAL_OK;
    }

    if (dev_type == IMU_QMI8658) {
        st = rd(QMI_AX_L, b, 12);
        if (st != HAL_OK) return st;
        int16_t ax = (int16_t)(b[1]  << 8 | b[0]);
        int16_t ay = (int16_t)(b[3]  << 8 | b[2]);
        int16_t az = (int16_t)(b[5]  << 8 | b[4]);
        int16_t gx = (int16_t)(b[7]  << 8 | b[6]);
        int16_t gy = (int16_t)(b[9]  << 8 | b[8]);
        int16_t gz = (int16_t)(b[11] << 8 | b[10]);
        d->ax = ax / QMI_ACC_LSB_PER_G;   d->ay = ay / QMI_ACC_LSB_PER_G;   d->az = az / QMI_ACC_LSB_PER_G;
        d->gx = gx / QMI_GYR_LSB_PER_DPS; d->gy = gy / QMI_GYR_LSB_PER_DPS; d->gz = gz / QMI_GYR_LSB_PER_DPS;
        return HAL_OK;
    }

    return HAL_ERROR;
}

const char *IMU_Name(void)
{
    switch (dev_type) {
    case IMU_MPU6050: return "MPU6050";
    case IMU_QMI8658: return "QMI8658";
    default:          return "none";
    }
}
