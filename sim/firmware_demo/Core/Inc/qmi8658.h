/*
 * qmi8658.h - minimal QMI8658 driver (I2C, polling) for the Hiwonder RRC v1.2.
 *
 * The IMU sits on I2C2 (PB10 SCL, PB11 SDA) at 7-bit address 0x6A.
 * Register numbers are from the QMI8658A datasheet (docs/datasheets).
 */
#ifndef QMI8658_H
#define QMI8658_H

#include "stm32f4xx_hal.h"

#define QMI8658_ADDR        0x6Au   /* 7-bit, SA0 low */

#define QMI_WHO_AM_I        0x00u   /* reads 0x05 */
#define QMI_REVISION_ID     0x01u
#define QMI_CTRL1           0x02u
#define QMI_CTRL2           0x03u   /* accel full scale + ODR */
#define QMI_CTRL3           0x04u   /* gyro full scale + ODR */
#define QMI_CTRL5           0x06u
#define QMI_CTRL7           0x08u   /* sensor enable */
#define QMI_STATUS0         0x2Eu
#define QMI_TEMP_L          0x33u
#define QMI_AX_L            0x35u   /* AX_L .. GZ_H = 12 bytes */
#define QMI_RESET           0x60u   /* write 0xB0 = soft reset */

typedef struct {
    float acc[3];   /* m/s^2 */
    float gyro[3];  /* rad/s, bias removed */
} qmi8658_data_t;

/* Returns HAL_OK if WHO_AM_I matched and the sensor was configured. */
HAL_StatusTypeDef qmi8658_init(I2C_HandleTypeDef *hi2c);

/* Averages n gyro samples with the rig still and stores them as the bias. */
HAL_StatusTypeDef qmi8658_calibrate_gyro(uint16_t n);

/* One burst read of accel + gyro (12 bytes). */
HAL_StatusTypeDef qmi8658_read(qmi8658_data_t *out);

#endif /* QMI8658_H */
