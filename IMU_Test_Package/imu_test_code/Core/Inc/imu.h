/*
 * imu.h - IMU driver for the Hiwonder ROS Robot Controller V1.2
 *
 * The on-board IMU (U4) is either an MPU6050 or a QMI8658, depending on the
 * board batch. Both sit on I2C2 (PB10 = SCL, PB11 = SDA). IMU_Init() scans the
 * bus, identifies the chip from its WHO_AM_I register and configures it.
 */
#ifndef IMU_H
#define IMU_H

#include "main.h"

typedef enum {
    IMU_NONE = 0,
    IMU_MPU6050,
    IMU_QMI8658
} imu_type_t;

typedef struct {
    float ax, ay, az;   /* acceleration in g        */
    float gx, gy, gz;   /* angular rate in deg/s    */
} imu_data_t;

/* Scan the bus, print every address that answers, identify and set up the IMU.
 * Returns the chip type found (IMU_NONE if nothing usable answered). */
imu_type_t IMU_Init(I2C_HandleTypeDef *hi2c);

/* Read one sample. Returns HAL_OK on success. */
HAL_StatusTypeDef IMU_Read(imu_data_t *d);

/* Human-readable name of the detected chip. */
const char *IMU_Name(void);

#endif /* IMU_H */
