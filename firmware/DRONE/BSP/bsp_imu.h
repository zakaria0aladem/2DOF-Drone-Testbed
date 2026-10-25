/*
 * bsp_imu.h
 *
 *  Created on: Oct 25, 2026
 *      Author: amr
 */

#ifndef BSP_IMU_H_
#define BSP_IMU_H_

#include <stdint.h>

typedef struct {
	float ax, ay, az;	/* acceleration [g]                          */
	float gx, gy, gz;	/* angular rate, bias removed [deg/s]        */
	float roll, pitch;	/* angle from the complementary filter [deg] */
} IMU_Data_t;

uint8_t IMU_Init(void);
void IMU_Update(void);
const IMU_Data_t *IMU_GetData(void);
void IMU_Print(void);




#endif /* BSP_IMU_H_ */
