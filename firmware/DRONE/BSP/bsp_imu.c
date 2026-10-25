/*
 * bsp_imu.c
 *
 *  Created on: Oct 25, 2026
 *      Author: amr
 */


#include "main.h"
#include "bsp_imu.h"
#include <stdio.h>
#include <math.h>

/* QMI8658 on I2C2 (PB10 = SCL, PB11 = SDA) */
extern I2C_HandleTypeDef hi2c2;

#define IMU_ADDR		(0x6A << 1)
#define REG_WHO_AM_I	0x00	/* reads 0x05 on a QMI8658                    */
#define REG_CTRL1		0x02	/* register auto-increment                    */
#define REG_CTRL2		0x03	/* accelerometer range + data rate            */
#define REG_CTRL3		0x04	/* gyroscope range + data rate                */
#define REG_CTRL7		0x08	/* sensor enable                              */
#define REG_AX_L		0x35	/* 12 bytes: AX AY AZ GX GY GZ, little-endian */

#define WHO_AM_I_VALUE	0x05
#define ACC_LSB_PER_G	8192.0f	/* +-4 g     */
#define GYR_LSB_PER_DPS	64.0f	/* +-512 dps */
#define RAD_TO_DEG		57.2958f
#define PRINT_PERIOD_MS	100

static IMU_Data_t imu_data;
static uint8_t imu_found = 0;
static uint32_t last_print_ms = 0;

static void IMU_WriteReg(uint8_t reg, uint8_t value){
	HAL_I2C_Mem_Write(&hi2c2, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 50);
}

/*checks the chip answers and sets it up, returns 1 if found*/
uint8_t IMU_Init(void){
	uint8_t id = 0;
	imu_found = 0;

	HAL_Delay(200);	/* let the IMU power up */
	if(HAL_I2C_Mem_Read(&hi2c2, IMU_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &id, 1, 50) != HAL_OK
			|| id != WHO_AM_I_VALUE){
		printf("IMU not found (WHO_AM_I = 0x%02X)\r\n", id);
		return 0;
	}

	IMU_WriteReg(REG_CTRL1, 0x40);	/* auto-increment, little-endian */
	IMU_WriteReg(REG_CTRL2, 0x15);	/* accel +-4 g, ~250 Hz          */
	IMU_WriteReg(REG_CTRL3, 0x55);	/* gyro +-512 dps, ~250 Hz       */
	IMU_WriteReg(REG_CTRL7, 0x03);	/* enable accel + gyro           */
	HAL_Delay(50);

	imu_found = 1;
	printf("IMU found: QMI8658\r\n");
	return 1;
}

/*reads one sample and works out roll and pitch, call every cycle*/
void IMU_Update(void){
	uint8_t b[12];
	int16_t raw[6];

	if(imu_found == 0){
		return;
	}
	if(HAL_I2C_Mem_Read(&hi2c2, IMU_ADDR, REG_AX_L, I2C_MEMADD_SIZE_8BIT, b, 12, 50) != HAL_OK){
		return;
	}

	for(uint8_t i = 0; i < 6; i++){
		raw[i] = (int16_t)(b[2*i + 1] << 8 | b[2*i]);
	}

	imu_data.ax = raw[0] / ACC_LSB_PER_G;
	imu_data.ay = raw[1] / ACC_LSB_PER_G;
	imu_data.az = raw[2] / ACC_LSB_PER_G;
	imu_data.gx = raw[3] / GYR_LSB_PER_DPS;
	imu_data.gy = raw[4] / GYR_LSB_PER_DPS;
	imu_data.gz = raw[5] / GYR_LSB_PER_DPS;

	/* angles from gravity alone, valid while the rig is not shaking */
	imu_data.roll  = atan2f(imu_data.ay, imu_data.az) * RAD_TO_DEG;
	imu_data.pitch = atan2f(-imu_data.ax,
			sqrtf(imu_data.ay * imu_data.ay + imu_data.az * imu_data.az)) * RAD_TO_DEG;
}

const IMU_Data_t *IMU_GetData(void){
	return &imu_data;
}

/*prints the latest sample, at most once every PRINT_PERIOD_MS*/
void IMU_Print(void){
	if(HAL_GetTick() - last_print_ms < PRINT_PERIOD_MS){
		return;
	}
	last_print_ms = HAL_GetTick();

	printf("acc[g] %6.2f %6.2f %6.2f | gyro[dps] %7.1f %7.1f %7.1f | roll %6.1f  pitch %6.1f\r\n",
			imu_data.ax, imu_data.ay, imu_data.az,
			imu_data.gx, imu_data.gy, imu_data.gz,
			imu_data.roll, imu_data.pitch);
}
