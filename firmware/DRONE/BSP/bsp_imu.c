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

#define IMU_PORT		GPIOB
#define IMU_SCL_PIN		GPIO_PIN_10
#define IMU_SDA_PIN		GPIO_PIN_11

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

#define CAL_SAMPLES		250		/* x 4 ms = 1 s of gyro averaging at start-up         */
#define FILTER_TAU_S	0.5f	/* faster than 0.5 s trust the gyro, slower the accel */
#define PRINT_PERIOD_MS	100

static IMU_Data_t imu_data;
static uint8_t imu_found = 0;
static uint32_t last_print_ms = 0;
static uint32_t last_cycles = 0;
static float gyro_bias[3];

static void IMU_WriteReg(uint8_t reg, uint8_t value){
	HAL_I2C_Mem_Write(&hi2c2, IMU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 50);
}

/*reads one sample: acc in g, gyro in deg/s (bias not removed), returns 1 if ok*/
static uint8_t IMU_ReadRaw(float acc[3], float gyro[3]){
	uint8_t b[12];

	if(HAL_I2C_Mem_Read(&hi2c2, IMU_ADDR, REG_AX_L, I2C_MEMADD_SIZE_8BIT, b, 12, 50) != HAL_OK){
		return 0;
	}
	for(uint8_t i = 0; i < 3; i++){
		acc[i]  = (int16_t)(b[2*i + 1] << 8 | b[2*i])     / ACC_LSB_PER_G;
		gyro[i] = (int16_t)(b[2*i + 7] << 8 | b[2*i + 6]) / GYR_LSB_PER_DPS;
	}
	return 1;
}

/*roll and pitch from gravity alone, valid while the rig is not accelerating*/
static void IMU_AccAngles(const float acc[3], float *roll, float *pitch){
	*roll  = atan2f(acc[1], acc[2]) * RAD_TO_DEG;
	*pitch = atan2f(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2])) * RAD_TO_DEG;
}

/*frees the bus if a reset cut the IMU off in the middle of a read:
  9 clock pulses let it finish sending, then a STOP ends the transfer*/
static void IMU_BusRecover(void){
	GPIO_InitTypeDef gpio = {0};

	HAL_I2C_DeInit(&hi2c2);
	__HAL_RCC_I2C2_FORCE_RESET();
	__HAL_RCC_I2C2_RELEASE_RESET();

	HAL_GPIO_WritePin(IMU_PORT, IMU_SCL_PIN | IMU_SDA_PIN, GPIO_PIN_SET);
	gpio.Pin = IMU_SCL_PIN | IMU_SDA_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_OD;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IMU_PORT, &gpio);

	for(uint8_t i = 0; i < 9; i++){
		HAL_GPIO_WritePin(IMU_PORT, IMU_SCL_PIN, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(IMU_PORT, IMU_SCL_PIN, GPIO_PIN_SET);
		HAL_Delay(1);
	}
	HAL_GPIO_WritePin(IMU_PORT, IMU_SDA_PIN, GPIO_PIN_RESET);	/* STOP: SDA low -> high */
	HAL_Delay(1);
	HAL_GPIO_WritePin(IMU_PORT, IMU_SDA_PIN, GPIO_PIN_SET);
	HAL_Delay(1);

	HAL_I2C_Init(&hi2c2);	/* puts PB10/PB11 back on I2C2 */
}

/*averages the gyro while the rig is still: that average is the bias,
  then starts the filter from the accelerometer angle*/
static void IMU_Calibrate(void){
	float acc[3], gyro[3];
	float sum[3] = {0};
	uint16_t n = 0;

	printf("IMU: keep the rig still, calibrating...\r\n");
	for(uint16_t i = 0; i < CAL_SAMPLES; i++){
		if(IMU_ReadRaw(acc, gyro)){
			sum[0] += gyro[0];
			sum[1] += gyro[1];
			sum[2] += gyro[2];
			n++;
		}
		HAL_Delay(4);
	}
	for(uint8_t i = 0; i < 3; i++){
		gyro_bias[i] = (n > 0) ? sum[i] / n : 0.0f;
	}
	printf("IMU: gyro bias %.1f %.1f %.1f dps\r\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

	IMU_AccAngles(acc, &imu_data.roll, &imu_data.pitch);
}

/*frees the bus, checks the chip answers, sets it up and calibrates
  the gyro (rig must be still for 1 s), returns 1 if found*/
uint8_t IMU_Init(void){
	uint8_t id = 0;
	HAL_StatusTypeDef status = HAL_ERROR;
	imu_found = 0;

	HAL_Delay(200);	/* let the IMU power up */
	IMU_BusRecover();

	for(uint8_t attempt = 0; attempt < 3; attempt++){
		status = HAL_I2C_Mem_Read(&hi2c2, IMU_ADDR, REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &id, 1, 50);
		if(status == HAL_OK && id == WHO_AM_I_VALUE){
			break;
		}
		HAL_Delay(10);
	}

	if(status != HAL_OK){
		printf("IMU not found (no answer on I2C2, error %d)\r\n", status);
		return 0;
	}
	if(id != WHO_AM_I_VALUE){
		printf("IMU not found (WHO_AM_I = 0x%02X, expected 0x05)\r\n", id);
		return 0;
	}

	IMU_WriteReg(REG_CTRL1, 0x40);	/* auto-increment, little-endian */
	IMU_WriteReg(REG_CTRL2, 0x15);	/* accel +-4 g, ~250 Hz          */
	IMU_WriteReg(REG_CTRL3, 0x55);	/* gyro +-512 dps, ~250 Hz       */
	IMU_WriteReg(REG_CTRL7, 0x03);	/* enable accel + gyro           */
	HAL_Delay(50);
	printf("IMU found: QMI8658\r\n");

	IMU_Calibrate();

	/* CPU cycle counter, used to measure the exact time between updates */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	last_cycles = DWT->CYCCNT;

	imu_found = 1;
	return 1;
}

/*reads one sample and updates the filtered roll and pitch, call every cycle*/
void IMU_Update(void){
	float acc[3], gyro[3];
	float acc_roll, acc_pitch, dt, k;
	uint32_t now;

	if(imu_found == 0){
		return;
	}
	if(IMU_ReadRaw(acc, gyro) == 0){
		return;
	}

	now = DWT->CYCCNT;
	dt = (float)(now - last_cycles) / (float)SystemCoreClock;
	last_cycles = now;

	imu_data.ax = acc[0];
	imu_data.ay = acc[1];
	imu_data.az = acc[2];
	imu_data.gx = gyro[0] - gyro_bias[0];
	imu_data.gy = gyro[1] - gyro_bias[1];
	imu_data.gz = gyro[2] - gyro_bias[2];

	/* complementary filter: the gyro follows fast motion, the accelerometer
	   slowly pulls the angle back so it does not drift */
	IMU_AccAngles(acc, &acc_roll, &acc_pitch);
	k = FILTER_TAU_S / (FILTER_TAU_S + dt);
	imu_data.roll  = k * (imu_data.roll  + imu_data.gx * dt) + (1.0f - k) * acc_roll;
	imu_data.pitch = k * (imu_data.pitch + imu_data.gy * dt) + (1.0f - k) * acc_pitch;
}

const IMU_Data_t *IMU_GetData(void){
	return &imu_data;
}

/*prints the latest sample, at most once every PRINT_PERIOD_MS*/
void IMU_Print(void){
	if(imu_found == 0){
		return;
	}
	if(HAL_GetTick() - last_print_ms < PRINT_PERIOD_MS){
		return;
	}
	last_print_ms = HAL_GetTick();

	printf("acc[g] %6.2f %6.2f %6.2f | gyro[dps] %7.1f %7.1f %7.1f | roll %6.1f  pitch %6.1f\r\n",
			imu_data.ax, imu_data.ay, imu_data.az,
			imu_data.gx, imu_data.gy, imu_data.gz,
			imu_data.roll, imu_data.pitch);
}
