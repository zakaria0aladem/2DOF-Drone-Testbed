/*
 * bsp_yaw.h
 *
 *  Created on: Nov 20, 2026
 *      Author: zakar
 */

#ifndef BSP_YAW_H_
#define BSP_YAW_H_


void YAW_Init(void);
void YAW_SetTarget(float target_deg);
void YAW_Update(void);
float YAW_GetOutput(void);
float YAW_GetTarget(void);
void YAW_SetGains(float new_kp, float new_ki, float new_kd);
float YAW_GetKp(void);
float YAW_GetKd(void);
float YAW_GetKi(void);

#endif /* BSP_YAW_H_ */
