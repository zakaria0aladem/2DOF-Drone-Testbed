/*
 * bsp_enc.h
 *
 *  Created on: Oct 25, 2026
 *      Author: zakar
 */

#ifndef BSP_ENC_H_
#define BSP_ENC_H_

#define ENC_YAW 1
#define ENC_PITCH 2

void ENC_Init(void);
int32_t ENC_GetCount(uint8_t axis);
float ENC_GetDeg(uint8_t axis);
void ENC_Zero(uint8_t axis);


#endif /* BSP_ENC_H_ */
