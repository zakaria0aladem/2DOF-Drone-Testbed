/*
 * bsp_pid.h
 *
 *  Created on: Oct 25, 2026
 *      Author: amr
 */

#ifndef BSP_PID_H_
#define BSP_PID_H_

#include <stdint.h>

void PID_Init(void);
void PID_Update(void);
void PID_SetTarget(float target_deg);
void PID_Print(void);




#endif /* BSP_PID_H_ */
