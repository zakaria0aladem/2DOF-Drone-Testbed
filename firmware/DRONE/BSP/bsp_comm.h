/*
 * bsp_comm.h
 *
 *  Created on: Nov 8, 2026
 *      Author: amr
 *
 *  Raspberry Pi link on USART3 (UART2 USB-C port, 115200 8-N-1).
 *  Every message is one text line:  $<fields>*<checksum>\r\n
 *  checksum = XOR of all characters between '$' and '*', as 2 hex digits.
 *
 *  Board -> Pi, every 20 ms (50 Hz):
 *    $TEL,ms,pitch,enc_pitch,enc_yaw,target,output,left_us,right_us,armed,link_lost,bad*CS
 *  Pi -> board, about 10 times a second (also works as the heartbeat):
 *    $SET,<target deg>*CS
 */

#ifndef BSP_COMM_H_
#define BSP_COMM_H_

#include <stdint.h>

void COMM_Init(void);
void COMM_Update(void);
uint8_t COMM_IsLinkLost(void);

#endif /* BSP_COMM_H_ */
