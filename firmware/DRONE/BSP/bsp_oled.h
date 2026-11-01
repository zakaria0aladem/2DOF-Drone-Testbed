/*
 * bsp_oled.h
 *
 *  Created on: Nov 1, 2026
 *      Author: zakar
 */

#ifndef BSP_OLED_H_
#define BSP_OLED_H_

uint8_t OLED_Init(void);
void OLED_Update(void);
void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t on);
void OLED_Clear(void);
void OLED_DrawChar(uint8_t x, uint8_t y, char c);
void OLED_Print(uint8_t x, uint8_t y, const char *text);
void OLED_PrintFloat(uint8_t x, uint8_t y, float value);

#endif /* BSP_OLED_H_ */
