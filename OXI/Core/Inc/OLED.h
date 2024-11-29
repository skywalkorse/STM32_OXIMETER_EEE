#ifndef __OLED_H
#define __OLED_H

#include "stm32l4xx_hal.h"

// OLED I2C Address
#define OLED_I2C_ADDR 0x78

// Public Functions
void OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t Y, uint8_t X);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif
