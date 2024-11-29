#include "OLED.h"
#include "OLED_Font.h"
#include <string.h>

// I2C handle pointer
static I2C_HandleTypeDef *OLED_hi2c;

// Send a command to OLED
static void OLED_WriteCommand(uint8_t Command) {
    uint8_t data[2] = {0x00, Command};
    HAL_I2C_Master_Transmit(OLED_hi2c, OLED_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

// Send data to OLED
static void OLED_WriteData(uint8_t Data) {
    uint8_t data[2] = {0x40, Data};
    HAL_I2C_Master_Transmit(OLED_hi2c, OLED_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

// Initialize OLED
void OLED_Init(I2C_HandleTypeDef *hi2c) {
    OLED_hi2c = hi2c;

    // Delay for power stabilization
    HAL_Delay(100);

    // Initialization commands
    OLED_WriteCommand(0xAE); // Display off
    OLED_WriteCommand(0xD5); // Set display clock divide ratio
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xA8); // Set multiplex ratio
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xD3); // Set display offset
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x40); // Set display start line
    OLED_WriteCommand(0x8D); // Charge pump
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0x20); // Set memory addressing mode
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0xA1); // Set segment re-map
    OLED_WriteCommand(0xC8); // Set COM output scan direction
    OLED_WriteCommand(0xDA); // Set COM pins hardware configuration
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0x81); // Set contrast control
    OLED_WriteCommand(0xCF);
    OLED_WriteCommand(0xD9); // Set pre-charge period
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDB); // Set VCOMH deselect level
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0xA4); // Entire display ON
    OLED_WriteCommand(0xA6); // Set normal display
    OLED_WriteCommand(0xAF); // Display ON

    // Clear the display
    OLED_Clear();
}

// Clear the OLED screen
void OLED_Clear(void) {
    for (uint8_t i = 0; i < 8; i++) {
        OLED_SetCursor(i, 0);
        for (uint8_t j = 0; j < 128; j++) {
            OLED_WriteData(0x00);
        }
    }
}

// Set cursor position
void OLED_SetCursor(uint8_t Y, uint8_t X) {
    OLED_WriteCommand(0xB0 + Y);                    // Set page address
    OLED_WriteCommand(0x00 + (X & 0x0F));          // Set lower column address
    OLED_WriteCommand(0x10 + ((X >> 4) & 0x0F));   // Set higher column address
}

// Display a string on OLED
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String) {
    uint8_t i = 0;

    // 绘制字符串的上半部分
    OLED_SetCursor((Line - 1) * 2, Column * 8); // 上半部分起始位置
    while (String[i] != '\0') {
        for (uint8_t j = 0; j < 8; j++) {
            OLED_WriteData(OLED_F8x16[String[i] - ' '][j]);
        }
        i++;
    }

    // 绘制字符串的下半部分
    i = 0;
    OLED_SetCursor((Line - 1) * 2 + 1, Column * 8); // 下半部分起始位置
    while (String[i] != '\0') {
        for (uint8_t j = 0; j < 8; j++) {
            OLED_WriteData(OLED_F8x16[String[i] - ' '][j + 8]);
        }
        i++;
    }
}

/*
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char) {
    // 显示上半部分
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);
    for (uint8_t i = 0; i < 8; i++) {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);
    }

    // 显示下半部分
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);
    for (uint8_t i = 0; i < 8; i++) {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);
    }
}*/


// Display a number on OLED
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length) {
    char str[11]; // Max length for 32-bit integer + null terminator
    snprintf(str, sizeof(str), "%0*u", Length, Number);
    OLED_ShowString(Line, Column, str);
}
