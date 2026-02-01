/*
 * lcd_i2c.c
 *
 *  Created on: 1 lut 2026
 *      Author: kalek
 */


#include "lcd_i2c.h"

extern I2C_HandleTypeDef hi2c2; // Odwołanie do uchwytu I2C2 z main.c/i2c.c

void LCD_Write_CMD(uint8_t cmd)
{
  uint8_t data_u, data_l;
  uint8_t data_t[4];
  data_u = (cmd & 0xf0);
  data_l = ((cmd << 4) & 0xf0);
  data_t[0] = data_u | 0x0C;  // en=1, rs=0
  data_t[1] = data_u | 0x08;  // en=0, rs=0
  data_t[2] = data_l | 0x0C;  // en=1, rs=0
  data_t[3] = data_l | 0x08;  // en=0, rs=0
  HAL_I2C_Master_Transmit(&hi2c2, LCD_ADDR, (uint8_t *)data_t, 4, 100);
}

void LCD_Write_DATA(uint8_t data)
{
  uint8_t data_u, data_l;
  uint8_t data_t[4];
  data_u = (data & 0xf0);
  data_l = ((data << 4) & 0xf0);
  data_t[0] = data_u | 0x0D;  // en=1, rs=1
  data_t[1] = data_u | 0x09;  // en=0, rs=1
  data_t[2] = data_l | 0x0D;  // en=1, rs=1
  data_t[3] = data_l | 0x09;  // en=0, rs=1
  HAL_I2C_Master_Transmit(&hi2c2, LCD_ADDR, (uint8_t *)data_t, 4, 100);
}

void LCD_Init(void)
{
  HAL_Delay(50);
  LCD_Write_CMD(0x30);
  HAL_Delay(5);
  LCD_Write_CMD(0x30);
  HAL_Delay(1);
  LCD_Write_CMD(0x30);
  HAL_Delay(10);
  LCD_Write_CMD(0x20); // 4-bit mode
  HAL_Delay(10);

  LCD_Write_CMD(0x28); // Function set: DL=0 (4-bit), N=1 (2 lines), F=0 (5x8)
  HAL_Delay(1);
  LCD_Write_CMD(0x08); // Display off
  HAL_Delay(1);
  LCD_Write_CMD(0x01); // Clear display
  HAL_Delay(1);
  LCD_Write_CMD(0x06); // Entry mode set
  HAL_Delay(1);
  LCD_Write_CMD(0x0C); // Display on, Cursor off
}

void LCD_SendString(char *str)
{
  while (*str) LCD_Write_DATA(*str++);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t address;
    if (row == 0) address = 0x80 + col;
    else          address = 0xC0 + col;
    LCD_Write_CMD(address);
}

void LCD_Clear(void)
{
  LCD_Write_CMD(0x01);
  HAL_Delay(2);
}
