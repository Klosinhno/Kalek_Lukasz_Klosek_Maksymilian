#ifndef LCD_I2C_H_
#define LCD_I2C_H_

#include "main.h"

// Adres I2C (domyślny dla większości to 0x27 << 1 = 0x4E)
// Jeśli nie działa, spróbuj 0x3F << 1 = 0x7E
#define LCD_ADDR (0x27 << 1)

void LCD_Init(void);
void LCD_SendString(char *str);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);

#endif /* LCD_I2C_H_ */
