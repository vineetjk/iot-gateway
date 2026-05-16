/**
 * ssd1306.h
 * =========================================================================
 * SSD1306 0.91" 128x32 OLED driver — I2C1 (PB6=SCL, PB7=SDA).
 * I2C address: 0x3C (SA0=GND). Frame buffer: 512 bytes.
 * =========================================================================
 */
#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define SSD1306_ADDR   (0x3C << 1)
#define SSD1306_W      128
#define SSD1306_H      32

typedef enum { Black=0, White=1 } SSD1306_Color;

typedef struct { uint8_t w, h; const uint8_t *data; } FontDef;

extern FontDef Font_6x8;
extern FontDef Font_7x10;

void SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_Clear(void);
void SSD1306_SetCursor(uint8_t x, uint8_t y);
void SSD1306_WriteString(const char *str, FontDef font, SSD1306_Color col);
void SSD1306_WriteStringAt(int16_t x, uint8_t y, const char *str,
                            FontDef font, SSD1306_Color col);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_Color col);

/* Drawing primitives */
void SSD1306_DrawHLine(uint8_t x, uint8_t y, uint8_t w, SSD1306_Color col);
void SSD1306_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_Color col);
void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_Color col);
void SSD1306_InvertScreen(void);
void SSD1306_DrawBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                         const uint8_t *data, SSD1306_Color col);

/* Large numeric font (digits, '.', '-', ' ') — 12x16 */
extern FontDef Font_12x16;
void SSD1306_WriteLargeString(int16_t x, uint8_t y, const char *s, SSD1306_Color col);

#endif /* SSD1306_H */
