#ifndef _SSD1306_H_
#define _SSD1306_H_

#include "stm32f10x.h"                  // Device header
#include "FONTS.h"
#include "stdbool.h"

#define DC_HIGH_DATA GPIOB ->ODR |= GPIO_ODR_ODR10;
#define DC_LOW_COMMAND 	GPIOB-> ODR &= ~GPIO_ODR_ODR10;
#define RESET_HIGH GPIOB-> ODR |= GPIO_ODR_ODR1;
#define RESET_LOW	GPIOB-> ODR &= ~GPIO_ODR_ODR1;
#define SSD1306_LCDWIDTH 128
#define SSD1306_LCDHEIGHT 64
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifndef pgm_read_dword
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#endif

volatile typedef enum Dispaly_status{
	Drawing_Compleated,
	Drawing_NOT_Compleated,
	Clear_NOT_Compleated, 
  Clear_Compleated
}Dispaly_status_t;

// functions prototypes
char (*Display_Buffer(void))[SSD1306_LCDWIDTH];
void Adafruit_GFX_drawBitmap(int8_t x, int8_t y, const uint8_t *bitmap, uint16_t size);
void SSD1306_Init(void); // to enable SSD1306
void DrawPixel(const uint8_t x, const uint8_t y, char (*buf_ptr)[SSD1306_LCDWIDTH]);
Dispaly_status_t SSD1306_Draw_Display(char (*buf_ptr)[SSD1306_LCDWIDTH]);
Dispaly_status_t SSD1306_Clear_Display(char (*buf_ptr)[SSD1306_LCDWIDTH]);
void Draw_Line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
//void Draw_grapgh (void);
void Set_Cursor (uint8_t x, uint8_t y);
void ssd1306_WriteString(char *str);  
void ssd1306_RESET (void);
void Adafruit_GFX_drawChar(unsigned char c, char (*buff_ptr)[SSD1306_LCDWIDTH]);
void Adafruit_GFX_setFont(const GFXfont *f);
void Adafruit_GFX_drawCircle(int16_t r);

#endif
