
#include "stm32f10x.h"                  // Device header
#include "Short_Delays_us_ms.h"
#include "SPI_Transmitt.h" 
#include "SSD1306.h"
#include "Solder_Station_STM32.h" 
#include "stdlib.h" 
#include "stdio.h"

//display bufer
 char (*Display_Buffer(void))[SSD1306_LCDWIDTH]{
	static char buffer[SSD1306_LCDHEIGHT/8][SSD1306_LCDWIDTH];
	return buffer;
}
 // set one pixel
 void DrawPixel(const uint8_t x, const uint8_t y, char (*buf_ptr)[SSD1306_LCDWIDTH]){
	static bool Display_Cleared = false; 
	 
	 if(Display_Cleared == false){
		Display_Cleared = true; 
		DMA1_Channel5->CCR &= ~DMA_CCR5_EN;  
	 	DMA1_Channel5->CCR |= DMA_CCR5_MINC;
	  DMA1_Channel5->CMAR = (uint32_t)(buf_ptr);
}
		*(*(buf_ptr+y/8)+x) |= 1<<(y%8); // set pixel
}

// draw full display
Dispaly_status_t SSD1306_Draw_Display(char (*buf_ptr)[SSD1306_LCDWIDTH]){
	Dispaly_status_t DrawingStatus = Drawing_NOT_Compleated;
	   DMA1_Channel5->CCR &= ~DMA_CCR5_EN; 
		 DMA1_Channel5->CNDTR = 1024;  
		 DMA1_Channel5->CCR |= DMA_CCR5_EN;
		 while (!(DMA1->ISR & DMA_ISR_TCIF5)){}
		 DMA1->IFCR  |=  DMA_IFCR_CTCIF5;
	   while((!(SPI2->SR & SPI_SR_TXE)) && ((SPI2->SR & SPI_SR_BSY))){}
			 #ifdef DEBUG
				Debug_UARTSend("SSD1306 Draw Display Done\n\r");
			 #endif
		DrawingStatus = Drawing_Compleated;
		return (DrawingStatus);
}
// clear whole display 
Dispaly_status_t SSD1306_Clear_Display(char (*buf_ptr)[SSD1306_LCDWIDTH]){
	Dispaly_status_t ClearStatus = Clear_NOT_Compleated;
	for (uint8_t i = 0; i < 64; i++){
	   for(uint8_t j = 0; j < 128; j++)
			*(*(buf_ptr + i/8) + j) = 0x00;
	}
	Set_Cursor(0, 0);
	ClearStatus = Clear_Compleated;
	return (ClearStatus);
}
// reset SSD1306
 void ssd1306_RESET (void)
 {
	 RESET_HIGH;
	 Short_Delays_Systik_us_ms(10, ms);
	 RESET_LOW;
	 Short_Delays_Systik_us_ms(10, ms);
	 RESET_HIGH;
 }

 
 void Adafruit_GFX_drawBitmap(int8_t x, int8_t y, const uint8_t *bitmap, uint16_t size) 
{
	uint8_t byte = 0;
	for(uint16_t i=0; i<size; i++){
			byte = bitmap[i];
			for(uint8_t j=0; j<8; j++){
			if(byte & 0x80){
				DrawPixel(x+j, y+i, Display_Buffer());
			}
			byte <<= 1;	
		}
			
		}
}

void SSD1306_Init(void)      // to fully init SSD1306
{
		DC_LOW_COMMAND;
		SPI2_SEND_Command(0xAE); //display off
		SPI2_SEND_Command(0x20);//Set Memory Addressing Mode
		SPI2_SEND_Command(0x10);// 00,Horizontal Addressing Mode; 01,Vertical Addressing Mode;
                                // 10,Page Addressing Mode (RESET); 11,Invalid
		SPI2_SEND_Command(0xC8);//mirror vertically (normal mode)
		
		SPI2_SEND_Command(0x21); //set column command
		SPI2_SEND_Command(0); //start column
		SPI2_SEND_Command(127); //end column
		SPI2_SEND_Command(0x22); //set page 	
		SPI2_SEND_Command(0); 	// start page
		SPI2_SEND_Command(7); // end page
	
		SPI2_SEND_Command(0x40); //--set start line address - CHECK
		SPI2_SEND_Command(0x80); //--set contrast control register - CHECK
		SPI2_SEND_Command(0xFF); // do not change influence on start point of pixels
    SPI2_SEND_Command(0xA1);  // Mirror horizontally (normal mode)
		SPI2_SEND_Command(0xA6); //--set normal color
		SPI2_SEND_Command(0xA8); //--set multiplex ratio(1 to 64) - CHECK
		SPI2_SEND_Command(0x3F);
		SPI2_SEND_Command(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content !!!!!!!!!!!!!!!!!
		SPI2_SEND_Command(0xD3); //-set display offset - CHECK
		SPI2_SEND_Command(0x00); //-not offset
		SPI2_SEND_Command(0xD5); //--set display clock divide ratio/oscillator frequency
		SPI2_SEND_Command(0x10); //--set divide ratio
		SPI2_SEND_Command(0xD9); //--set pre-charge period
		SPI2_SEND_Command(0x22);
		SPI2_SEND_Command(0xDA); //--set com pins hardware configuration - CHECK
		SPI2_SEND_Command(0x12);
		SPI2_SEND_Command(0xDB); //--set vcomh
		SPI2_SEND_Command(0x20); //0x20,0.77xVcc
		SPI2_SEND_Command(0x8D); //--set DC-DC enable
		SPI2_SEND_Command(0x14);
		SPI2_SEND_Command(0xAF); //--turn on SSD1306 panel // enable OLED	
		DC_HIGH_DATA;
		#ifdef DEBUG
			Debug_UARTSend("SSD1306 was initted\n\r");
		#endif
}
// position on display
typedef struct
{
 	uint16_t x;
	uint16_t y;
}Pos;
Pos position;

// set cursor to desired position
void Set_Cursor (uint8_t x, uint8_t y)
{
	position.x = x;
	position.y = y;
}
// init TMR2
void TIM2_init (void){
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 &=~(TIM_CR1_UDIS);
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->ARR =  45000; // set auto reload value
	TIM2->PSC = 100 ;
	TIM2->CR1 |= TIM_CR1_CEN;
}
// help functions from GFX
GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
  return gfxFont->glyph + c;
}

uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont) {
	
  return gfxFont->bitmap;

}
// write full string
void ssd1306_WriteString(char* str) 
{
    while (*str !='\0') {
			 Adafruit_GFX_drawChar(*str, Display_Buffer());
        // Next char
        str++;
    }  
}
// draw one char
void Adafruit_GFX_drawChar(unsigned char c, char (*buff_ptr)[SSD1306_LCDWIDTH]) {
		
    c -= 32;// the same as (uint8_t)pgm_read_byte(&gfxFont->first);
    GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c);
    uint8_t *bitmap = pgm_read_bitmap_ptr(gfxFont);
		
    uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
    uint8_t w = pgm_read_byte(&glyph->width); 
	  uint8_t h = pgm_read_byte(&glyph->height); 
    //int8_t xo = pgm_read_byte(&glyph->xOffset),
          // yo = pgm_read_byte(&glyph->yOffset);
   uint8_t bits = 0, bit = 0;

    for (uint8_t yy = 0; yy < h; yy++) {
      for (uint8_t  xx = 0; xx < w; xx++) {
        if (!(bit++ &7)) {
          bits = pgm_read_byte(&bitmap[bo++]);
        }
        if (bits & 0x80) {
            DrawPixel(position.x+ xx, position.y + yy, Display_Buffer());
        }
        bits <<= 1;
      }
    }
		position.x += pgm_read_word(&glyph->xAdvance);  // edited, to have space between characters
  } 

	void Adafruit_GFX_setFont(const GFXfont *f) {
		
		gfxFont = (GFXfont *)f;
}

// draw circle in some position
void Adafruit_GFX_drawCircle(int16_t r) {

  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  DrawPixel(position.x, position.y + r, Display_Buffer());
  DrawPixel(position.x, position.y - r, Display_Buffer());
  DrawPixel(position.x + r, position.y, Display_Buffer());
  DrawPixel(position.x - r, position.y, Display_Buffer());

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    DrawPixel(position.x + x, position.y + y,Display_Buffer());
    DrawPixel(position.x - x, position.y + y, Display_Buffer());
    DrawPixel(position.x + x, position.y - y, Display_Buffer());
    DrawPixel(position.x - x, position.y - y, Display_Buffer());
    DrawPixel(position.x + y, position.y + x, Display_Buffer());
    DrawPixel(position.x - y, position.y + x, Display_Buffer());
    DrawPixel(position.x + y, position.y - x, Display_Buffer());
    DrawPixel(position.x - y, position.y - x, Display_Buffer());
		
  }
}
