/*
 * SSD1306.h
 *
 *  Created on: Dec 24, 2018
 *      Author: pouelle9
 */

#ifndef SSD1306_SSD1306_H_
#define SSD1306_SSD1306_H_

#include "stm32l4xx_hal.h"

// Screen dimensions
#define SSD1306ADDRESS  	  0x78
#define SCR_W                 (uint8_t)128 // width
#define SCR_H                 (uint8_t)64  // height

//i2C communication modes
#define COMMAND_MODE		  (uint8_t)0x00
#define DATA_MODE			  (uint8_t)0x40

// SSD1306 command definitions
#define SSD1306_CMD_SETMUX    (uint8_t)0xA8 // Set multiplex ratio (N, number of lines active on display)
#define SSD1306_CMD_SETOFFS   (uint8_t)0xD3 // Set display offset
#define SSD1306_CMD_STARTLINE (uint8_t)0x40 // Set display start line
#define SSD1306_CMD_SEG_NORM  (uint8_t)0xA0 // Column 0 is mapped to SEG0 (X coordinate normal)
#define SSD1306_CMD_SEG_INV   (uint8_t)0xA1 // Column 127 is mapped to SEG0 (X coordinate inverted)
#define SSD1306_CMD_COM_NORM  (uint8_t)0xC0 // Scan from COM0 to COM[N-1] (N - mux ratio, Y coordinate normal)
#define SSD1306_CMD_COM_INV   (uint8_t)0xC8 // Scan from COM[N-1] to COM0 (N - mux ratio, Y coordinate inverted)
#define SSD1306_CMD_COM_HW    (uint8_t)0xDA // Set COM pins hardware configuration
#define SSD1306_CMD_CONTRAST  (uint8_t)0x81 // Contrast control
#define SSD1306_CMD_EDON      (uint8_t)0xA5 // Entire display ON enabled (all pixels on, RAM content ignored)
#define SSD1306_CMD_EDOFF     (uint8_t)0xA4 // Entire display ON disabled (output follows RAM content)
#define SSD1306_CMD_INV_OFF   (uint8_t)0xA6 // Entire display inversion OFF (normal display)
#define SSD1306_CMD_INV_ON    (uint8_t)0xA7 // Entire display inversion ON (all pixels inverted)
#define SSD1306_CMD_CLOCKDIV  (uint8_t)0xD5 // Set display clock divide ratio/oscillator frequency
#define SSD1306_CMD_CHGPUMP   (uint8_t)0x8D // Charge pump setting
#define SSD1306_CMD_DISP_ON   (uint8_t)0xAF // Display ON
#define SSD1306_CMD_DISP_OFF  (uint8_t)0xAE // Display OFF (sleep mode)
#define SSD1306_CMD_MEM_MODE  (uint8_t)0x20 // Set memory addressing mode
#define SSD1306_CMD_SET_COL   (uint8_t)0x21 // Set column address
#define SSD1306_CMD_SET_PAGE  (uint8_t)0x22 // Set page address
#define SSD1306_CMD_VCOMH     (uint8_t)0xDB // Set Vcomh deselect level
#define SSD1306_CMD_SCRL_HR   (uint8_t)0x26 // Setup continuous horizontal scroll right
#define SSD1306_CMD_SCRL_HL   (uint8_t)0x27 // Setup continuous horizontal scroll left
#define SSD1306_CMD_SCRL_VHR  (uint8_t)0x29 // Setup continuous vertical and horizontal scroll right
#define SSD1306_CMD_SCRL_VHL  (uint8_t)0x2A // Setup continuous vertical and horizontal scroll left
#define SSD1306_CMD_SCRL_STOP (uint8_t)0x2E // Deactivate scroll
#define SSD1306_CMD_SCRL_ACT  (uint8_t)0x2F // Activate scroll
#define SSD1306_SETPRECHARGE  (uint8_t)0xD9 //set the precharge

// Entire display on/off enumeration
enum {
	LCD_ENTIRE_PIXELS_OFF = 0,
	LCD_ENTIRE_PIXELS_ON  = !LCD_ENTIRE_PIXELS_OFF
};

// Display pixels inversion enumeration
enum {
	LCD_INVERT_OFF = 0,
	LCD_INVERT_ON  = !LCD_INVERT_OFF
};

// Display ON/OFF enumeration
enum {
	LCD_OFF = 0,
	LCD_ON  = !LCD_OFF
};

// Screen orientation enumeration
enum {
	LCD_ORIENT_NORMAL = 0, // No rotation
	LCD_ORIENT_CW     = 1, // Clockwise rotation
	LCD_ORIENT_CCW    = 2, // Counter-clockwise rotation
	LCD_ORIENT_180    = 3  // 180 degrees rotation
};

// Screen horizontal scroll direction enumeration
enum {
	LCD_SCROLL_RIGHT = 0, // Scroll right
	LCD_SCROLL_LEFT  = 1  // Scroll left
};

// Screen scroll interval enumeration
enum {
	LCD_SCROLL_IF2   = 0x07, // 2 frames
	LCD_SCROLL_IF3   = 0x04, // 3 frames
	LCD_SCROLL_IF4   = 0x05, // 4 frames
	LCD_SCROLL_IF5   = 0x00, // 5 frames
	LCD_SCROLL_IF25  = 0x06, // 25 frames
	LCD_SCROLL_IF64  = 0x01, // 64 frames
	LCD_SCROLL_IF128 = 0x02, // 128 frames
	LCD_SCROLL_IF256 = 0x03  // 256 frames
};

// Pixel draw mode
enum {
	LCD_PSET = 0x00, // Set pixel
	LCD_PRES = 0x01, // Reset pixel
	LCD_PINV = 0x02  // Invert pixel
};

// Font structure scan lines enumeration
enum {
	FONT_V = (uint8_t)0,        // Vertical font scan lines
	FONT_H = (uint8_t)(!FONT_V) // Horizontal font scan lines
};


// Font descriptor
typedef struct {
	uint8_t font_Width;       // Width of character
	uint8_t font_Height;      // Height of character
	uint8_t font_BPC;         // Bytes for one character
	uint8_t font_Scan;        // Font scan lines behavior
	uint8_t font_MinChar;     // Code of the first known symbol
	uint8_t font_MaxChar;     // Code of the last known symbol
	uint8_t font_UnknownChar; // Code of the unknown symbol
	uint8_t font_Data[];      // Font data
} Font_TypeDef;


// Sequence to set display address [X=0 Y=0]
static const uint8_t SSD1306_SET_ADDR_0x0[] =
{
	COMMAND_MODE,
	SSD1306_CMD_SET_COL,
	0x00,
	SCR_W - 1,
	COMMAND_MODE,
	SSD1306_CMD_SET_PAGE,
	0x00,
	(SCR_H >> 3) - 1
};

// Public variables
extern uint16_t scr_width;
extern uint16_t scr_height;
extern uint8_t LCD_PixelMode;

// Function prototypes
void  SSD1306_Init(I2C_HandleTypeDef hi2c);
void SSD1306_Orientation(I2C_HandleTypeDef hi2c, uint8_t orientation);
void SSD1306_SetXDir(I2C_HandleTypeDef hi2c, uint8_t x_map);
void SSD1306_SetYDir(I2C_HandleTypeDef hi2c, uint8_t y_map);
void SSD1306_Contrast(I2C_HandleTypeDef hi2c, uint8_t contrast);
void SSD1306_Fill(uint8_t pattern);
void SSD1306_Flush(I2C_HandleTypeDef hi2c);
void LCD_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2);


#if (SSD1306_OPT_PIXEL)
inline void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t Mode);
#else
void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t Mode);
#endif

uint16_t LCD_PutStr(uint8_t X, uint8_t Y, uint8_t *str, const Font_TypeDef *Font);
uint8_t LCD_PutChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font);
#endif /* SSD1306_SSD1306_H_ */
