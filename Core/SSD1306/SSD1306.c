#include "SSD1306.h"

// Screen dimensions
uint16_t scr_width  = SCR_W;
uint16_t scr_height = SCR_H;

// Pixel drawing mode
// Whereas in most drawing operations pixels are set, use global variable to select drawing mode
// instead of passing set/reset/invert mode in each call of drawing functions
uint8_t LCD_PixelMode = LCD_PSET;

// Display image orientation
static uint8_t scr_orientation = LCD_ORIENT_NORMAL;

// Video RAM buffer
static uint8_t vRAM[(SCR_W * SCR_H) >> 3] __attribute__((aligned(4)));

// Vertical line drawing look up table for first byte
static const uint8_t LUT_FB[] = { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };

// Vertical line drawing look up table for last byte
static const uint8_t LUT_LB[] = { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };

static void SSD1306_cmd_single(I2C_HandleTypeDef hi2c, uint8_t cmd1)
{
	uint8_t buff[] = {COMMAND_MODE, cmd1};
	HAL_I2C_Master_Transmit(&hi2c, SSD1306ADDRESS, buff, sizeof(buff), 100);
}

static void SSD1306_cmd_double(I2C_HandleTypeDef hi2c, uint8_t cmd1, uint8_t cmd2)
{
	uint8_t buff[] = {COMMAND_MODE, cmd1, cmd2};
	HAL_I2C_Master_Transmit(&hi2c, SSD1306ADDRESS, buff, sizeof(buff), 100);
}

//init the display
void SSD1306_Init(I2C_HandleTypeDef hi2c)
{
	SSD1306_cmd_single(hi2c, SSD1306_CMD_DISP_OFF);
	SSD1306_cmd_double(hi2c, SSD1306_CMD_CLOCKDIV, 0x80);
	SSD1306_cmd_double(hi2c, SSD1306_CMD_SETMUX, 0x3F);
	SSD1306_cmd_double(hi2c, SSD1306_CMD_SETOFFS, 0x00);
	SSD1306_cmd_single(hi2c, SSD1306_CMD_STARTLINE | 0x00);
	SSD1306_cmd_double(hi2c, SSD1306_CMD_CHGPUMP, ((0x2 == 0x1) ? 0x10 : 0x14));
	SSD1306_cmd_double(hi2c, SSD1306_CMD_MEM_MODE, 0x00);
	SSD1306_cmd_single(hi2c, SSD1306_CMD_SEG_NORM | 0x1);
	SSD1306_cmd_single(hi2c, SSD1306_CMD_COM_INV);
	SSD1306_cmd_double(hi2c, SSD1306_CMD_COM_HW, 64 == 32 ? 0x02 : 0x12);
	SSD1306_cmd_double(hi2c, SSD1306_CMD_CONTRAST, 64 == 32 ? 0x8F : ((0x2 == 0x1) ? 0x9F : 0xCF));
	SSD1306_cmd_double(hi2c, SSD1306_SETPRECHARGE, (0x2 == 0x1) ? 0x22 : 0xF1);
	SSD1306_cmd_double(hi2c, SSD1306_CMD_VCOMH, 0x40);
	SSD1306_cmd_single(hi2c, SSD1306_CMD_EDOFF);
	SSD1306_cmd_single(hi2c, SSD1306_CMD_INV_OFF);
	SSD1306_cmd_single(hi2c, SSD1306_CMD_DISP_ON);
}

// Set display orientation
// input:
//   orientation - new display orientation (one of LCD_ORIENT_XXX values)
// note: normal orientation is FPC on top of COG
// note: this setting specifies an orientation of display, not orientation of image
void SSD1306_Orientation(I2C_HandleTypeDef hi2c, uint8_t orientation) {
	// Configure display SEG/COM scan direction
	switch(orientation) {
		case LCD_ORIENT_CW:
			// Clockwise rotation
			scr_width  = SCR_H;
			scr_height = SCR_W;
			SSD1306_SetXDir(hi2c, LCD_INVERT_ON);
			SSD1306_SetYDir(hi2c, LCD_INVERT_OFF);
			break;
		case LCD_ORIENT_CCW:
			// Counter-clockwise rotation
			scr_width  = SCR_H;
			scr_height = SCR_W;
			SSD1306_SetXDir(hi2c, LCD_INVERT_OFF);
			SSD1306_SetYDir(hi2c, LCD_INVERT_ON);
			break;
		case LCD_ORIENT_180:
			// 180 degree rotation
			scr_width  = SCR_W;
			scr_height = SCR_H;
			SSD1306_SetXDir(hi2c, LCD_INVERT_OFF);
			SSD1306_SetYDir(hi2c, LCD_INVERT_OFF);
			break;
		default:
			// Normal orientation
			scr_width  = SCR_W;
			scr_height = SCR_H;
			SSD1306_SetXDir(hi2c, LCD_INVERT_ON);
			SSD1306_SetYDir(hi2c, LCD_INVERT_ON);
			break;
	}

	// Store orientation
	scr_orientation = orientation;
}

// Set X coordinate mapping (normal or mirrored)
// input:
//   x_map - new mapping of X coordinate (one of LCD_INVERT_XXX values)
// note: LCD_INVERT_OFF means normal SEG scan direction
// note: new setting will only affect subsequent data output
void SSD1306_SetXDir(I2C_HandleTypeDef hi2c, uint8_t x_map)
{
	uint8_t i2cData2[2];

	i2cData2[0] = 0x00;
	i2cData2[1] = (x_map ? SSD1306_CMD_SEG_INV : SSD1306_CMD_SEG_NORM);
	HAL_I2C_Master_Transmit(&hi2c, SSD1306ADDRESS, i2cData2, sizeof(i2cData2[0]), 100);
}

// Set Y coordinate mapping (normal or mirrored)
// input:
//   y_map - new mapping of Y coordinate (one of LCD_INVERT_XXX values)
// note: LCD_INVERT_OFF means normal COM scan direction
// note: new setting flip screen image immediately
void SSD1306_SetYDir(I2C_HandleTypeDef hi2c, uint8_t y_map)
{
	uint8_t i2cData2[2];

	i2cData2[0] = 0x00;
	i2cData2[1] = (y_map ? SSD1306_CMD_COM_INV : SSD1306_CMD_COM_NORM);
	HAL_I2C_Master_Transmit(&hi2c, SSD1306ADDRESS, i2cData2, sizeof(i2cData2[0]), 100);
}

void SSD1306_Contrast(I2C_HandleTypeDef hi2c, uint8_t contrast)
{
	uint8_t i2cData2[3];

	i2cData2[0] = 0x00;
	i2cData2[1] = SSD1306_CMD_CONTRAST;
	i2cData2[2] = contrast;
	HAL_I2C_Master_Transmit(&hi2c, SSD1306ADDRESS, i2cData2, sizeof(i2cData2), 100);
}

void SSD1306_Fill(uint8_t pattern)
{
	uint16_t i;

	for (i = (SCR_W * SCR_H) >> 3; i--; )
		vRAM[i] = pattern;
}

uint16_t LCD_PutStr(uint8_t X, uint8_t Y, uint8_t *str, const Font_TypeDef *Font)
{
	uint8_t pX = X;
	uint8_t eX = scr_width - Font->font_Width - 1;

	while (*str) {
		pX += LCD_PutChar(pX,Y,*str++,Font);
		if (pX > eX) break;
	}

	return (pX - X);
}

uint8_t LCD_PutChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font)
{
	uint8_t pX;
	uint8_t pY;
	uint8_t tmpCh;
	uint8_t bL;
	const uint8_t *pCh;

	// If the specified character code is out of bounds should substitute the code of the "unknown" character
	if (Char < Font->font_MinChar || Char > Font->font_MaxChar) Char = Font->font_UnknownChar;

	// Pointer to the first byte of character in font data array
	pCh = &Font->font_Data[(Char - Font->font_MinChar) * Font->font_BPC];

	// Draw character
	if (Font->font_Scan == FONT_V) {
		// Vertical pixels order
		if (Font->font_Height < 9) {
			// Height is 8 pixels or less (one byte per column)
			pX = X;
			while (pX < X + Font->font_Width) {
				pY = Y;
				tmpCh = *pCh++;
				while (tmpCh) {
					if (tmpCh & 0x01) LCD_Pixel(pX,pY,LCD_PixelMode);
					tmpCh >>= 1;
					pY++;
				}
				pX++;
			}
		} else {
			// Height is more than 8 pixels (several bytes per column)
			pX = X;
			while (pX < X + Font->font_Width) {
				pY = Y;
				while (pY < Y + Font->font_Height) {
					bL = 8;
					tmpCh = *pCh++;
					if (tmpCh) {
						while (bL) {
							if (tmpCh & 0x01) LCD_Pixel(pX,pY,LCD_PixelMode);
							tmpCh >>= 1;
							if (tmpCh) {
								pY++;
								bL--;
							} else {
								pY += bL;
								break;
							}
						}
					} else {
						pY += bL;
					}
				}
				pX++;
			}
		}
	} else {
		// Horizontal pixels order
		if (Font->font_Width < 9) {
			// Width is 8 pixels or less (one byte per row)
			pY = Y;
			while (pY < Y + Font->font_Height) {
				pX = X;
				tmpCh = *pCh++;
				while (tmpCh) {
					if (tmpCh & 0x01) LCD_Pixel(pX,pY,LCD_PixelMode);
					tmpCh >>= 1;
					pX++;
				}
				pY++;
			}
		} else {
			// Width is more than 8 pixels (several bytes per row)
			pY = Y;
			while (pY < Y + Font->font_Height) {
				pX = X;
				while (pX < X + Font->font_Width) {
					bL = 8;
					tmpCh = *pCh++;
					if (tmpCh) {
						while (bL) {
							if (tmpCh & 0x01) LCD_Pixel(pX,pY,LCD_PixelMode);
							tmpCh >>= 1;
							if (tmpCh) {
								pX++;
								bL--;
							} else {
								pX += bL;
								break;
							}
						}
					} else {
						pX += bL;
					}
				}
				pY++;
			}
		}
	}

	return Font->font_Width + 1;
}

#if (SSD1306_OPT_PIXEL)
__attribute__((always_inline)) void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t Mode) {
#else
void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t Mode) {
#endif // SSD1306_OPT_PIXEL
	register uint32_t offset;
	register uint32_t bpos;

	// Offset of pixel in the vRAM array must be computed by formula ((Y >> 3) * SCR_W) + X
	// Since screen is 128 pixel width the formula can be simplified to ((Y >> 3) << 7) + X
	// For 90 degree rotation X and Y must be swapped
	if (scr_orientation == LCD_ORIENT_CW || scr_orientation == LCD_ORIENT_CCW) {
		offset = ((X >> 3) << 7) + Y;
		bpos   = X & 0x07;
	} else {
		offset = ((Y >> 3) << 7) + X;
		bpos   = Y & 0x07;
	}

	// Return if offset went out outside of vRAM
	if (offset > ((SCR_W * SCR_H) >> 3)) {
		return;
	}

#if (SSD1306_USE_BITBAND)
	switch (Mode) {
		case LCD_PRES:
			*(uint32_t *)(SRAM_BB_BASE + (((uint32_t)((void *)(&vRAM[offset])) - SRAM_BASE) << 5) + (bpos << 2))  = 0;
			break;
		case LCD_PINV:
			*(uint32_t *)(SRAM_BB_BASE + (((uint32_t)((void *)(&vRAM[offset])) - SRAM_BASE) << 5) + (bpos << 2)) ^= 1;
			break;
		default:
			*(uint32_t *)(SRAM_BB_BASE + (((uint32_t)((void *)(&vRAM[offset])) - SRAM_BASE) << 5) + (bpos << 2))  = 1;
			break;
	}
#else // (SSD1306_USE_BITBAND)
	switch (Mode) {
		case LCD_PRES:
			vRAM[offset] &= ~(1 << bpos);
			break;
		case LCD_PINV:
			vRAM[offset] ^=  (1 << bpos);
			break;
		default:
			vRAM[offset] |=  (1 << bpos);
			break;
	}
#endif // SSD1306_USE_BITBAND
}

// Send vRAM buffer into display
void SSD1306_Flush(I2C_HandleTypeDef hi2c)
{
	uint8_t buffInit[] = {COMMAND_MODE, SSD1306_CMD_SET_COL, 0x00, SCR_W - 1};
	HAL_I2C_Master_Transmit(&hi2c, SSD1306ADDRESS, buffInit, sizeof(buffInit), 100);
	buffInit[0] = COMMAND_MODE;
	buffInit[1] = SSD1306_CMD_SET_PAGE;
	buffInit[2] = 0x00;
	buffInit[3] = ((SCR_H >> 3) - 1);
	HAL_I2C_Master_Transmit(&hi2c, SSD1306ADDRESS, buffInit, sizeof(buffInit), 100);

	SSD1306_cmd_single(hi2c, SSD1306_CMD_EDOFF);

	uint8_t buff[17];
	buff[0] = 0x40; // Data Mode

	uint16_t vRAMSize = sizeof(vRAM);
	// send display buffer in 16 byte chunks
	for(uint16_t i = 0; i < vRAMSize; i += 16 )
	{
		uint8_t x;

		// TODO - this will segfault if buffer.size() % 16 != 0
		for(x = 1; x < sizeof(buff); x++)
			buff[x] = vRAM[i+x-1];

		HAL_I2C_Master_Transmit(&hi2c, SSD1306ADDRESS, buff, sizeof(buff), 100);
	}
}

static void LCD_VLineInt(uint8_t X, uint8_t Y, uint8_t H)
{
	uint8_t *ptr;
	uint8_t mask;
	uint8_t modulo;

	// Pointer to the first byte of line in video buffer
	// This is optimized formula, original is "((Y >> 3) * SCR_W) + X"
	ptr = &vRAM[((Y >> 3) << 7)] + X;

	// First partial byte?
	modulo = (Y & 0x07);
	if (modulo) {
		// Get bit mask for first partial byte from lookup table
		modulo = 8 - modulo;
		mask = LUT_FB[modulo];

		// Trim mask if line is will not go out from a current byte
		if (modulo > H) mask &= (0xFF >> (modulo - H));

		// Modify bits in first byte of line
		switch (LCD_PixelMode) {
			case LCD_PRES:
				*ptr &= ~mask;
				break;
			case LCD_PINV:
				*ptr ^=  mask;
				break;
			default:
				*ptr |=  mask;
				break;
		}

		// Return if line is over
		if (modulo > H) return;

		// Shift pointer to the next byte in line and decrease line height counter
		ptr += SCR_W;
		H   -= modulo;
	}

	// Fill solid bytes
	if (H > 7) {
		// Separate cycle for each case of pixel mode (to improve performance)
		switch (LCD_PixelMode) {
			case LCD_PRES:
				do {
					*ptr = 0x00;
					ptr += SCR_W;
					H   -= 8;
				} while (H > 7);
				break;
			case LCD_PINV:
				do {
					*ptr = ~(*ptr);
					ptr += SCR_W;
					H   -= 8;
				} while (H > 7);
				break;
			default:
				do {
					*ptr = 0xFF;
					ptr += SCR_W;
					H   -= 8;
				} while (H > 7);
				break;
		}
	}

	// Last partial byte?
	if (H) {
		// Get bit mask for last partial byte from lookup table
		modulo = (H & 0x07);
		mask   = LUT_LB[modulo];

		// Modify bits in last byte of line
		switch (LCD_PixelMode) {
			case LCD_PRES:
				*ptr &= ~mask;
				break;
			case LCD_PINV:
				*ptr ^=  mask;
				break;
			default:
				*ptr |=  mask;
				break;
		}
	}
}

// Draw filled rectangle
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
void LCD_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2)
{
	uint8_t Z,E,T,L;

	// Fill rectangle by vertical lines is most optimal, therefore calculate coordinates
	// with regard of screen rotation
	if (scr_orientation == LCD_ORIENT_CW || scr_orientation == LCD_ORIENT_CCW) {
		if (X1 > X2) {
			T = X2; L = X1 - X2;
		} else {
			T = X1; L = X2 - X1;
		}

		if (Y1 > Y2) {
			Z = Y1; E = Y2;
		} else {
			Z = Y2; E = Y1;
		}
	} else {
		if (Y1 > Y2) {
			T = Y2; L = Y1 - Y2;
		} else {
			T = Y1; L = Y2 - Y1;
		}

		if (X1 > X2) {
			Z = X1; E = X2;
		} else {
			Z = X2; E = X1;
		}
	}
	L++;

	// Fill a rectangle
	do {
		LCD_VLineInt(Z,T,L);
	} while (Z-- > E);
}
