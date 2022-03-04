#ifndef LCD_CUBE_LCD_H
#define LCD_CUBE_LCD_H

#include "types.h"

#define USE_HORIZONTAL     0

#define LCD_W 240
#define LCD_H 320

//extern u16  POINT_COLOR;
//extern u16  BACK_COLOR;

#define LCD_REGION_NUMBER        MPU_REGION_NUMBER0
#define LCD_ADDRESS_START        0x60000000
#define LCD_REGION_SIZE            MPU_REGION_SIZE_256MB

#define LCD_LED   5

#define LCD_LED_CLR() HAL_GPIO_WritePin(GPIOB, 1 << LCD_LED, GPIO_PIN_RESET)

typedef struct {
    vu16 LCD_REG;
    vu16 LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE ((u32)(0x60000000 | 0x0007FFFE))
#define LCD ((LCD_TypeDef *) LCD_BASE)

#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define BRED        0xF81F
#define GRED        0xFFE0
#define GBLUE       0x07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN       0xBC40
#define BRRED       0xFC07
#define GRAY        0x8430
#define DARKBLUE    0x01CF
#define LIGHTBLUE   0x7D7C
#define GRAYBLUE    0x5458
#define LIGHTGREEN  0x841F
#define LIGHTGRAY   0xEF5B
#define LGRAY       0xC618
#define LGRAYBLUE   0xA651
#define LBBLUE      0x2B12

//void LCD_Init(void);
//
//u16 LCD_read(void);
//
//void LCD_Clear(u16 Color);
//
//void LCD_SetCursor(u16 Xpos, u16 Ypos);
//
//void LCD_DrawPoint(u16 x, u16 y);//����
//u16 LCD_ReadPoint(u16 x, u16 y); //����
//void LCD_SetWindows(u16 xStar, u16 yStar, u16 xEnd, u16 yEnd);
//
//u16 LCD_RD_DATA(void);//��ȡLCD����
//void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue);
//
//void LCD_WR_REG(u16 data);
//
//void LCD_WR_DATA(u16 data);
//
//void LCD_ReadReg(u16 LCD_Reg, u8 *Rval, int n);
//
//void LCD_WriteRAM_Prepare(void);
//
//void LCD_ReadRAM_Prepare(void);
//
//void Lcd_WriteData_16Bit(u16 Data);
//
//u16 Lcd_ReadData_16Bit(void);
//
//void LCD_direction(u8 direction);
//
//u16 Color_To_565(u8 r, u8 g, u8 b);
//
//u16 LCD_Read_ID(void);

#endif //LCD_CUBE_LCD_H
