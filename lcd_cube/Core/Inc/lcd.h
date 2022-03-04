#ifndef LCD_CUBE_LCD_H
#define LCD_CUBE_LCD_H

#include "types.h"
#include "main.h"
#include "stm32h7xx_hal.h"

#define LCD_SETX_CMD 0x2A
#define LCD_SETY_CMD 0x2B
#define LCD_WRAM_CMD 0x2C
#define LCD_RRAM_CMD 0x2E

#define USE_HORIZONTAL     0

#define LCD_WIDTH 240
#define LCD_HEIGHT 320

#define LCD_REGION_NUMBER  MPU_REGION_NUMBER0
#define LCD_ADDRESS_START  0x60000000
#define LCD_REGION_SIZE    MPU_REGION_SIZE_256MB

typedef struct {
    vu16 LCD_REG;
    vu16 LCD_RAM;
} LCD_TypeDef;

#define LCD_BASE ((u32)(LCD_ADDRESS_START | 0x0007FFFE))
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


#define LCD_LED_CLR() HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_RESET)
#define LCD_LED_SET() HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_SET)

static inline void LCD_WR_REG(u16 data) {
    LCD->LCD_REG = data;
}

static inline void LCD_WR_DATA(u16 data) {
    LCD->LCD_RAM = data;
}

static inline vu16 LCD_RD_DATA(void) {
    return LCD->LCD_RAM;
}

static inline void LCD_ReadReg(u16 LCD_Reg, u8 *buffer, s32 size) {
    LCD_WR_REG(LCD_Reg);
    for (s32 k = 0; k < size; k++) {
        buffer[k] = LCD_RD_DATA();
        HAL_Delay(1);  // 1 ms
    }
}

static inline u16 LCD_Read_ID(void) {
    u8 val[4] = {0};
    LCD_ReadReg(0xD3, val, sizeof(val));
    return (val[2] << 8) | val[3];
}

void LCD_direction(u8 direction);
void LCD_SetWindows(u16 x_start, u16 y_start, u16 x_end, u16 y_end);
void LCD_Clear(u16 Color);
void LCD_Init();

#endif //LCD_CUBE_LCD_H
