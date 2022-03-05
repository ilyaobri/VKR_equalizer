#ifndef LCD_CUBE_LCD_H
#define LCD_CUBE_LCD_H

#include "types.h"
#include "main.h"
#include "stm32h7xx_hal.h"

typedef enum {
    LCD_CMD_SLEEP_OUT = 0x11,
    LCD_CMD_GAMMA_SET = 0x26,
    LCD_CMD_DISPLAY_OFF = 0x28,
    LCD_CMD_DISPLAY_ON = 0x29,
    LCD_CMD_COLUMN_ADDRESS_SET = 0x2A,
    LCD_CMD_PAGE_ADDRESS_SET = 0x2B,
    LCD_CMD_MEMORY_WRITE = 0x2C,
    LCD_CMD_COLOR_SET = 0x2D,
    LCD_CMD_MEMORY_READ = 0x2E,
    LCD_CMD_MEMORY_ACCESS_CONTROL = 0x36,
    LCD_CMD_PIXEL_FORMAT_SET = 0x3A,
    LCD_CMD_FRAME_RATE_CONTROL = 0xB1,
    LCD_CMD_DISPLAY_FUNCTION_CONTROL = 0xB6,
    LCD_CMD_POWER_CONTROL_1 = 0xC0,
    LCD_CMD_POWER_CONTROL_2 = 0xC1,
    LCD_CMD_VCOM_CONTROL_1 = 0xC5,
    LCD_CMD_VCOM_CONTROL_2 = 0xC7,
    LCD_CMD_POWER_CONTROL_A = 0xCB,
    LCD_CMD_POWER_CONTROL_B = 0xCF,
    LCD_CMD_READ_ID4 = 0xD3,
    LCD_CMD_READ_ID1 = 0xDA,
    LCD_CMD_READ_ID2 = 0xDB,
    LCD_CMD_READ_ID3 = 0xDC,
    LCD_CMD_POSITIVE_GAMMA_CORRECTION = 0xE0,
    LCD_CMD_NEGATIVE_GAMMA_CORRECTION = 0xE1,
    LCD_CMD_DRIVER_TIMING_CONTROL_A0 = 0xE8,
    LCD_CMD_DRIVER_TIMING_CONTROL_A1 = 0xE9,
    LCD_CMD_DRIVER_TIMING_CONTROL_B = 0xEA,
    LCD_CMD_POWER_ON_SEQUENCE_CONTROL = 0xED,
    LCD_CMD_ENABLE_3G = 0xF2,
    LCD_CMD_PUMP_RATIO_CONTROL = 0xF7
} lcd_cmd_t;

typedef struct {
    vu16 *cmd;
    vu16 *data;
} lcd_regs_t;

typedef struct {
    GPIO_TypeDef* port;
    u16 pin;
} lcd_led_t;

typedef struct {
    lcd_regs_t regs;
    lcd_led_t led;

    u32 width;
    u32 height;
    u32 direction;

    u32 columns;
    u32 pages;
} lcd_t;

typedef struct {
    u8 ic_version;
    u16 ic_model;
} lcd_id4_t;

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

void lcd_config_regs(lcd_t *lcd, u32 base, u8 dc_pin);

void lcd_config_led(lcd_t *lcd, GPIO_TypeDef* led_port, u16 led_pin);

void lcd_config_size(lcd_t *lcd, u32 width, u32 height, u8 direction);

void lcd_cmd_read_id4(lcd_t *lcd, lcd_id4_t *id4);

void lcd_frame(lcd_t *lcd, u16 *frame);

void lcd_fill(lcd_t *lcd, u16 color);

void lcd_init(lcd_t *lcd);

void lcd_led(lcd_t *lcd, u32 state);

#endif //LCD_CUBE_LCD_H
