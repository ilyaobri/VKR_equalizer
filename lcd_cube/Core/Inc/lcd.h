#ifndef LCD_CUBE_LCD_H
#define LCD_CUBE_LCD_H

#include "types.h"
#include "xbits.h"
#include "main.h"
#include "color.h"
#include "stm32h7xx_hal.h"

typedef enum {
    LCD_CMD_READ_DISPLAY_STATUS = 0x09,
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
    GPIO_TypeDef *port;
    u16 pin;
} lcd_led_t;

typedef volatile enum {
    LCD_STATE_IDLE = 0,
    LCD_STATE_BUSY = 1,
    LCD_STATE_ERROR = 2
} lcd_state_t;

typedef struct {
    lcd_regs_t regs;
    lcd_led_t led;

    DMA_HandleTypeDef *dma;

    u32 width;
    u32 height;
    u32 direction;

    u32 columns;
    u32 pages;

    volatile lcd_state_t state;
} lcd_t;

typedef struct {
    u8 ic_version;
    u16 ic_model;
} lcd_id4_t;

typedef struct {
    u8 gamma_curve;
    u8 display_on;
    u8 all_pixel_off;
    u8 all_pixel_on;
    u8 inversion_status;
    u8 vertical_scrolling;
    u8 normal_mode;
    u8 sleep_out;
    u8 partial_mode;
    u8 idle_mode;
    u8 color_pixel_format;
    u8 horizontal_refresh_order;
    u8 rgb_bgr_order;
    u8 vertical_refresh;
    u8 row_column_exchange;
    u8 column_address_order;
    u8 row_address_order;
    u8 booster_voltage;
} lcd_status_t;

#define LCD_STATUS_BOOSTER_VOLTAGE(status) xbit_u32(status, 31)
#define LCD_STATUS_ROW_ADDRESS_ORDER(status) xbit_u32(status, 30)
#define LCD_STATUS_COLUMN_ADDRESS_ORDER(status) xbit_u32(status, 29)
#define LCD_STATUS_ROW_COLUMN_EXCHANGE(status) xbit_u32(status, 28)
#define LCD_STATUS_VERTICAL_REFRESH(status) xbit_u32(status, 27)
#define LCD_STATUS_RGB_BGR_ORDER(status) xbit_u32(status, 26)
#define LCD_STATUS_HORIZONTAL_REFRESH_ORDER(status) xbit_u32(status, 25)
#define LCD_STATUS_COLOR_PIXEL_FORMAT(status) xbits_u32(status, 22, 20)
#define LCD_STATUS_IDLE_MODE(status) xbit_u32(status, 19)
#define LCD_STATUS_PARTIAL_MODE(status) xbit_u32(status, 18)
#define LCD_STATUS_SLEEP(status) xbit_u32(status, 17)
#define LCD_STATUS_NORMAL_MODE(status) xbit_u32(status, 16)
#define LCD_STATUS_VERTICAL_SCROLLING(status) xbit_u32(status, 14)
#define LCD_STATUS_INVERSION_STATUS(status) xbit_u32(status, 13)
#define LCD_STATUS_ALL_PIXEL_ON(status) xbit_u32(status, 12)
#define LCD_STATUS_ALL_PIXEL_OFF(status) xbit_u32(status, 11)
#define LCD_STATUS_DISPLAY_ON(status) xbit_u32(status, 10)
#define LCD_STATUS_GAMMA_CURVE(status) xbits_u32(status, 8, 6)

void lcd_config_dma(lcd_t *lcd, DMA_HandleTypeDef *dma);

void lcd_config_regs(lcd_t *lcd, u32 base, u8 dc_pin);

void lcd_config_led(lcd_t *lcd, GPIO_TypeDef *led_port, u16 led_pin);

void lcd_config_size(lcd_t *lcd, u32 width, u32 height, u8 direction);

void lcd_cmd_read_id4(lcd_t *lcd, lcd_id4_t *id4);

u32 lcd_cmd_read_display_status(lcd_t *lcd);

void lcd_parse_status(u32 status, lcd_status_t *lcd_status);

void lcd_wait_idle(lcd_t *lcd);

void lcd_frame_dma(lcd_t *lcd, u16 *frame);

void lcd_frame(lcd_t *lcd, u16 *frame);

void lcd_fill(lcd_t *lcd, color_t color);

void lcd_init(lcd_t *lcd, lcd_id4_t *id, lcd_status_t *status);

void lcd_led(lcd_t *lcd, u32 state);

#endif //LCD_CUBE_LCD_H
