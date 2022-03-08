//
// Created by Alexei Gladkikh on 08.03.2022.
//

#ifndef LCD_CUBE_COLOR_H
#define LCD_CUBE_COLOR_H

typedef enum {
    WHITE = 0xFFFF,
    BLACK = 0x0000,
    BLUE = 0x001F,
    BRED = 0xF81F,
    GRED = 0xFFE0,
    GBLUE = 0x07FF,
    RED = 0xF800,
    MAGENTA = 0xF81F,
    GREEN = 0x07E0,
    CYAN = 0x7FFF,
    YELLOW = 0xFFE0,
    BROWN = 0xBC40,
    BRRED = 0xFC07,
    GRAY = 0x8430,
    DARKBLUE = 0x01CF,
    LIGHTBLUE = 0x7D7C,
    GRAYBLUE = 0x5458,
    LIGHTGREEN = 0x841F,
    LIGHTGRAY = 0xEF5B,
    LGRAY = 0xC618,
    LGRAYBLUE = 0xA651,
    LBBLUE = 0x2B12,
} color_t;

#endif //LCD_CUBE_COLOR_H
