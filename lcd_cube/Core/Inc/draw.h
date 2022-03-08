//
// Created by Alexei Gladkikh on 07.03.2022.
//

#ifndef LCD_CUBE_DRAW_H
#define LCD_CUBE_DRAW_H

#include <assert.h>
#include "types.h"
#include "font.h"
#include "color.h"

typedef struct {
    u16* canvas;
    u32 width;
    u32 height;
    u32 total_points;
} frame_t;

static inline u32 frame_calc_offset(frame_t *frame, u32 y) {
    return y * frame->width;
}

static inline u32 frame_calc_index(frame_t *frame, u32 x, u32 y) {
    return frame_calc_offset(frame, y) + x;
}

static inline void frame_set_by_xy(frame_t *frame, u32 x, u32 y, color_t color) {
    assert(x < frame->width);
    assert(y < frame->height);

    u32 index = frame_calc_index(frame, x, y);
    frame->canvas[index] = color;
}

static inline void frame_set_by_index(frame_t *frame, u32 index, color_t color) {
    frame->canvas[index] = color;
}

void frame_init(frame_t *frame, u16 *canvas, u32 width, u32 height);

void frame_draw_rect(frame_t *frame, color_t color, u32 x0, u32 y0, u32 x1, u32 y1);

void frame_fill(frame_t *frame, color_t color);

void frame_draw_char(frame_t *frame, color_t color, u32 x, u32 y, font_size_t size, char ch);

void frame_draw_string(frame_t *frame, color_t color, u32 x, u32 y, font_size_t size, string str);

void frame_printf_string(frame_t *frame, color_t color, u32 x, u32 y, font_size_t size, string format, ...);

//void frame_draw_bar(frame_t *frame, color_t color, u32 x, u32 y, u32 width, u32 height, u32 percent);

#endif //LCD_CUBE_DRAW_H
