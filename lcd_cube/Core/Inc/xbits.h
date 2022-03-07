//
// Created by Alexei Gladkikh on 06.03.2022.
//

#ifndef LCD_CUBE_XBITS_H
#define LCD_CUBE_XBITS_H

#include <limits.h>
#include "types.h"

static inline u32 ushr(u32 value, u8 count) {
    return value >> count;
}

static inline u32 shl(u32 value, u8 count) {
    return value << count;
}

static inline u8 xbit_u32(u32 value, u8 index) {
    return ushr(value, index) & 1u;
}

static inline u32 xbits_u32(u32 value, u8 high, u8 low) {
    return ushr(value, low) & (shl(1u, high - low + 1u) - 1u);
}

static inline u32 bitmask_32(u8 high, u8 low) {
    return ushr(UINT32_MAX, 31 - high) & ~ushr(UINT32_MAX, 32 - low);
}

static inline u32 bzero_u32(u32 value, u8 high, u8 low) {
    return value & ~bitmask_32(high, low);
}

static inline u32 mask_u32(u32 value, u8 high, u8 low) {
    return value & bitmask_32(high, low);
}

static inline void insert_u32(u32* dst, u32 src, u8 high, u8 low) {
    u32 shifted = shl(src, low);
    u32 masked = mask_u32(shifted, high, low);
    u32 pattern = bzero_u32(*dst, high, low);
    *dst = pattern | masked;
}

#define MODIND(array, index, size) array[(index) % (size)]

#define MAX_OF(a, b) ((a) < (b)) ? (b) : (a)
#define MIN_OF(a, b) ((a) > (b)) ? (b) : (a)

#endif //LCD_CUBE_XBITS_H
