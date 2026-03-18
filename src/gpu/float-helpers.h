#ifndef FLOAT_HELPERS_H
#define FLOAT_HELPERS_H
#include "rpi.h"
#include <stdint.h>

// ex: 
static inline int16_t float_to_fixed12_4(float f) { 
    return (int16_t)(f * 16.0f);
}

static inline int16_t float_to_fixed12_4_round(float f) { 
    return (int16_t) (f * 16.0f + (f >= 0.0f ? 0.5f : -0.5f));
}

static inline float fixed12_4_to_float(int16_t x) { 
    return (float)x / 16.0f;
}

// NV pipeline stores x_s in 16 LSBs, y_s in 16 MSBs
static inline uint32_t pack_xs_ys_fixed12_4(int16_t xs, int16_t ys) { 
    return ((uint32_t)(uint16_t)ys << 16) | (uint32_t)(uint16_t)xs;
}

static inline void unpack_xs_ys_fixed12_4(uint32_t packed, int16_t *xs, int16_t *ys) { 
    *xs = (int16_t)(packed & 0xFFFF);
    *ys = (int16_t)(packed >> 16);
}

#endif
