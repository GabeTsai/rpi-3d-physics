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

static void printk_float(const char *name, float v) {
    // replace direct pointer cast with union to avoid compiler warnings 
    union { float f; unsigned u; } _pun = { .f = v }; 
    unsigned u = _pun.u;
    unsigned absu = u & 0x7fffffff;
    int sign = (u >> 31) & 1;

    if ((absu & 0x7f800000) == 0x7f800000) {
        if (absu & 0x007fffff)
            printk("%s = NaN (0x%x)\n", name, u);
        else
            printk("%s = %sINF\n", name, sign ? "-" : "+");
        return;
    }

    if (absu == 0) {
        printk("%s = %s0.000000e+00\n", name, sign ? "-" : "");
        return;
    }

    if (sign) v = -v;

    /*
     * Scale v into [1, 10).  Both loops are bounded:
     *   - scale-down: v starts finite and halves each step, terminates in < 40 iters.
     *   - scale-up:   guard `v > 0.0f` prevents an infinite loop when FTZ mode
     *                 flushes a subnormal * 10 to zero.  If that happens we bail
     *                 out with exp10 already reflecting the exponent, and whole=0
     *                 which prints as "0.000000e-XX" — ugly but not a hang.
     */
     
    int exp10 = 0;
    int scale_iters = 0;
    while (v >= 10.0f && scale_iters < 50) { v *= 0.1f; exp10++; scale_iters++; }
    while (v < 1.0f  && v > 0.0f && scale_iters < 50) { v *= 10.0f; exp10--; scale_iters++; }

    int whole = (int)v;
    if (whole < 0) whole = 0;
    if (whole > 9) whole = 9;
    float frac = v - (float)whole;
    if (frac < 0.0f) frac = 0.0f;

    printk("%s = ", name);
    if (sign) printk("-");
    printk("%d.", whole);

    for (int i = 0; i < 6; i++) {
        frac *= 10.0f;
        int d = (int)frac;
        if (d < 0) d = 0;
        if (d > 9) d = 9;
        printk("%d", d);
        frac -= (float)d;
        if (frac < 0.0f) frac = 0.0f;
    }

    printk("e");
    if (exp10 >= 0) printk("+");
    else { printk("-"); exp10 = -exp10; }

    if (exp10 >= 100) {
        int h = 0; while (exp10 >= 100) { exp10 -= 100; h++; }
        int t = 0; while (exp10 >= 10)  { exp10 -= 10;  t++; }
        printk("%d%d%d\n", h, t, exp10);
    } else {
        int t = 0; while (exp10 >= 10) { exp10 -= 10; t++; }
        printk("%d%d\n", t, exp10);
    }
}

#endif
