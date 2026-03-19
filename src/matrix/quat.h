#pragma once

#include "rpi.h"
#include "rpi-math.h"
#include "vec3.h"

typedef struct __attribute__((aligned(8))) {
    float w, x, y, z;
} quat;


static inline quat quat_make(float w, float x, float y, float z) {
    quat q = {w, x, y, z};
    return q;
}

static inline quat quat_identity(void) {
    quat q = {1.0f, 0.0f, 0.0f, 0.0f};
    return q;
}

static inline quat quat_add(quat a, quat b) {
    quat r = {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
    return r;
}

static inline quat quat_sub(quat a, quat b) {
    quat r = {a.w - b.w, a.x - b.x, a.y - b.y, a.z - b.z};
    return r;
}

static inline quat quat_scale(quat q, float s) {
    quat r = {q.w * s, q.x * s, q.y * s, q.z * s};
    return r;
}

static inline unsigned fbits(float x) {
    union {
        float f;
        unsigned u;
    } t;
    t.f = x;
    return t.u;
}

static inline int float_is_nan(float x) {
    unsigned u = *(unsigned *)&x;
    return ((u & 0x7f800000) == 0x7f800000) &&  // exponent all 1s
           ((u & 0x007fffff) != 0);             // mantissa non-zero
}

static __attribute__((noinline)) float quat_dot(const quat *a, const quat *b) {
    // printk("QD0 a=%x b=%x\n", (unsigned)a, (unsigned)b);
    volatile float sink1 = a->w;
    // printk("QD1\n");
    volatile float sink2 = a->x;
    // printk("QD2\n");
    volatile float sink3 = a->y;
    // printk("QD3\n");
    volatile float sink4 = a->z;
    // printk("QD4\n");
    return sink1 * b->w + sink2 * b->x + sink3 * b->y + sink4 * b->z;
}
// static inline float quat_dot(quat a, quat b) {
//     float inta = a.w * b.w;
//     float intb = a.x * b.x;
//     float intc = a.y * b.y;

//     // printk("D0\n");
//     // printk("addr a.z = %x\n", (unsigned)&a.z);
//     // printk("a=%x &a->z=%x offset=%d\n",
//     //    (unsigned)&a,
//     //    (unsigned)&a.z,
//     //    (int)((char*)&a.z - (char*)&a));

//     volatile float sink1, sink2, sink3, sink4;

// printk("T0\n");
// sink1 = a.w;
// printk("T1\n");
// sink2 = a.x;
// printk("T2\n");
// sink3 = a.y;
// printk("T3\n");
// sink4 = a.z;
// printk("T4\n");

// if ((sink1 || sink2 || sink3 || sink4) == 0){panic("foisd");}
// float az = a.z;
// // printk("az bits=%x\n", fbits(az));    // printk("D1 az=%f\n", az);

//     float bz = b.z;
//     // printk("D2 bz=%f\n", bz);

//     float intd = az * bz;
//     // printk("E intd=%f\n", intd);

//     return inta + intb + intc + intd;
// }

static inline float quat_norm_sq(quat q) {
    // printk("quat_norm_sq\n");
    return quat_dot(&q, &q);
}

static void printk_float(const char *name, float v) {
    unsigned u = *(unsigned *)&v;
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


static inline float quat_norm(quat q) {
    // printk("quat_norm\n");
    float norm_sq = quat_norm_sq(q);
    // printk_float("norm_sq", norm_sq);
    if (norm_sq <= 1e-2f) return 0.0f;
    return sqrtf(norm_sq);
}

static inline quat quat_normalize(quat q) {
    // printk("a\n");
    float n = quat_norm(q);
    // printk("b\n");
    if (n <= 1e-8f) return quat_identity();
    return quat_scale(q, 1.0f / n);
}

static inline quat quat_conjugate(quat q) {
    quat r = {q.w, -q.x, -q.y, -q.z};
    return r;
}

static inline quat quat_mul(quat a, quat b) {
    quat r = {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
    return r;
}

static inline vec3 quat_rotate_vec3(quat q, vec3 v) {
    // printk("QR0\n");
    // printk_float("qr.q.w", q.w);
    // printk_float("qr.q.x", q.x);
    // printk_float("qr.q.y", q.y);
    // printk_float("qr.q.z", q.z);

    // printk_float("qr.v.x", v.x);
    // printk_float("qr.v.y", v.y);
    // printk_float("qr.v.z", v.z);

    quat p = {0.0f, v.x, v.y, v.z};
    // printk("QR1\n");

    quat qc = quat_conjugate(q);
    // printk("QR2\n");

    quat t = quat_mul(q, p);
    // printk("QR3\n");

    quat r = quat_mul(t, qc);
    // printk("QR4\n");

    vec3 out = {.x = r.x, .y = r.y, .z = r.z};
    // printk("QR5\n");

    // printk_float("qr.out.x", out.x);
    // printk_float("qr.out.y", out.y);
    // printk_float("qr.out.z", out.z);

    return out;
}

static inline quat quat_from_axis_angle(vec3 axis, float angle_rad) {
    vec3 n = vec3_normalize(axis);
    float half = 0.5f * angle_rad;
    float s = sinf(half);
    quat q = {cosf(half), n.x * s, n.y * s, n.z * s};
    return quat_normalize(q);
}
static inline quat quat_from_angular_velocity(vec3 w) {
    quat q = {0.0f, w.x, w.y, w.z};
    return q;
}

static inline quat quat_from_euler(float pitch, float roll, float yaw) {
    quat qx = quat_from_axis_angle(vec3_make(1.0f, 0.0f, 0.0f), pitch);
    quat qy = quat_from_axis_angle(vec3_make(0.0f, 1.0f, 0.0f), roll);
    quat qz = quat_from_axis_angle(vec3_make(0.0f, 0.0f, 1.0f), yaw);
    
    return quat_normalize(quat_mul(qz, quat_mul(qy, qx)));
}