#pragma once

#include "rpi.h"
#include "rpi-math.h"

typedef struct {
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
    printk("QD0 a=%x b=%x\n", (unsigned)a, (unsigned)b);
    volatile float sink1 = a->w;
    printk("QD1\n");
    volatile float sink2 = a->x;
    printk("QD2\n");
    volatile float sink3 = a->y;
    printk("QD3\n");
    volatile float sink4 = a->z;
    printk("QD4\n");
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
    return quat_dot(&q, &q);
}

static inline float quat_norm(quat q) {
    return sqrtf(quat_norm_sq(q));
}

static inline quat quat_normalize(quat q) {
    float n = quat_norm(q);
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
    quat p = {0.0f, v.x, v.y, v.z};
    quat qn = quat_normalize(q);
    quat r = quat_mul(quat_mul(qn, p), quat_conjugate(qn));
    vec3 out = {.x = r.x, .y = r.y, .z = r.z};
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