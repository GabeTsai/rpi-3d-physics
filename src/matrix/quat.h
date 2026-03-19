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
    volatile float sink1 = a->w;
    volatile float sink2 = a->x;
    volatile float sink3 = a->y;
    volatile float sink4 = a->z;
    return sink1 * b->w + sink2 * b->x + sink3 * b->y + sink4 * b->z;
}

static inline float quat_norm_sq(quat q) {
    return quat_dot(&q, &q);
}

static inline float quat_norm(quat q) {
    float norm_sq = quat_norm_sq(q);
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
    quat p = {0.0f, v.x, v.y, v.z};
    quat qc = quat_conjugate(q);
    quat t = quat_mul(q, p);
    quat r = quat_mul(t, qc);
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
