#pragma once

#include "rpi.h"
#include "rpi-math.h"

typedef union {
    struct { float x, y, z; };
    struct { float r, g, b; };
} vec3;

static inline vec3 vec3_make(float x, float y, float z) {
    vec3 v = {.x = x, .y = y, .z = z};
    return v;
}

static inline void vec3_print(const char *name, vec3 v) { 
    output("%s = (%f, %f, %f)\n", name, v.x, v.y, v.z);
}

static inline vec3 vec3_init(float v) { 
return vec3_make(v, v, v);
}

static inline void vec3_to_arr(vec3 v, float out[3]) { 
    out[0] = v.x;
    out[1] = v.y;
    out[2] = v.z;
}

static inline vec3 vec3_zero(void) {
    vec3 v = {.x = 0.0f, .y = 0.0f, .z = 0.0f};
    return v;
}

static inline vec3 vec3_add(vec3 a, vec3 b) {
    vec3 r = {.x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z};
    return r;
}

static inline vec3 vec3_sub(vec3 a, vec3 b) {
    vec3 r = {.x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z};
    return r;
}

static inline vec3 vec3_scale(vec3 a, float s) {
    vec3 r = {.x = a.x * s, .y = a.y * s, .z = a.z * s};
    return r;
}

static inline vec3 vec3_neg(vec3 a) {
    return vec3_scale(a, -1.0f);
}

static inline vec3 vec3_min(vec3 a, vec3 b) {
    vec3 r = {
        .x = (a.x < b.x) ? a.x : b.x,
        .y = (a.y < b.y) ? a.y : b.y,
        .z = (a.z < b.z) ? a.z : b.z
    };
    return r;
}

static inline vec3 vec3_max(vec3 a, vec3 b) {
    vec3 r = {
        .x = (a.x > b.x) ? a.x : b.x,
        .y = (a.y > b.y) ? a.y : b.y,
        .z = (a.z > b.z) ? a.z : b.z
    };
    return r;
}

static inline vec3 vec3_hadamard(vec3 a, vec3 b) {
    vec3 r = {.x = a.x * b.x, .y = a.y * b.y, .z = a.z * b.z};
    return r;
}

static inline float vec3_dot(vec3 a, vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline vec3 vec3_cross(vec3 a, vec3 b) {
    vec3 r = {
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z =a.x * b.y - a.y * b.x
    };
    return r;
}

static inline float vec3_norm_sq(vec3 a) {
    return vec3_dot(a, a);
}

static inline float vec3_norm(vec3 a) {
    return sqrtf(vec3_norm_sq(a));
}

static inline vec3 vec3_normalize(vec3 a) {
    float n = vec3_norm(a);
    if (n <= 1e-8f) return vec3_zero();
    return vec3_scale(a, 1.0f / n);
}

static inline vec3 vec3_face_norm(vec3 a, vec3 b, vec3 c) { 
    vec3 e1 = vec3_sub(b, a);
    vec3 e2 = vec3_sub(c, a);
    vec3 n = vec3_cross(e1, e2);
    return vec3_normalize(n);
}

static inline vec3 vec3_lerp3(vec3 a, vec3 b, vec3 c, float wa, float wb, float wc) {
    return vec3_add(
        vec3_add(vec3_scale(a, wa), vec3_scale(b, wb)),
        vec3_scale(c, wc)
    );
}

static inline float vec3_sun_intensity(vec3 light_dir,vec3 a, vec3 b, vec3 c, float ambient_intensity) {
    vec3 light_norm = vec3_normalize(light_dir);
    vec3 face_norm = vec3_face_norm(a, b, c);
    return fmaxf(ambient_intensity, vec3_dot(light_norm, face_norm));
}

static inline vec3 vec3_sun_intensity_rgb(vec3 light_dir, vec3 rgb, vec3 a, vec3 b, vec3 c, float ambient_intensity) {
    float intensity = vec3_sun_intensity(light_dir, a, b, c, ambient_intensity);
    return vec3_scale(rgb, intensity);
}