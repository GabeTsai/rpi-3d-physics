#pragma once

#include <math.h>
#include "physics.h"

static inline vec3 vec3_make(float x, float y, float z) {
    vec3 v = {x, y, z};
    return v;
}

static inline vec3 vec3_zero(void) {
    vec3 v = {0.0f, 0.0f, 0.0f};
    return v;
}

static inline vec3 vec3_add(vec3 a, vec3 b) {
    vec3 r = {a.x + b.x, a.y + b.y, a.z + b.z};
    return r;
}

static inline vec3 vec3_sub(vec3 a, vec3 b) {
    vec3 r = {a.x - b.x, a.y - b.y, a.z - b.z};
    return r;
}

static inline vec3 vec3_scale(vec3 a, float s) {
    vec3 r = {a.x * s, a.y * s, a.z * s};
    return r;
}

static inline vec3 vec3_hadamard(vec3 a, vec3 b) {
    vec3 r = {a.x * b.x, a.y * b.y, a.z * b.z};
    return r;
}

static inline float vec3_dot(vec3 a, vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline vec3 vec3_cross(vec3 a, vec3 b) {
    vec3 r = {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
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
