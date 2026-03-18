#pragma once

#include "matrix.h"
#include "physics.h"

/*
 * Internal cache so we don't rebuild T1/T2 every call when rotating
 * around the same point.
 */
typedef struct {
    int initialized;
    float cx, cy, cz;
    matrix T1;
    matrix T2;
    matrix R;
    matrix TR;
    matrix M;
    int lock;
} rotate_xyz_cache_t;

static rotate_xyz_cache_t cache;

static inline vec3 transform_xyz(const matrix *T, vec3 p) {
    float ox = T->entries[0] * p.x + T->entries[1] * p.y + T->entries[2] * p.z + T->entries[3];
    float oy = T->entries[4] * p.x + T->entries[5] * p.y + T->entries[6] * p.z + T->entries[7];
    float oz = T->entries[8] * p.x + T->entries[9] * p.y + T->entries[10] * p.z + T->entries[11];

    return (vec3){.x = ox, .y = oy, .z =oz};
}

static inline vec3 rotate_xyz_around_point(vec3 p,
                                           float cx, float cy, float cz,
                                           float pitch, float roll, float yaw)
{
    while (cache.lock);
    cache.lock = 1;
    if (!cache.initialized ||
        cache.cx != cx || cache.cy != cy || cache.cz != cz)
    {
        cache.cx = cx;
        cache.cy = cy;
        cache.cz = cz;
        cache.T1 = matrix_homogeneous_init(-cx, -cy, -cz, 0.0f, 0.0f, 0.0f);
        cache.T2 = matrix_homogeneous_init( cx,  cy,  cz, 0.0f, 0.0f, 0.0f);
        cache.initialized = 1;
    }

    cache.R  = matrix_homogeneous_init(0.0f, 0.0f, 0.0f, roll, pitch, yaw);
    cache.TR = matrix_gemm(&cache.R, &cache.T1);
    cache.M  = matrix_gemm(&cache.T2, &cache.TR);
    cache.lock = 0;
    return transform_xyz(&cache.M, p);
}