#pragma once

#include "matrix.h"
#include "physics.h"

static inline vec3 transform_xyz(const matrix *T, vec3 p) {
    const float *entries = T->entries;

    float ox = entries[0] * p.x + entries[1] * p.y + entries[2] * p.z + entries[3];
    float oy = entries[4] * p.x + entries[5] * p.y + entries[6] * p.z + entries[7];
    float oz = entries[8] * p.x + entries[9] * p.y + entries[10] * p.z + entries[11];

    return (vec3){.x = ox, .y = oy, .z =oz};
}

static inline vec3 rotate_xyz_around_point(vec3 p,
                                           float cx, float cy, float cz,
                                           float pitch, float roll, float yaw)
{
    matrix T1 = matrix_homogeneous_init(-cx, -cy, -cz, 0.0f, 0.0f, 0.0f);
    matrix R  = matrix_homogeneous_init(0.0f, 0.0f, 0.0f, roll, pitch, yaw); //swapped because ??
    matrix T2 = matrix_homogeneous_init(cx, cy, cz, 0.0f, 0.0f, 0.0f);

    matrix TR = matrix_gemm(&R, &T1);
    matrix M  = matrix_gemm(&T2, &TR);

    return transform_xyz(&M, p);
}