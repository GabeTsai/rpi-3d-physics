#pragma once

#include "matrix.h"
#include "vec3.h"
#include "quat.h"

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

    matrix T1 = matrix_homogeneous_init(-cx, -cy, -cz, 0.0f, 0.0f, 0.0f);
    matrix T2 = matrix_homogeneous_init( cx,  cy,  cz, 0.0f, 0.0f, 0.0f);
    matrix R  = matrix_homogeneous_init(0.0f, 0.0f, 0.0f, pitch, roll, yaw);
    matrix TR = matrix_gemm(&R, &T1);
    matrix M  = matrix_gemm(&T2, &TR);
    return transform_xyz(&M, p);
}