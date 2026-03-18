#pragma once

#include <stdlib.h>
#include <string.h>
#include "rpi.h"
#include "rpi-math.h"

#define MAX_ENTRIES 16

typedef struct {
    int rows, cols;
    float entries[MAX_ENTRIES];
} matrix;

static inline matrix matrix_init(int rows, int cols) {
    assert(rows >= 0 && cols >= 0);
    assert((size_t)rows * (size_t)cols <= MAX_ENTRIES);

    matrix m = {0};
    m.rows = rows;
    m.cols = cols;
    return m;
}

// static inline void matrix_free(matrix *m) {
//     // free(m->entries);
//     // m->entries = NULL;
//     // m->rows = m->cols = 0;

//     // No free() allowed :((
//     m->entries = NULL;
//     m->rows = 0;
//     m->cols = 0;
// }

static inline int matrix_same_shape(const matrix *a, const matrix *b) {
    return a && b && a->rows == b->rows && a->cols == b->cols;
}

static inline int matrix_valid(const matrix *m) {
    return m && m->rows >= 0 && m->cols >= 0 && (m->rows == 0 || m->cols == 0 || m->entries != NULL);
}

static inline size_t matrix_len(const matrix *m) {
    return (size_t)m->rows * (size_t)m->cols;
}

static inline size_t matrix_index(const matrix *m, int r, int c) {
    return (size_t)r * (size_t)m->cols + (size_t)c;
}

static inline float matrix_get(const matrix *m, int r, int c) {
    return m->entries[matrix_index(m, r, c)];
}

static inline void matrix_set(matrix *m, int r, int c, float val) {
    m->entries[matrix_index(m, r, c)] = val;
}

// static inline float* matrix_get(matrix *m, int r, int c) {
//     return &m->entries[matrix_index(m, r, c)];
// }

static inline const float* matrix_const_get(const matrix *m, int r, int c) {
    return &m->entries[matrix_index(m, r, c)];
}

static inline matrix matrix_homogeneous_init(
    float tx, float ty, float tz,
    float pitch, float roll, float yaw
) {
    matrix m = matrix_init(4, 4);

    float cx = cosf(pitch);
    float sx = sinf(pitch);

    float cy = cosf(roll);
    float sy = sinf(roll);

    float cz = cosf(yaw);
    float sz = sinf(yaw);

    float *e = m.entries;

    e[0] = cz * cy;
    e[1] = cz * sy * sx - sz * cx;
    e[2] = cz * sy * cx + sz * sx;
    e[3] = tx;

    e[4] = sz * cy;
    e[5] = sz * sy * sx + cz * cx;
    e[6] = sz * sy * cx - cz * sx;
    e[7] = ty;

    e[8]  = -sy;
    e[9]  = cy * sx;
    e[10] = cy * cx;
    e[11] = tz;

    e[12] = 0.0f;
    e[13] = 0.0f;
    e[14] = 0.0f;
    e[15] = 1.0f;

    return m;
}

matrix matrix_add(const matrix *a, const matrix *b);
matrix matrix_sub(const matrix *a, const matrix *b);
matrix matrix_scale(const matrix *a, float x);
matrix matrix_gemm(const matrix *a, const matrix *b);

