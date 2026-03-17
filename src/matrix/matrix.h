#pragma once

#include <stdlib.h>
#include <string.h>
#include "rpi.h"
#include "rpi-math.h"

typedef struct {
    int rows, cols;
    float *entries;
} matrix;

static inline matrix matrix_init(int rows, int cols) {
    matrix m;

    m.rows = rows;
    m.cols = cols;

    size_t nbytes = (size_t)rows * (size_t)cols * sizeof(float);
    m.entries = kmalloc(nbytes);

    memset(m.entries, 0, nbytes);
    return m;
}

static inline void matrix_free(matrix *m) {
    // free(m->entries);
    // m->entries = NULL;
    // m->rows = m->cols = 0;

    // No free() allowed :((
    m->entries = NULL;
    m->rows = 0;
    m->cols = 0;
}

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

    float cp = cosf(pitch);
    float sp = sinf(pitch);

    float cr = cosf(roll);
    float sr = sinf(roll);

    float cy = cosf(yaw);
    float sy = sinf(yaw);

    float *e = m.entries;

    e[0] = cy*cp;
    e[1] = cy*sp*sr - sy*cr;
    e[2] = cy*sp*cr + sy*sr;
    e[3] = tx;

    e[4] = sy*cp;
    e[5] = sy*sp*sr + cy*cr;
    e[6] = sy*sp*cr - cy*sr;
    e[7] = ty;

    e[8]  = -sp;
    e[9]  = cp*sr;
    e[10] = cp*cr;
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

