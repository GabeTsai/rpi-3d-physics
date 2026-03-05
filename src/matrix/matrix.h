#pragma once

#include <stdlib.h>
#include <string.h>

typedef struct {
    int rows, cols;
    float *entries;
} matrix;

static inline int matrix_init(matrix *m, int rows, int cols) {
    m->rows = rows;
    m->cols = cols;
    m->entries = NULL;

    size_t nbytes = (size_t) rows * (size_t) cols * sizeof(float);
    if (posix_memalign((void**)&m->entries, 16, nbytes) != 0) return -1;
    memset(m->entries, 0, nbytes);
    return 0;
}

static inline void matrix_free(matrix *m) {
    free(m->entries);
    m->entries = NULL;
    m->rows = m->cols = 0;
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

int matrix_add(matrix *out, const matrix *a, const matrix *b);
int matrix_sub(matrix *out, const matrix *a, const matrix *b);
int matrix_scale(matrix *out, const matrix *a, float x);
int matrix_gemm(matrix *out, const matrix *a, const matrix *b);

