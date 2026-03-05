#include "matrix.h"

int matrix_add(matrix *out, const matrix *a, const matrix *b) {
    if (!matrix_valid(out) || !matrix_valid(a) || !matrix_valid(b)) return -1;
    if (!matrix_same_shape(out, a) || !matrix_same_shape(a, b)) return -1;

    const size_t n = (size_t) out->rows * (size_t) out->cols;
    const float *a_entries = a->entries;
    const float *b_entries = b->entries;
    float *out_entries = out->entries;

    for (size_t i = 0; i < n; i++) out_entries[i] = a_entries[i] + b_entries[i];

    return 0;
}

int matrix_sub(matrix *out, const matrix *a, const matrix *b) {
    if (!matrix_valid(out) || !matrix_valid(a) || !matrix_valid(b)) return -1;
    if (!matrix_same_shape(out, a) || !matrix_same_shape(a, b)) return -1;

    const size_t n = (size_t)out->rows * (size_t)out->cols;
    const float *a_entries = a->entries;
    const float *b_entries = b->entries;
    float *out_entries = out->entries;

    for (size_t i = 0; i < n; i++) out_entries[i] = a_entries[i] - b_entries[i];

    return 0;
}

int matrix_scale(matrix *out, const matrix *a, float x) {
    if (!matrix_valid(out) || !matrix_valid(a)) return -1;
    if (!matrix_same_shape(out, a)) return -1;

    const size_t n = (size_t)out->rows * (size_t)out->cols;
    const float *a_entries = a->entries;
    float *out_entries = out->entries;

    for (size_t i = 0; i < n; i++) out_entries[i] = a_entries[i] * x;

    return 0;
}

// out = a * b  (naive O(m*n*k))
// Assumes row-major storage.
// Requires: out->rows=a->rows, out->cols=b->cols, a->cols=b->rows
int matrix_gemm(matrix *out, const matrix *a, const matrix *b) {
    if (!matrix_valid(out) || !matrix_valid(a) || !matrix_valid(b)) return -1;

    const int m = a->rows;
    const int k = a->cols;
    const int n = b->cols;

    if (k != b->rows) return -1;
    if (out->rows != m || out->cols != n) return -1;

    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            float sum = 0.0f;
            for (int p = 0; p < k; p++) {
                sum += a->entries[(size_t)i*(size_t)a->cols + (size_t)p] *
                       b->entries[(size_t)p*(size_t)b->cols + (size_t)j];
            }
            out->entries[(size_t)i*(size_t)out->cols + (size_t)j] = sum;
        }
    }
    return 0;
}