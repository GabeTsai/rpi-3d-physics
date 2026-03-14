#include "matrix.h"

matrix matrix_add(const matrix *a, const matrix *b) {
    if (!matrix_valid(a) || !matrix_valid(b)) {
        matrix m = {0};
        return m;
    }

    if (!matrix_same_shape(a, b)) {
        matrix m = {0};
        return m;
    }

    matrix out = matrix_init(a->rows, a->cols);

    const size_t n = matrix_len(a);
    const float *a_entries = a->entries;
    const float *b_entries = b->entries;
    float *out_entries = out.entries;

    for (size_t i = 0; i < n; i++)
        out_entries[i] = a_entries[i] + b_entries[i];

    return out;
}

matrix matrix_sub(const matrix *a, const matrix *b) {
    if (!matrix_valid(a) || !matrix_valid(b)) {
        matrix m = {0};
        return m;
    }

    if (!matrix_same_shape(a, b)) {
        matrix m = {0};
        return m;
    }

    matrix out = matrix_init(a->rows, a->cols);

    const size_t n = matrix_len(a);
    const float *a_entries = a->entries;
    const float *b_entries = b->entries;
    float *out_entries = out.entries;

    for (size_t i = 0; i < n; i++)
        out_entries[i] = a_entries[i] - b_entries[i];

    return out;
}

matrix matrix_scale(const matrix *a, float x) {
    if (!matrix_valid(a)) {
        matrix m = {0};
        return m;
    }

    matrix out = matrix_init(a->rows, a->cols);

    const size_t n = matrix_len(a);
    const float *a_entries = a->entries;
    float *out_entries = out.entries;

    for (size_t i = 0; i < n; i++)
        out_entries[i] = a_entries[i] * x;

    return out;
}

// out = a * b  (naive O(m*n*k))
// row-major
matrix matrix_gemm(const matrix *a, const matrix *b) {
    if (!matrix_valid(a) || !matrix_valid(b)) {
        matrix m = {0};
        return m;
    }

    if (a->cols != b->rows) {
        matrix m = {0};
        return m;
    }

    const int m = a->rows;
    const int k = a->cols;
    const int n = b->cols;

    matrix out = matrix_init(m, n);

    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            float sum = 0.0f;

            for (int p = 0; p < k; p++) {
                sum += a->entries[(size_t)i*(size_t)a->cols + (size_t)p] *
                       b->entries[(size_t)p*(size_t)b->cols + (size_t)j];
            }

            out.entries[(size_t)i*(size_t)n + (size_t)j] = sum;
        }
    }

    return out;
}