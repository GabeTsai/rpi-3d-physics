// matrix_perf.c
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include "rpi.h"
#include "rpi-math.h"

#include "matrix.h"

// -------------------------
// Timing helpers
// -------------------------
static inline uint64_t nsec_now(void) {
#if defined(CLOCK_MONOTONIC_RAW)
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
#endif
}

static inline double sec_since(uint64_t t0_ns) {
    return (double)(nsec_now() - t0_ns) * 1e-9;
}

// Prevent optimizer from eliminating work
static volatile float g_sink = 0.0f;

// -------------------------
// Simple helpers
// -------------------------
static void fill_pattern(matrix *m, float seed) {
    const size_t n = (size_t)m->rows * (size_t)m->cols;
    for (size_t i = 0; i < n; i++) {
        float x = seed + (float)(i % 1024) * 0.001f;
        m->entries[i] = sinf(x) + 0.1f * cosf(3.0f * x);
    }
}

static float checksum(const matrix *m) {
    const size_t n = (size_t)m->rows * (size_t)m->cols;
    double acc = 0.0;
    for (size_t i = 0; i < n; i += 16) {
        acc += m->entries[i];
    }
    return (float)acc;
}

static void die(const char *msg) {
    fprintf(stderr, "error: %s\n", msg);
    exit(1);
}

// -------------------------
// Benchmarks
// -------------------------
static void bench_add(int rows, int cols, int iters) {
    matrix A = matrix_init(rows, cols);
    matrix B = matrix_init(rows, cols);
    matrix C = {0};

    if (!matrix_valid(&A) || !A.entries) die("matrix_init A failed");
    if (!matrix_valid(&B) || !B.entries) die("matrix_init B failed");

    fill_pattern(&A, 0.1f);
    fill_pattern(&B, 1.3f);

    for (int i = 0; i < 5; i++) {
        if (C.entries) matrix_free(&C);
        C = matrix_add(&A, &B);
        if (!matrix_valid(&C) || !C.entries) die("matrix_add failed");
    }

    uint64_t t0 = nsec_now();
    for (int i = 0; i < iters; i++) {
        if (C.entries) matrix_free(&C);
        C = matrix_add(&A, &B);
        if (!matrix_valid(&C) || !C.entries) die("matrix_add failed");
    }
    double dt = sec_since(t0);

    const double bytes = (double)rows * (double)cols * (double)sizeof(float) * 3.0 * (double)iters;
    const double gbps = bytes / dt / 1e9;

    float cs = checksum(&C);
    g_sink += cs;

    printf("ADD   %dx%d  iters=%d  time=%.6fs  BW=%.3f GB/s  checksum=%.5f\n",
           rows, cols, iters, dt, gbps, cs);

    matrix_free(&A);
    matrix_free(&B);
    matrix_free(&C);
}

static void bench_scale(int rows, int cols, int iters, float k) {
    matrix A = matrix_init(rows, cols);
    matrix C = {0};

    if (!matrix_valid(&A) || !A.entries) die("matrix_init A failed");

    fill_pattern(&A, 0.7f);

    for (int i = 0; i < 5; i++) {
        if (C.entries) matrix_free(&C);
        C = matrix_scale(&A, k);
        if (!matrix_valid(&C) || !C.entries) die("matrix_scale failed");
    }

    uint64_t t0 = nsec_now();
    for (int i = 0; i < iters; i++) {
        if (C.entries) matrix_free(&C);
        C = matrix_scale(&A, k);
        if (!matrix_valid(&C) || !C.entries) die("matrix_scale failed");
    }
    double dt = sec_since(t0);

    const double bytes = (double)rows * (double)cols * (double)sizeof(float) * 2.0 * (double)iters;
    const double gbps = bytes / dt / 1e9;

    float cs = checksum(&C);
    g_sink += cs;

    printf("SCALE %dx%d  iters=%d  time=%.6fs  BW=%.3f GB/s  checksum=%.5f\n",
           rows, cols, iters, dt, gbps, cs);

    matrix_free(&A);
    matrix_free(&C);
}

static void bench_gemm(int m, int k, int n, int iters) {
    matrix A = matrix_init(m, k);
    matrix B = matrix_init(k, n);
    matrix C = {0};

    if (!matrix_valid(&A) || !A.entries) die("matrix_init A failed");
    if (!matrix_valid(&B) || !B.entries) die("matrix_init B failed");

    fill_pattern(&A, 0.2f);
    fill_pattern(&B, 0.9f);

    for (int i = 0; i < 2; i++) {
        if (C.entries) matrix_free(&C);
        C = matrix_gemm(&A, &B);
        if (!matrix_valid(&C) || !C.entries) die("matrix_gemm failed");
    }

    uint64_t t0 = nsec_now();
    for (int i = 0; i < iters; i++) {
        if (C.entries) matrix_free(&C);
        C = matrix_gemm(&A, &B);
        if (!matrix_valid(&C) || !C.entries) die("matrix_gemm failed");
    }
    double dt = sec_since(t0);

    const double flops = 2.0 * (double)m * (double)n * (double)k * (double)iters;
    const double gflops = flops / dt / 1e9;

    float cs = checksum(&C);
    g_sink += cs;

    printf("GEMM  %dx%dx%d iters=%d time=%.6fs  %.3f GFLOP/s  checksum=%.5f\n",
           m, k, n, iters, dt, gflops, cs);

    matrix_free(&A);
    matrix_free(&B);
    matrix_free(&C);
}

// -------------------------
// Main
// -------------------------
int main(int argc, char **argv) {
    (void)argc; (void)argv;

    printf("matrix_perf starting...\n");

    bench_add(256, 256, 200);
    bench_add(512, 512, 80);
    bench_add(1024, 1024, 20);

    bench_scale(256, 256, 250, 0.75f);
    bench_scale(512, 512, 100, 0.75f);
    bench_scale(1024, 1024, 25, 0.75f);

    bench_gemm(128, 128, 128, 15);
    bench_gemm(256, 256, 256, 4);
    bench_gemm(512, 512, 512, 1);

    printf("sink=%f\n", g_sink);
    return 0;
}