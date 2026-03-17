#ifndef V3D_H
#define V3D_H

#include <stdint.h>
#include <stdbool.h>
#include "rpi.h"
#include "mailbox.h"
#include "bit-support.h"

// https://docs.broadcom.com/doc/12358545

#define V3D_BASE 0x20C00000

#define BUS_TO_CPU(ptr) ((uint32_t) (ptr) & ~GPU_OFFSET) // GPU bus to CPU address
#define CPU_TO_BUS(ptr) ((uint32_t) (ptr) | GPU_OFFSET) // CPU address to GPU bus

// page 82 of the manual
// 
typedef enum { 
    V3D_IDENT0 = V3D_BASE + 0x000,
    V3D_IDENT1 = V3D_BASE + 0x004,
    V3D_IDENT2 = V3D_BASE + 0x008,
    V3D_SCRATCH = V3D_BASE + 0x010,
    V3D_CT0CS = V3D_BASE + 0x100,
    V3D_CT1CS = V3D_BASE + 0x104,
    V3D_CT0EA = V3D_BASE + 0x108,
    V3D_CT1EA = V3D_BASE + 0x10c,
    V3D_CT0CA = V3D_BASE + 0x110,
    V3D_CT1CA = V3D_BASE + 0x114,
    V3D_PCS = V3D_BASE + 0x130,
    V3D_BFC = V3D_BASE + 0x134,
    V3D_RFC = V3D_BASE + 0x138,
    V3D_BPCA = V3D_BASE + 0x300,
    V3D_BPCS = V3D_BASE + 0x304,
    V3D_BPOA = V3D_BASE + 0x308,
    V3D_BPOS = V3D_BASE + 0x30c,

    V3D_SQRSV0 = V3D_BASE + 0x410,
    V3D_SQRSV1 = V3D_BASE + 0x414,

    V3D_PCTRC = V3D_BASE + 0x670,
    V3D_PCTRE = V3D_BASE + 0x674, // enable perf counters
    V3D_PCTR0 = V3D_BASE + 0x678,
    V3D_PCTRS0 = V3D_BASE + 0x67c,
    V3D_PCTR1 = V3D_BASE + 0x680,
    V3D_PCTRS1 = V3D_BASE + 0x684,
    V3D_PCTR2 = V3D_BASE + 0x688,
    V3D_PCTRS2 = V3D_BASE + 0x68c,
    V3D_PCTR3 = V3D_BASE + 0x690,
    V3D_PCTRS3 = V3D_BASE + 0x694,

    V3D_DBGE = V3D_BASE + 0xf00,
    V3D_FDBGO = V3D_BASE + 0xf04,
    V3D_ERRSTAT = V3D_BASE + 0xf20
} V3DReg;

void reset_cle_thread(int thread_id);
bool is_cle_thread_running(int thread_id);

static inline uint32_t get_ctnreg(uint32_t ct_reg, int thread_id) { 
    if (thread_id == 0) { 
        return GET32(ct_reg);
    } else if (thread_id == 1) { 
        return GET32(ct_reg + 4);
    } else { 
        panic("invalid thread id: %d", thread_id);
    }
}
static inline uint32_t get_ctnca(int thread_id) { 
    return get_ctnreg(V3D_CT0CA, thread_id);
}

static inline uint32_t get_ctncea(int thread_id) { 
    return get_ctnreg(V3D_CT0EA, thread_id);
}

static inline uint32_t get_ctncs(int thread_id) { 
    return get_ctnreg(V3D_CT0CS, thread_id);
}

static inline uint32_t get_pcs(void) { 
    return bits_get(GET32(V3D_PCS), 0, 8);
}

static inline uint32_t get_errstat(void) { 
    return bits_get(GET32(V3D_ERRSTAT), 0, 15);
}

static inline uint32_t get_dbge(void) { 
    return bits_get(GET32(V3D_DBGE), 0, 20);
}

static inline uint32_t get_flush_count(void) { 
    return bits_get(GET32(V3D_BFC), 0, 7);
}

static inline uint32_t get_frame_count(void) { 
    return bits_get(GET32(V3D_RFC), 0, 7);
}

static inline void clear_flush_count(void) { 
    PUT32(V3D_BFC, 0b1);
}

static inline void clear_frame_count(void) { 
    PUT32(V3D_RFC, 0b1);
}

#endif