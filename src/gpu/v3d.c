#include "v3d.h"
#include "stdint.h"
#include "rpi.h"

static void check_thread_id(int thread_id) {
    if (!((thread_id == 0) || (thread_id == 1))) {
        panic("invalid thread id: %d", thread_id);
    }
}

void reset_cle_thread(int thread_id) { 
    check_thread_id(thread_id);
    uint32_t addr = V3D_CT0CS + thread_id * 4;
    // uint32_t cs = GET32(addr);
    // cs |= 1 << 15; // reset bit
    // PUT32(addr, cs);
    // uint32_t addr = V3D_CT0CS + thread_id * 4;
    PUT32(addr, 1u << 15);
}

bool is_cle_thread_running(int thread_id) {
    check_thread_id(thread_id);
    uint32_t addr = V3D_CT0CS + thread_id * 4;
    uint32_t cs = GET32(addr);
    return cs & (1 << 15);
}