#ifndef NV_H
#define NV_H

#include <stdint.h>
#include <stdbool.h>
#include "rpi.h"

#define MAX_VARYINGS 3
/*
 * Code for NV-pipeline specific formats
 * Naming convention: nv_vertex<num fields>_t
 * coords with an s suffix are after perspective divide
 */

 // coords here are after perspective divide
typedef struct { 
    uint32_t xs_ys; // 32 bits, xs and ys are 16 bits each, 12.4 fixed point
    float zs; // 32
    float inv_wc;
    float varyings[MAX_VARYINGS];
} nv_vertex_nch_nps_t;  // no clip header, no point size

typedef struct { 
    float xc;
    float yc;
    float zc;
    float wc;

    uint32_t packed_xs_ys;
    float zs;
    float inv_wc;
    float varyings[MAX_VARYINGS];
} nv_vertex_nch_ps_t;  // clip header, no point size

#endif