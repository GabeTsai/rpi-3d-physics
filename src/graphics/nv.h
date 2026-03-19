#ifndef NV_H
#define NV_H

#include "rpi.h"
#include "rpi-math.h"
#include "vec3.h"
#include "quat.h"
#include "geom.h"
#include "physics.h"
#include "camera.h"
#include "graphics-settings.h"

#define MAX_VARYINGS 5
#define INV_WC_DEF 1.0f
#define SHADER_CFG_FLAG_BITS_DEF 0x01 // no clip header, no point size

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
    float varyings[3];
} nv_vertex_nch_nps_t;  // no clip header, no point size

typedef struct { 
    float xc;
    float yc;
    float zc;
    float wc;

    uint32_t xs_ys;
    float zs;
    float inv_wc;
    // float varyings[MAX_VARYINGS];
} __attribute__((packed)) nv_vertex_ch_nps_t;  // clip header, no point size

nv_vertex_nch_nps_t make_nv_vertex_nch_nps(float xs, float ys, float zs, float inv_wc, float r, float g, float b);

void put_triangle_to_nv(triangle tri, int idx_in_mesh, uint16_t *vert_index_list, nv_vertex_nch_nps_t *shaded_vertex_data_addr, float r, float g, float b);
void put_mesh_geom_to_nv(mesh_geom mesh, uint16_t *vert_index_list, nv_vertex_nch_nps_t *shaded_vertex_data_addr);

void put_proj_tri_to_nv(float xs[3], float ys[3], float zs[3],
    float r, float g, float b,
    int base_idx,
    uint16_t *vert_index_list,
    nv_vertex_nch_nps_t *shaded_vertex_data_addr);

#endif