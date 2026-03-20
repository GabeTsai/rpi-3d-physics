#ifndef CL_H
#define CL_H

#include "cl-interface.h"
#include "stdint.h"
#include "stdbool.h"
#include "rpi.h"
#include "v3d.h"
#include "simple-frag-shader.h"
#include "mailbox-interface.h"
#include "float-helpers.h"
#include "graphics-settings.h"
#include "nv.h"

#define ONE_MB 1024 * 1024

#define CL_BUF_SIZE_DEF ONE_MB
#define TILE_ALLOC_SIZE_DEF ONE_MB

typedef struct {
    uint32_t tile_alloc_addr;
    uint32_t tile_state_data_addr;
} binning_state_t;

static inline tile_bin_cfg_t default_tile_bin_cfg(uint32_t alloc_addr, uint32_t alloc_size,
    uint32_t state_addr, uint8_t w, uint8_t h) {
    return (tile_bin_cfg_t) {
        .addr                          = alloc_addr,
        .size                          = alloc_size,
        .tile_state_data_addr          = state_addr,
        .width_tiles                   = w,
        .height_tiles                  = h,
        .multisample_mode              = false,
        .hdr_on                        = false,
        .init_tile_state_data          = true,
        .tile_alloc_initial_block_size = 0,
        .tile_alloc_block_size         = 0,
        .double_buffer_non_ms          = false,
    };
}

static inline configuration_bits_t default_configuration_bits(void) {
    return (configuration_bits_t) {
        .enable_forward_facing_prim = true,
        .enable_rev_facing_prim = true,
        .clockwise_prim = false,
        .depth_offset = false,
        .anti_alias_points_lines = false,
        .covg_read_type = false,
        .rast_oversample_mode = 0,
        .covg_pipe_select = false,
        .covg_update_mode = 0,
        .covg_read_mode = false,
        .depth_test_func = DEPTH_TEST_LT,
        .z_updates = true,
        .early_z = false,
        .early_z_updates = false,
    };
}

// back-face culling enabled — use for 3D meshes with consistently outward-wound triangles
// back-face culling tells the GPU to discard triangles that wind clockwise on the screen 
// in other words, the normal vector of the face points in the same direction as the camera's view vector
static inline configuration_bits_t default_configuration_bits_3d(void) {
    return (configuration_bits_t) {
        .enable_forward_facing_prim = true,
        .enable_rev_facing_prim = false, 
        .clockwise_prim = false,
        .depth_offset = false,
        .anti_alias_points_lines = false,
        .covg_read_type = false,
        .rast_oversample_mode = 0,
        .covg_pipe_select = false,
        .covg_update_mode = 0,
        .covg_read_mode = false,
        .depth_test_func = DEPTH_TEST_LT,
        .z_updates = true,
        .early_z = true,
        .early_z_updates = true,
    };
}

static inline tile_render_cfg_t default_tile_render_cfg(uint32_t fb_base_addr, uint16_t width_px, uint16_t height_px) {
    return (tile_render_cfg_t) {
        .addr = fb_base_addr,
        .width = width_px,
        .height = height_px,
        .multisample_mode = false,
        .hdr_on = false,
        .no_hdr_fb_c_format = 1, // rgba8888
        .decimate_mode = 0,
        .memory_format = 0, // T-format
        .double_buffer_non_ms = false
    };
}

typedef struct { 
    uint16_t width_px;
    uint16_t height_px;
    uint8_t width_tiles;
    uint8_t height_tiles;
    uint32_t fb_base_addr;
} render_state_t;

binning_state_t cl_init_binning(cl_builder_t *cl, uint8_t width_tiles, uint8_t height_tiles,
                                uint16_t width_px, uint16_t height_px, int back_face_culling);

static inline nv_shader_state_cfg_t default_nv_shader_state_cfg(uint8_t vertex_data_stride, uint32_t frag_shader_code_addr, 
                        uint32_t shaded_vertex_data_addr, uint16_t *vert_index_list, int num_vertices) {
    return (nv_shader_state_cfg_t) {
        .flag_bits = SHADER_CFG_FLAG_BITS_DEF,
        .vertex_data_stride = vertex_data_stride,
        .frag_shader_num_unifs = 0,
        .frag_shader_num_varyings = MAX_VARYINGS,
        .frag_shader_code_addr = frag_shader_code_addr,
        .frag_shader_unif_addr = 0,
        .shaded_vertex_data_addr = shaded_vertex_data_addr,
    };
}

static inline indexed_primitive_list_cfg_t default_indexed_primitive_list_cfg(uint8_t primitive_mode, uint8_t index_type, 
                        uint32_t num_vertices, uint32_t index_data_addr, uint32_t max_index) {
    return (indexed_primitive_list_cfg_t) {
        .primitive_mode = primitive_mode,
        .index_type = index_type,
        .num_vertices = num_vertices,
        .index_data_addr = index_data_addr,
        .max_index = max_index,
    };
}

void cl_bin_primitives(cl_builder_t *cl, int vertex_data_stride, uint32_t frag_shader_code_addr, 
                        uint32_t shaded_vertex_data_addr, uint16_t *vert_index_list, int num_vertices);

void cl_bin_one_frame(cl_builder_t *cl);

void cl_init_rendering(cl_builder_t *cl, uint32_t tile_alloc_addr, render_state_t render_state);

void cl_render_one_frame(cl_builder_t *cl);
#endif