#ifndef CL_H
#define CL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum { 
    HALT = 0,
    NOP = 1,
    FLUSH = 4,
    FLUSH_ALL_STATE = 5,
    START_TILE_BINNING = 6,
    BRANCH = 16,
    BRANCH_TO_SUBLIST = 17,
    STORE_MSAA_TLCB = 24,
    STORE_MSAA_TLCB_END = 25, // multisample anti-aliased tile color buffer
    STORE_FULL_RES_TLB = 26,
    STORE_TLB = 28,
    LOAD_TLB = 29,
    INDEXED_PRIMITIVE_LIST = 32,
    VERTEX_ARRAY_PRIMITIVES = 33,
    PRIMITIVE_LIST_FORMAT = 56,
    NV_SHADER_STATE = 65,
    CONFIGURATION_BITS = 96,
    CLIP_WINDOW = 102,
    CLIPPER_XY_SCALING = 105,
    VIEWPORT_OFFSET = 103,
    TILE_BIN_MODE_CFG = 112,
    TILE_RENDER_MODE_CFG = 113, 
    CLEAR_COLORS = 114,
    TILE_COORDS = 115,
} ControlRecordId;

// the buffer in here later becomes the control list
typedef struct { 
    uint8_t *base; // ARM CPU pointer to start of cl buffer
    uint32_t gpu_addr; // GPU-visible address 
    uint32_t bytes_written; // number of bytes emitted by CLE
    uint32_t capacity; // max buffer size in bytes
} cl_builder_t;

void cl_init(cl_builder_t *cl, uint8_t *cpu_ptr, uint32_t gpu_addr, uint32_t capacity);
void cl_reset(cl_builder_t *cl);

void cl_emit_bytes(cl_builder_t *cl, const void *v, uint32_t size);
void cl_emit_uint8(cl_builder_t *cl, uint8_t v);
void cl_emit_uint16(cl_builder_t *cl, uint16_t v);
void cl_emit_uint32(cl_builder_t *cl, uint32_t v);
void cl_emit_uint64(cl_builder_t *cl, uint64_t v);

void cl_emit_single_control_id(cl_builder_t *cl, ControlRecordId id);

// page 67
typedef struct { 
    uint8_t buf_to_store;
    uint8_t format;
    uint8_t mode; 
    uint8_t pixel_color_format;
    bool disable_double_buf_swap;
    bool disable_color_buf_clear;
    bool disable_z_stencil_clear;
    bool disable_vg_mask_clear;
    bool disable_color_buf_dump;
    bool disable_z_stencil_dump;
    bool disable_vg_mask_dump;
    bool last_tile;
    uint32_t tile_dump_buf_base_addr;
} store_tile_buf_cfg_t;

typedef struct { 

} load_tile_buf_cfg_t;

void cl_emit_store_tile_buf(cl_builder_t *cl, store_tile_buf_cfg_t cfg);
void cl_emit_load_tile_buf(cl_builder_t *cl, load_tile_buf_cfg_t cfg);

typedef struct { 
    uint32_t addr; // gpu bus address - tile allocation memory addr
    uint32_t size; // size of tile in bytes
    uint32_t tile_state_data_addr; // base addr of tile state
    uint8_t width_tiles; // height in tiles
    uint8_t height_tiles;
    bool multisample_mode;
    bool hdr_on; // 64 bit color depth or not
    bool init_tile_state_data; 
    uint8_t tile_alloc_initial_block_size; 
    uint8_t tile_alloc_block_size;
    bool double_buffer_non_ms;
} tile_bin_cfg_t;

void cl_emit_tile_bin_cfg(cl_builder_t *cl, tile_bin_cfg_t cfg);

// page 71
typedef struct {
    uint32_t addr; // must be gpu bus address
    uint16_t width;
    uint16_t height;
    bool multisample_mode;
    bool hdr_on;
    uint8_t no_hdr_fb_c_format;
    uint8_t decimate_mode;
    uint8_t memory_format;
    bool enable_vg_mask_buf;
    bool covg_mode;
    bool early_z_update_dir;
    bool early_z_cov_disable;
    bool double_buffer_non_ms;
} tile_render_cfg_t;

void cl_emit_tile_render_mode_cfg(cl_builder_t *cl, tile_render_cfg_t cfg);
void cl_emit_clear_colors(cl_builder_t *cl, uint64_t color, uint32_t clear_z, uint8_t clear_vg_mask, uint8_t clear_stencil);
void cl_emit_tile_coords(cl_builder_t *cl, uint8_t col, uint8_t row);

typedef struct __attribute__((packed, aligned(16))) { 
    uint8_t flag_bits;
    uint8_t vertex_data_stride;
    uint8_t frag_shader_num_unifs; // not used currently lmao
    uint8_t frag_shader_num_varyings;
    uint32_t frag_shader_code_addr;
    uint32_t frag_shader_unif_addr;
    uint32_t shaded_vertex_data_addr;
} nv_shader_state_cfg_t;

nv_shader_state_cfg_t* cl_emit_nv_shader_state(cl_builder_t *cl, nv_shader_state_cfg_t cfg);

// page 70 
// 
void cl_emit_clip_window(cl_builder_t *cl, uint16_t clip_window_h, uint16_t clip_window_w, uint16_t clip_window_b, uint16_t clip_window_l);
// xy coords after perspective division are normalized to [-1, 1]
// we convert to screen coords by using the specified viewport offset (+) and clipper xy scaling (*)
void cl_emit_viewport_offset(cl_builder_t *cl, int16_t x_center, int16_t y_center);
// height and width are in 1/16 pixels - page 70
void cl_emit_clipper_xy_scaling(cl_builder_t *cl, float viewport_half_width, float viewport_half_height);

// covg bit fields control Coverage Accumulation Pipe stuff, which is not needed for triangle rendeirng
typedef struct __attribute__((packed, aligned(4))) { 
    unsigned 
        enable_forward_facing_prim:1,
        enable_rev_facing_prim:1,
        clockwise_prim:1,
        depth_offset:1,
        anti_alias_points_lines:1, // not actually supported LOL
        covg_read_type:1,
        rast_oversample_mode:2,
        covg_pipe_select:1, 
        covg_update_mode:2,
        covg_read_mode:1,
        depth_test_func:3,
        z_updates:1,
        early_z:1,
        early_z_updates:1;
} configuration_bits_t;

void cl_emit_configuration_bits(cl_builder_t *cl, configuration_bits_t cfg);

typedef struct { 
    uint8_t primitive_mode;
    uint8_t index_type;
    uint32_t num_vertices;
    uint32_t index_data_addr;
    uint32_t max_index;
} indexed_primitive_list_cfg_t;

void cl_emit_primitive_list_format(cl_builder_t *cl, uint8_t primitive_type, uint8_t data_type);

void cl_emit_indexed_primitive_list(cl_builder_t *cl, indexed_primitive_list_cfg_t cfg);

void cl_emit_branch_to_sublist(cl_builder_t *cl, uint32_t sublist_addr);

#endif