#include "cl-interface.h"
#include "rpi.h"
#include "v3d.h"
#include <string.h>

#define verbose_cl_interface 1

void cl_init(cl_builder_t *cl, uint8_t *cpu_ptr, uint32_t gpu_addr, uint32_t capacity) {
    cl->base = cpu_ptr;
    cl->gpu_addr = gpu_addr;
    cl->capacity = capacity;
    cl->bytes_written = 0;
}

void cl_reset(cl_builder_t *cl) {
    cl->bytes_written = 0;
}

void cl_emit_bytes(cl_builder_t *cl, const void *v, uint32_t size) {
    if (cl->bytes_written + size > cl->capacity) {
        panic("CL buffer overflow");
    }
    memcpy(&cl->base[cl->bytes_written], v, size);
    cl->bytes_written += size;
    
}

void cl_emit_uint8(cl_builder_t *cl, uint8_t v)   { cl_emit_bytes(cl, &v, sizeof v); }
void cl_emit_uint16(cl_builder_t *cl, uint16_t v) { cl_emit_bytes(cl, &v, sizeof v); }
void cl_emit_uint32(cl_builder_t *cl, uint32_t v) { cl_emit_bytes(cl, &v, sizeof v); }
void cl_emit_uint64(cl_builder_t *cl, uint64_t v) { cl_emit_bytes(cl, &v, sizeof v); }

void cl_emit_int16(cl_builder_t *cl, int16_t v) { cl_emit_bytes(cl, &v, sizeof v); }

void cl_emit_float(cl_builder_t *cl, float v) { cl_emit_bytes(cl, &v, sizeof v); }

void cl_emit_store_tile_buf(cl_builder_t *cl, store_tile_buf_cfg_t cfg) {
    cl_emit_uint8(cl, STORE_TLB);
    assert ((cfg.tile_dump_buf_base_addr & 0xf) == 0);

    uint8_t flag1 = 0;
    flag1 |= cfg.buf_to_store;
    flag1 |= cfg.format << 4;
    flag1 |= cfg.mode << 6;
    cl_emit_uint8(cl, flag1);

    uint8_t flag2 = 0;
    flag2 |= cfg.pixel_color_format;
    flag2 |= cfg.disable_double_buf_swap << 4;
    flag2 |= cfg.disable_color_buf_clear << 5;
    flag2 |= cfg.disable_z_stencil_clear << 6;
    flag2 |= cfg.disable_vg_mask_clear << 7;
    cl_emit_uint8(cl, flag2);

    uint32_t flag3 = 0;
    flag3 |= cfg.disable_color_buf_dump;
    flag3 |= cfg.disable_z_stencil_dump << 1;
    flag3 |= cfg.disable_vg_mask_dump << 2;
    flag3 |= cfg.last_tile << 3;
    flag3 |= ((cfg.tile_dump_buf_base_addr >> 4) & 0x0FFFFFFF) << 4;
    cl_emit_uint32(cl, flag3);
}

void cl_emit_single_control_id(cl_builder_t *cl, ControlRecordId id) {
    cl_emit_uint8(cl, id);
}

void cl_emit_tile_bin_cfg(cl_builder_t *cl, tile_bin_cfg_t cfg) {
    cl_emit_single_control_id(cl, TILE_BIN_MODE_CFG);
    cl_emit_uint32(cl, cfg.addr);
    cl_emit_uint32(cl, cfg.size);
    cl_emit_uint32(cl, cfg.tile_state_data_addr);
    cl_emit_uint8(cl, cfg.width_tiles);
    cl_emit_uint8(cl, cfg.height_tiles);
    uint8_t flag1 = 0;
    flag1 |= cfg.multisample_mode;
    flag1 |= cfg.hdr_on << 1;
    flag1 |= cfg.init_tile_state_data << 2;
    flag1 |= cfg.tile_alloc_initial_block_size << 3;
    flag1 |= cfg.tile_alloc_block_size << 5;
    flag1 |= cfg.double_buffer_non_ms << 7;
    cl_emit_uint8(cl, flag1);
}

void cl_emit_tile_render_mode_cfg(cl_builder_t *cl, tile_render_cfg_t cfg) {
    cl_emit_uint8(cl, TILE_RENDER_MODE_CFG);
    cl_emit_uint32(cl, cfg.addr);
    cl_emit_uint16(cl, cfg.width);
    cl_emit_uint16(cl, cfg.height);
    uint8_t flag1 = 0;
    flag1 |= cfg.multisample_mode;
    flag1 |= cfg.hdr_on << 1;
    flag1 |= cfg.no_hdr_fb_c_format << 2; 
    flag1 |= cfg.decimate_mode << 4; 
    flag1 |= cfg.memory_format << 6;
    cl_emit_uint8(cl, flag1);
    uint8_t flag2 = 0;
    flag2 |= cfg.enable_vg_mask_buf;
    flag2 |= cfg.covg_mode << 1;
    flag2 |= cfg.early_z_update_dir << 2;
    flag2 |= cfg.early_z_cov_disable << 3;
    flag2 |= cfg.double_buffer_non_ms << 4;
    cl_emit_uint8(cl, flag2);
}

// we need this or else the first frame will contain junk
// https://github.com/librerpi/lk-overlay/blob/master/platform/bcm28xx/v3d/v3d.c
void cl_clear_tlb(cl_builder_t *rendering_cl) {
    cl_emit_tile_coords(rendering_cl, 0, 0);
    cl_emit_single_control_id(rendering_cl, STORE_TLB);
    cl_emit_uint16(rendering_cl,  0);
    cl_emit_uint32(rendering_cl,  0);
}

void cl_emit_clear_colors(cl_builder_t *cl, uint64_t color, uint32_t clear_z, uint8_t clear_vg_mask, uint8_t clear_stencil) {
    cl_emit_uint8(cl, CLEAR_COLORS);
    cl_emit_uint64(cl, color);
    // 24-bit Z clear value (little-endian): 0xFFFFFF = 1.0 (far plane)
    cl_emit_uint8(cl, clear_z & 0xFF);
    cl_emit_uint8(cl, (clear_z >> 8) & 0xFF);
    cl_emit_uint8(cl, (clear_z >> 16) & 0xFF);
    cl_emit_uint8(cl, clear_vg_mask);
    cl_emit_uint8(cl, clear_stencil);
}

void cl_emit_tile_coords(cl_builder_t *cl, uint8_t col, uint8_t row) {
    cl_emit_uint8(cl, TILE_COORDS);
    cl_emit_uint8(cl, col);
    cl_emit_uint8(cl, row);
}

nv_shader_state_cfg_t* cl_emit_nv_shader_state(cl_builder_t *cl, nv_shader_state_cfg_t cfg) {
    // page 69 
    nv_shader_state_cfg_t* rec = (nv_shader_state_cfg_t*) kmalloc_aligned(sizeof(*rec), 16);
    *rec = cfg;

    cl_emit_uint8(cl, NV_SHADER_STATE);          // 65
    cl_emit_uint32(cl, CPU_TO_BUS((uint32_t*)rec));         // pointer only
    return rec;
}

void cl_emit_clip_window(cl_builder_t *cl, uint16_t clip_window_l, uint16_t clip_window_b, uint16_t clip_window_w, uint16_t clip_window_h) {
    cl_emit_uint8(cl, CLIP_WINDOW);
    cl_emit_uint16(cl, clip_window_l);
    cl_emit_uint16(cl, clip_window_b);
    cl_emit_uint16(cl, clip_window_w);
    cl_emit_uint16(cl, clip_window_h);
}

void cl_emit_viewport_offset(cl_builder_t *cl, int16_t x_center, int16_t y_center) {
    cl_emit_uint8(cl, VIEWPORT_OFFSET);
    cl_emit_int16(cl, x_center);
    cl_emit_int16(cl, y_center);
}

void cl_emit_z_min_max_clipping_planes(cl_builder_t *cl, float min_zw, float max_zw) {
    cl_emit_uint8(cl, Z_MIN_MAX_CLIPPING_PLANES);
    cl_emit_float(cl, min_zw);
    cl_emit_float(cl, max_zw);
}


void cl_emit_clipper_xy_scaling(cl_builder_t *cl, float viewport_half_width, float viewport_half_height) {
    cl_emit_uint8(cl, CLIPPER_XY_SCALING);
    cl_emit_float(cl, viewport_half_width);
    cl_emit_float(cl, viewport_half_height);
}

void cl_emit_clipper_z_scale_and_offset(cl_builder_t *cl, float viewport_z_scale, float viewport_z_offset) {
    cl_emit_uint8(cl, CLIPPER_Z_SCALE_AND_OFFSET);
    cl_emit_float(cl, viewport_z_scale);
    cl_emit_float(cl, viewport_z_offset);
}

void cl_emit_configuration_bits(cl_builder_t *cl, configuration_bits_t cfg) {
    cl_emit_uint8(cl, CONFIGURATION_BITS);

    uint8_t flag1 = 0;
    flag1 |= cfg.enable_forward_facing_prim;
    flag1 |= cfg.enable_rev_facing_prim << 1;
    flag1 |= cfg.clockwise_prim << 2;
    flag1 |= cfg.depth_offset << 3;
    flag1 |= cfg.anti_alias_points_lines << 4;
    flag1 |= cfg.covg_read_type << 5;
    flag1 |= cfg.rast_oversample_mode << 6;
    cl_emit_uint8(cl, flag1);

    uint8_t flag2 = 0;
    flag2 |= cfg.covg_pipe_select;
    flag2 |= cfg.covg_update_mode << 1;
    flag2 |= cfg.covg_read_mode << 3;
    flag2 |= cfg.depth_test_func << 4;
    cl_emit_uint8(cl, flag2);

    uint8_t flag3 = 0;
    flag3 |= cfg.z_updates;
    flag3 |= cfg.early_z << 1;
    flag3 |= cfg.early_z_updates << 2;

    cl_emit_uint8(cl, flag3);
}

void cl_emit_branch_to_sublist(cl_builder_t *cl, uint32_t sublist_addr) {
    cl_emit_uint8(cl, BRANCH_TO_SUBLIST);
    cl_emit_uint32(cl, sublist_addr);
}

void cl_emit_primitive_list_format(cl_builder_t *cl, PrimitiveType primitive_type, PrimitiveListDataType data_type) {
    cl_emit_uint8(cl, PRIMITIVE_LIST_FORMAT);
    uint8_t flag1 = ((primitive_type) << 4 | (data_type));
    cl_emit_uint8(cl, flag1);
}

void cl_emit_indexed_primitive_list(cl_builder_t *cl, indexed_primitive_list_cfg_t cfg) {
    cl_emit_uint8(cl, INDEXED_PRIMITIVE_LIST);
    uint8_t flag1 = 0;
    flag1 |= cfg.primitive_mode;
    flag1 |= cfg.index_type << 4;
    cl_emit_uint8(cl, flag1);
    cl_emit_uint32(cl, cfg.num_vertices);
    cl_emit_uint32(cl, cfg.index_data_addr);
    cl_emit_uint32(cl, cfg.max_index);
}