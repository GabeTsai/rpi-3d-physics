#include "cl.h"
#include "cl-interface.h"

binning_state_t cl_init_binning(cl_builder_t *cl, uint8_t width_tiles, uint8_t height_tiles,
                                uint16_t width_px, uint16_t height_px, int back_face_culling) {
    uint8_t *cl_buf = (uint8_t *) kmalloc(CL_BUF_SIZE_DEF);
    cl_init(cl, cl_buf, CPU_TO_BUS(cl_buf), CL_BUF_SIZE_DEF);

    uint32_t tile_alloc_size = TILE_ALLOC_SIZE_DEF;
    uint32_t tile_alloc_addr = CPU_TO_BUS(kmalloc_aligned(tile_alloc_size, 256));
    uint32_t tile_state_data_addr = CPU_TO_BUS(
        kmalloc_aligned(TILE_STATE_DATA_SIZE * width_tiles * height_tiles, 16));

    tile_bin_cfg_t binning_cfg = default_tile_bin_cfg(tile_alloc_addr, tile_alloc_size, tile_state_data_addr,
        width_tiles, height_tiles);

    cl_emit_tile_bin_cfg(cl, binning_cfg);
    cl_emit_single_control_id(cl, START_TILE_BINNING);

    cl_emit_primitive_list_format(cl, TRIANGLES, U16BITINDEX);

    cl_emit_clip_window(cl, 0, 0, width_px, height_px);
    // center of screen is (0, 0)
    cl_emit_viewport_offset(cl, width_px / 2 * 16, height_px / 2 * 16);
    configuration_bits_t cfg;
    if (back_face_culling) {
        cfg = default_configuration_bits_3d();
    } else {
        cfg = default_configuration_bits();
    }
    cl_emit_configuration_bits(cl, cfg);
    
    clear_flush_count();
    return (binning_state_t) {
        .tile_alloc_addr = tile_alloc_addr,
        .tile_state_data_addr = tile_state_data_addr,
    }; 
}

// bin primitives for up to 
void cl_bin_primitives(cl_builder_t *cl, int vertex_data_stride, uint32_t frag_shader_code_addr, 
                        uint32_t shaded_vertex_data_addr, uint16_t *vert_index_list, int num_vertices) {
    nv_shader_state_cfg_t shader_cfg = default_nv_shader_state_cfg(vertex_data_stride, frag_shader_code_addr, 
                        shaded_vertex_data_addr, vert_index_list, num_vertices);
    cl_emit_nv_shader_state(cl, shader_cfg);

    indexed_primitive_list_cfg_t primitive_list_cfg = default_indexed_primitive_list_cfg(TRIANGLES, U16BITINDEX, 
                        num_vertices, CPU_TO_BUS(vert_index_list), num_vertices - 1);
    cl_emit_indexed_primitive_list(cl, primitive_list_cfg);

}

void cl_bin_one_frame(cl_builder_t *cl) {
    uint32_t saved = cl->bytes_written;
    cl_emit_single_control_id(cl, FLUSH_ALL_STATE);
    reset_cle_thread(0);
    PUT32(V3D_CT0CA, cl->gpu_addr);
    PUT32(V3D_CT0EA, cl->gpu_addr + cl->bytes_written);
    while ((GET32(V3D_BFC) & 0xff) == 0) {}
    assert(GET32(V3D_CT0CA) == GET32(V3D_CT0EA));
    cl->bytes_written = saved;  // overwrite FLUSH_ALL_STATE next frame, not accumulate
}

void cl_init_rendering(cl_builder_t *cl, uint32_t tile_alloc_addr, render_state_t rs) {
    uint8_t *cl_buf = (uint8_t *) kmalloc(CL_BUF_SIZE_DEF);
    cl_init(cl, cl_buf, CPU_TO_BUS(cl_buf), CL_BUF_SIZE_DEF);

    cl_emit_clear_colors(cl, DEFAULT_FB_CLEAR_COLOR, 0xFFFFFF, 0, 0);
    tile_render_cfg_t render_cfg = default_tile_render_cfg(rs.fb_base_addr, rs.width_px, rs.height_px);
    cl_emit_tile_render_mode_cfg(cl, render_cfg);

    cl_clear_tlb(cl);

    for (uint32_t y = 0; y < rs.height_tiles; y++) { 
        for (uint32_t x = 0; x < rs.width_tiles; x++) { 
            cl_emit_tile_coords(cl, x, y);
            cl_emit_branch_to_sublist(cl, tile_alloc_addr + TILE_ALLOC_ENTRY_SIZE * (y * rs.width_tiles + x));

            if (x == rs.width_tiles - 1 && y == rs.height_tiles - 1) { 
                cl_emit_single_control_id(cl, STORE_MSAA_TLCB_END);
            } else { 
                cl_emit_single_control_id(cl, STORE_MSAA_TLCB);
            }
        }
    }
    clear_frame_count();
}

void cl_render_one_frame(cl_builder_t *cl) {
    reset_cle_thread(1);
    PUT32(V3D_CT1CA, cl->gpu_addr);
    PUT32(V3D_CT1EA, cl->gpu_addr + cl->bytes_written);
    while ((GET32(V3D_RFC) & 0xff) == 0) {}
    assert(GET32(V3D_CT1CA) == GET32(V3D_CT1EA));
}
