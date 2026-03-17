#include "cl.h"

binning_state_t cl_init_binning(cl_builder_t *cl, uint8_t width_tiles, uint8_t height_tiles,
                                uint16_t width_px, uint16_t height_px) {
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

    configuration_bits_t cfg = default_configuration_bits();
    cl_emit_configuration_bits(cl, cfg);

    return (binning_state_t) {
        .tile_alloc_addr = tile_alloc_addr,
        .tile_state_data_addr = tile_state_data_addr,
    }; 
}

void cl_bin_one_frame(cl_builder_t *cl) {
    reset_cle_thread(0);
    PUT32(V3D_CT0CA, cl->gpu_addr);
    PUT32(V3D_CT0EA, cl->gpu_addr + cl->bytes_written);
    while ((GET32(V3D_BFC) & 0xff) == 0) {}
    assert(GET32(V3D_CT0CA) == GET32(V3D_CT0EA));
}

void cl_init_rendering(cl_builder_t *cl, uint32_t fb_base_addr, uint32_t tile_alloc_addr) {
    uint8_t *cl_buf = (uint8_t *) kmalloc(CL_BUF_SIZE_DEF);
    cl_init(cl, cl_buf, CPU_TO_BUS(cl_buf), CL_BUF_SIZE_DEF);

    cl_emit_clear_colors(cl, 0xFF000000FF000000ULL, 0xFFFFFF, 0, 0);
    tile_render_cfg_t render_cfg = default_tile_render_cfg(fb_base_addr);
    cl_emit_tile_render_mode_cfg(cl, render_cfg);

    cl_clear_tlb(cl);

    cl_emit_tile_coords(cl, 0, 0);
    cl_emit_branch_to_sublist(cl, tile_alloc_addr + 32 * (0 * 1 + 0));

    cl_emit_single_control_id(cl, STORE_MSAA_TLCB_END);
}

void cl_render_one_frame(cl_builder_t *cl) {
    reset_cle_thread(1);
    PUT32(V3D_CT1CA, cl->gpu_addr);
    PUT32(V3D_CT1EA, cl->gpu_addr + cl->bytes_written);
    while ((GET32(V3D_RFC) & 0xff) == 0) {}
    assert(GET32(V3D_CT1CA) == GET32(V3D_CT1EA));
}
