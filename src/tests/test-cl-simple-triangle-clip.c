#include "rpi.h"
#include "mailbox-interface.h"
#include "nv.h"
#include "v3d.h"
#include "stdint.h"
#include "bit-support.h"
#include "simple-frag-shader.h"
#include "mailbox-interface.h"
#include "float-helpers.h"
#include "cl-interface.h"

// Warning - this test does not work yet - nothing will render. 
// We could not figure out how to configure hardware 
// clipping on the GPU. If you do, please let us know. 
# define NUM_TILES 1

static inline float s_to_c(float s_coord, float viewport_dim, float w_c) {
    float half = viewport_dim * 0.5f;
    return ((s_coord - half) / half) * w_c;
}

static inline float s_to_centered_s(float s_coord, float viewport_dim) { 
    float half = viewport_dim * 0.5f;
    return s_coord - half;
}

static inline float centered_s_to_c(float centered_s_coord, float viewport_dim, float w_c) { 
    float half = viewport_dim * 0.5f;
    return (centered_s_coord / half) * w_c;
}

void notmain(void) { 
    kmalloc_init(10);
    mbox_response_t response = RPI_qpu_enable(1);
    
    output("V3D_IDENT0: %b\n", GET32(V3D_IDENT0));
    fb_info_t fb = RPI_fb_init(64, 64, 64, 64, 32);
    output("fb.base_addr: %x\n", fb.base_addr);
    output("fb cpu addr: %x\n", BUS_TO_CPU(fb.base_addr));
    uint32_t *fb_cpu = (uint32_t *)BUS_TO_CPU(fb.base_addr);
    output("before: %x %x %x %x\n", fb_cpu[0], fb_cpu[1], fb_cpu[2], fb_cpu[3]);

    cl_builder_t binning_cl;
    int capacity = 1024; // 1 KB
    uint8_t* binning_cl_base_ptr = (uint8_t*) kmalloc(capacity);
    uint32_t binning_cl_base_ptr_gpu = CPU_TO_BUS(binning_cl_base_ptr);
    output("binning_cl_base_ptr: %x\n", binning_cl_base_ptr);
    output("binning_cl_base_ptr_gpu: %x\n", binning_cl_base_ptr_gpu);
    cl_init(&binning_cl, binning_cl_base_ptr, binning_cl_base_ptr_gpu, capacity);

    uint32_t tile_alloc_size = 1024 * 1024; // 1 MB
    uint32_t tile_alloc_addr = CPU_TO_BUS(kmalloc_aligned(tile_alloc_size, 256));
    uint8_t *tile_alloc_cpu = (uint8_t*) BUS_TO_CPU(tile_alloc_addr);

    uint32_t tile_state_data_addr = CPU_TO_BUS(kmalloc_aligned(48 * NUM_TILES, 16));

    tile_bin_cfg_t binning_cfg = { 
        .addr = tile_alloc_addr,
        .size = tile_alloc_size, 
        .tile_state_data_addr = tile_state_data_addr, // must be 16-byte aligned
        .width_tiles = 1, // height/width in tiles
        .height_tiles = 1,
        .multisample_mode = false,
        .hdr_on = false,
        .init_tile_state_data = true,
        .tile_alloc_initial_block_size = 0, // 32 bytes
        .tile_alloc_block_size = 0, // 32 bytes
        .double_buffer_non_ms = false,
    };
    cl_emit_tile_bin_cfg(&binning_cl, binning_cfg);

    cl_emit_single_control_id(&binning_cl, START_TILE_BINNING); 

    // 16 bit triangle
    // cl_emit_primitive_list_format(&binning_cl, 2, 1);
    cl_emit_uint8(&binning_cl, PRIMITIVE_LIST_FORMAT);
    cl_emit_uint8(&binning_cl, 0x32);

    float viewport_hw = 64.0f / 2.0f;
    float viewport_hh = 64.0f / 2.0f;
    float viewport_hw_16th = viewport_hw * 16.0f;
    float viewport_hh_16th = viewport_hh * 16.0f;
    // left, bottom, width, height
    cl_emit_clip_window(&binning_cl, 0, 0, 64, 64);

    configuration_bits_t cfg = { 
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
    cl_emit_configuration_bits(&binning_cl, cfg);
    
    // viewport_offset is in 1/16th pixel units (same as XS/YS), so 32px * 16 = 512
    cl_emit_viewport_offset(&binning_cl, 512, 512); // centre at pixel (32, 32)

    cl_emit_clipper_xy_scaling(&binning_cl, viewport_hw_16th, viewport_hh_16th);
    cl_emit_clipper_z_scale_and_offset(&binning_cl, 0.5f, 0.5f);
    cl_emit_z_min_max_clipping_planes(&binning_cl, -1.0f, 1.0f);

    uint32_t* frag_shader_code_addr = (uint32_t*) simple_frag_shader;
    uint32_t frag_shader_code_addr_gpu = CPU_TO_BUS(frag_shader_code_addr);
    
    output("frag_shader_code_addr: %x\n", frag_shader_code_addr);
    output("frag_shader_code_addr_gpu: %x\n", frag_shader_code_addr_gpu);

    output("sizeof(nv_vertex_ch_nps_t): %d\n", sizeof(nv_vertex_ch_nps_t));
    assert(sizeof(nv_vertex_ch_nps_t) == 32);

    uint8_t vertex_data_stride = sizeof(nv_vertex_ch_nps_t);

    // 128-bit (16-byte) alignment required when clip header is present (datasheet p.80)
    nv_vertex_ch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_ch_nps_t *) kmalloc_aligned(vertex_data_stride * 3, 16);
    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);

    // these are in top-left absolute screen coordinates
    float xs_1_f = 32.0f;
    float ys_1_f = 12.0f;
    float xs_2_f = 12.0f;
    float ys_2_f = 52.0f;
    float xs_3_f = 52.0f;
    float ys_3_f = 52.0f;

    // relative to viewport center
    float xs_1_f_centered = s_to_centered_s(xs_1_f, 64);
    float ys_1_f_centered = s_to_centered_s(ys_1_f, 64);
    float xs_2_f_centered = s_to_centered_s(xs_2_f, 64);
    float ys_2_f_centered = s_to_centered_s(ys_2_f, 64);
    float xs_3_f_centered = s_to_centered_s(xs_3_f, 64);
    float ys_3_f_centered = s_to_centered_s(ys_3_f, 64);

    printk("xs_1_f: %f, ys_1_f: %f\n", xs_1_f, ys_1_f);
    printk("xs_2_f: %f, ys_2_f: %f\n", xs_2_f, ys_2_f);
    printk("xs_3_f: %f, ys_3_f: %f\n", xs_3_f, ys_3_f);

    // convert centered to clip space coords
    float xc_1 = centered_s_to_c(xs_1_f_centered, 64, 1.0f);
    float yc_1 = centered_s_to_c(ys_1_f_centered, 64, 1.0f);
    float xc_2 = centered_s_to_c(xs_2_f_centered, 64, 1.0f);
    float yc_2 = centered_s_to_c(ys_2_f_centered, 64, 1.0f);
    float xc_3 = centered_s_to_c(xs_3_f_centered, 64, 1.0f);
    float yc_3 = centered_s_to_c(ys_3_f_centered, 64, 1.0f);

    printk("xc_1: %f, yc_1: %f\n", xc_1, yc_1);
    printk("xc_2: %f, yc_2: %f\n", xc_2, yc_2);
    printk("xc_3: %f, yc_3: %f\n", xc_3, yc_3);

    int16_t xs_1 = float_to_fixed12_4(xs_1_f);
    int16_t ys_1 = float_to_fixed12_4(ys_1_f);
    shaded_vertex_data_addr[0] = (nv_vertex_ch_nps_t) {
        .xc = xc_1, .yc = yc_1,
        .zc = 0.0f, .wc = 1.0f,
        .xs_ys = pack_xs_ys_fixed12_4(xs_1, ys_1),
        .zs = 0.5f,
        .inv_wc = 1.0f,
    };

    int16_t xs_2 = float_to_fixed12_4(xs_2_f);
    int16_t ys_2 = float_to_fixed12_4(ys_2_f);
    shaded_vertex_data_addr[1] = (nv_vertex_ch_nps_t) {
        .xc = xc_2, .yc = yc_2,
        .zc = 0.0f, .wc = 1.0f,
        .xs_ys = pack_xs_ys_fixed12_4(xs_2, ys_2),
        .zs = 0.5f,
        .inv_wc = 1.0f,
    };

    int16_t xs_3 = float_to_fixed12_4(xs_3_f);
    int16_t ys_3 = float_to_fixed12_4(ys_3_f);
    shaded_vertex_data_addr[2] = (nv_vertex_ch_nps_t) {
        .xc = xc_3, .yc = yc_3,
        .zc = 0.0f, .wc = 1.0f,
        .xs_ys = pack_xs_ys_fixed12_4(xs_3, ys_3),
        .zs = 0.5f,
        .inv_wc = 1.0f,
    };

    // bit 3 = clip header present, bit 0 = single-threaded fragment shader
    // bit 2 (Enable Clipping) deliberately left off until Clipper XY Scaling
    // and Clipper Z Scale/Offset records are added
    uint8_t flag_bits = 0b1101;
    
    nv_shader_state_cfg_t shader_cfg = {
        .flag_bits = flag_bits,
        .vertex_data_stride = vertex_data_stride,
        .frag_shader_num_unifs = 0,
        .frag_shader_num_varyings = MAX_VARYINGS,
        .frag_shader_code_addr = frag_shader_code_addr_gpu,
        .frag_shader_unif_addr = 0,
        .shaded_vertex_data_addr = shaded_vertex_data_addr_gpu,
    };

    nv_shader_state_cfg_t* nv_shader_record = cl_emit_nv_shader_state(&binning_cl, shader_cfg);

    uint8_t *indices = kmalloc_aligned(3 * sizeof(uint8_t), 4);
    indices[0] = 0;
    indices[1] = 1;
    indices[2] = 2;

    cl_emit_indexed_primitive_list(&binning_cl, (indexed_primitive_list_cfg_t) {
        .primitive_mode = 4,
        .index_type = 0, // 8 bit
        .num_vertices = 3,
        .index_data_addr = CPU_TO_BUS(indices),
        .max_index = 2,
    });
    
    PUT32(V3D_BFC, 0b1); // clear frame count 
    cl_emit_single_control_id(&binning_cl, FLUSH_ALL_STATE);
    output("\n");
    
    reset_cle_thread(0);
    PUT32(V3D_CT0CA, binning_cl.gpu_addr);
    PUT32(V3D_CT0EA, binning_cl.gpu_addr + binning_cl.bytes_written);
    
    while ((GET32(V3D_BFC) & 0xff) == 0) {}

    assert(GET32(V3D_CT0CA) == GET32(V3D_CT0EA));
    output("flush count after: %x\n", bits_get(GET32(V3D_BFC), 0, 7));
    output("CT0CA after: %x CT0EA after : %x\n", GET32(V3D_CT0CA), GET32(V3D_CT0EA));
    output("CT0CS CTERR: %b\n", bits_get(GET32(V3D_CT0CS), 0, 5));
    output("PCS=%b\n", bits_get(GET32(V3D_PCS), 0, 8));
    output("ERRSTAT=%b\n", bits_get(GET32(V3D_ERRSTAT), 0, 15));
    output("DBGE=%b\n", bits_get(GET32(V3D_DBGE), 0, 20));
    output("ptb tile list completed\n\n");

    // // RENDERING PASS
    cl_builder_t rendering_cl;
    int rendering_capacity = 1024;
    uint8_t* rendering_cl_base_ptr = (uint8_t*) kmalloc(rendering_capacity);
    uint32_t rendering_cl_base_ptr_gpu = CPU_TO_BUS(rendering_cl_base_ptr);
    output("rendering_cl_base_ptr: %x\n", rendering_cl_base_ptr);
    output("rendering_cl_base_ptr_gpu: %x\n", rendering_cl_base_ptr_gpu);
    cl_init(&rendering_cl, rendering_cl_base_ptr, rendering_cl_base_ptr_gpu, rendering_capacity);
    
    cl_emit_clear_colors(&rendering_cl, 0xFF000000FF000000ULL, 0xFFFFFF, 0, 0);

    tile_render_cfg_t rendering_cfg = {
        .addr = fb.base_addr,
        .width = 64,
        .height = 64,
        .multisample_mode = false,
        .hdr_on = false,
        .no_hdr_fb_c_format = 1, // rgba8888
        .decimate_mode = 0,
        .memory_format = 0 // T-format
    }; 

    cl_emit_tile_render_mode_cfg(&rendering_cl, rendering_cfg);

    // force TLB clear
    cl_emit_tile_coords(&rendering_cl, 0, 0);
    cl_emit_single_control_id(&rendering_cl, STORE_TLB);
    cl_emit_uint16(&rendering_cl,  0);
    cl_emit_uint32(&rendering_cl,  0);

    cl_emit_tile_coords(&rendering_cl, 0, 0);
    // tile block size * index - within one tile, there can be multiple blocks
    cl_emit_branch_to_sublist(&rendering_cl, tile_alloc_addr + 32 * (0 * 1 + 0));

    cl_emit_single_control_id(&rendering_cl, STORE_MSAA_TLCB_END);

    PUT32(V3D_RFC, 1); // clear frame count
    output("frame count before: %x\n", bits_get(GET32(V3D_RFC), 0, 7));
    reset_cle_thread(1);

    output("v3d base addr: %x\n", V3D_BASE);
    
    PUT32(V3D_CT1CA, rendering_cl.gpu_addr);
    PUT32(V3D_CT1EA, rendering_cl.gpu_addr + rendering_cl.bytes_written);

    int iter = 0;
    while ((GET32(V3D_RFC) & 0xff) == 0) {}
    assert(GET32(V3D_CT1CA) == GET32(V3D_CT1EA));

    output("frame count after: %x\n", bits_get(GET32(V3D_RFC), 0, 7));
    output("CT1CA after: %x CT1EA after : %x\n", GET32(V3D_CT1CA), GET32(V3D_CT1EA));
    output("CT1CS CTERR: %b\n", bits_get(GET32(V3D_CT1CS), 0, 5));
    output("PCS=%b\n", bits_get(GET32(V3D_PCS), 0, 8));
    output("ERRSTAT=%b\n", bits_get(GET32(V3D_ERRSTAT), 0, 15));
    output("DBGE=%b\n", bits_get(GET32(V3D_DBGE), 0, 20));
    output("FDBGO=%b\n\n", bits_get(GET32(V3D_FDBGO), 0, 31));

    while(1);
}
