#include "rpi.h"
#include "mailbox-interface.h"
#include "cl.h"
#include "nv.h"
#include "v3d.h"
#include "stdint.h"
#include "bit-support.h"
#include "frag-shader-fixed-light.h"
#include "float-helpers.h"
#include "transform.h"
#include "vec3.h"

void notmain(void) { 
    kmalloc_init(10);
    mbox_response_t response = RPI_qpu_enable(1);

    int p_width = 128;
    int p_height = 128;
    int width_tiles = p_width / 64;
    int height_tiles = p_height / 64;

    output("V3D_IDENT0: %b\n", GET32(V3D_IDENT0));
    fb_info_t fb = RPI_fb_init(p_width, p_height, p_width, p_height, 32);
    output("fb.base_addr: %x\n", fb.base_addr);
    output("fb cpu addr: %x\n", BUS_TO_CPU(fb.base_addr));
    uint32_t *fb_cpu = (uint32_t *)BUS_TO_CPU(fb.base_addr);
    output("before: %x %x %x %x\n", fb_cpu[0], fb_cpu[1], fb_cpu[2], fb_cpu[3]);

    render_state_t render_state = {
        .fb_base_addr  = fb.base_addr,
        .width_px      = p_width,
        .height_px     = p_height,
        .width_tiles   = width_tiles,
        .height_tiles  = height_tiles,
    };

    cl_builder_t binning_cl;
    binning_state_t binning_state = cl_init_binning(&binning_cl, width_tiles, height_tiles, p_width, p_height);

    uint32_t* frag_shader_code_addr = (uint32_t*) frag_shader_fixed_light;
    uint32_t frag_shader_code_addr_gpu = CPU_TO_BUS(frag_shader_code_addr);
    output("frag_shader_code_addr: %x\n", frag_shader_code_addr);
    output("frag_shader_code_addr_gpu: %x\n", frag_shader_code_addr_gpu);

    assert(sizeof(nv_vertex_nch_nps_t) == 24);

    uint8_t vertex_data_stride = sizeof(nv_vertex_nch_nps_t);
    
    nv_vertex_nch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_nch_nps_t *) kmalloc_aligned(vertex_data_stride * 3, 16);
    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);
    
    float xs_1_f = 0.0f;
    float ys_1_f = -20.0f;
    float zs_1_f = 0.0f;
    float xs_2_f = -20.0f;
    float ys_2_f = 20.0f;
    float zs_2_f = 0.0f;
    float xs_3_f = 20.0f;
    float ys_3_f = 20.0f;
    float zs_3_f = 0.0f;

    float zlevel = 0.5f;
   
    float ambient_intensity = 0.3f;

    vec3 v1 = vec3_make(xs_1_f, ys_1_f, zs_1_f);
    vec3 v2 = vec3_make(xs_2_f, ys_2_f, zs_2_f);
    vec3 v3 = vec3_make(xs_3_f, ys_3_f, zs_3_f);

    vec3 light_dir = vec3_make(-1.0f, 1.0f, 1.0f);
    float intensity = vec3_sun_intensity(light_dir, v1, v2, v3, ambient_intensity);

    vec3 base_color = vec3_make(1.0f, 0.0f, 0.0f);
    vec3 color = vec3_scale(base_color, intensity);
    
    int16_t xs_1 = float_to_fixed12_4(xs_1_f);
    int16_t ys_1 = float_to_fixed12_4(ys_1_f);
    shaded_vertex_data_addr[0] = (nv_vertex_nch_nps_t) {
        .xs_ys = pack_xs_ys_fixed12_4(xs_1, ys_1),
        .zs = zlevel,
        .inv_wc = 1.0f,
        .varyings = { color.r, color.g, color.b }   
    };
    
    int16_t xs_2 = float_to_fixed12_4(xs_2_f);
    int16_t ys_2 = float_to_fixed12_4(ys_2_f);
    shaded_vertex_data_addr[1] = (nv_vertex_nch_nps_t) {
        .xs_ys = pack_xs_ys_fixed12_4(xs_2, ys_2),
        .zs = zlevel,
        .inv_wc = 1.0f,
        .varyings = { color.r, color.g, color.b }   
    };
    
    int16_t xs_3 = float_to_fixed12_4(xs_3_f);
    int16_t ys_3 = float_to_fixed12_4(ys_3_f);
    shaded_vertex_data_addr[2] = (nv_vertex_nch_nps_t) {
        .xs_ys = pack_xs_ys_fixed12_4(xs_3, ys_3),
        .zs = zlevel,
        .inv_wc = 1.0f,
        .varyings = { color.r, color.g, color.b }   
    };
    
    uint8_t flag_bits = 0x00;   // no clip header, no point size
    
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
        .index_type = 0, // 16 bit
        .num_vertices = 3,
        .index_data_addr = CPU_TO_BUS(indices),
        .max_index = 2,
    });
    
    cl_emit_single_control_id(&binning_cl, FLUSH_ALL_STATE);
    output("\n");

    cl_bin_one_frame(&binning_cl);

    output("flush count after: %x\n", get_flush_count());
    output("CT0CA after: %x CT0EA after : %x\n", GET32(V3D_CT0CA), GET32(V3D_CT0EA));
    output("CT0CS CTERR: %b\n", bits_get(GET32(V3D_CT0CS), 0, 5));
    output("PCS=%b\n", get_pcs());
    output("ERRSTAT=%b\n", get_errstat());
    output("DBGE=%b\n", get_dbge());
    output("ptb tile list completed\n\n");

    // RENDERING PASS
    cl_builder_t rendering_cl;
    cl_init_rendering(&rendering_cl, binning_state.tile_alloc_addr, render_state);

    float theta = 0.0f;
    
    while (1) {
        // Update only the vertex buffer contents.
        v1 = rotate_xyz_around_point((vec3){.x = xs_1_f, .y = ys_1_f, .z = zlevel},
                                        0.0f, 0.0f, 0.5f,
                                        0.75f*theta, theta, 1.5f*theta);

        v2 = rotate_xyz_around_point((vec3){.x = xs_2_f, .y = ys_2_f, .z = zlevel},
                                0.0f, 0.0f, 0.5f,
                                0.75f*theta, theta, 1.5f*theta);
        
        v3 = rotate_xyz_around_point((vec3){.x = xs_3_f, .y = ys_3_f, .z = zlevel},
                        0.0f, 0.0f, 0.5f,
                        0.75f*theta, theta, 1.5f*theta);
        
        intensity = vec3_sun_intensity(light_dir, v1, v2, v3, ambient_intensity);
        color = vec3_scale(base_color, intensity);

        int16_t xs_1 = float_to_fixed12_4(v1.x);
        int16_t ys_1 = float_to_fixed12_4(v1.y);
        shaded_vertex_data_addr[0] = (nv_vertex_nch_nps_t) {
            .xs_ys = pack_xs_ys_fixed12_4(xs_1, ys_1),
            .zs = zlevel,   // screen-space rotation: depth is constant
            .inv_wc = 1.0f,
            .varyings = { color.r, color.g, color.b }   // red
        };

        int16_t xs_2 = float_to_fixed12_4(v2.x);
        int16_t ys_2 = float_to_fixed12_4(v2.y);
        shaded_vertex_data_addr[1] = (nv_vertex_nch_nps_t) {
            .xs_ys = pack_xs_ys_fixed12_4(xs_2, ys_2),
            .zs = zlevel,
            .inv_wc = 1.0f,
            .varyings = { color.r, color.g, color.b }   // green
        };

        int16_t xs_3 = float_to_fixed12_4(v3.x);
        int16_t ys_3 = float_to_fixed12_4(v3.y);
        shaded_vertex_data_addr[2] = (nv_vertex_nch_nps_t) {
            .xs_ys = pack_xs_ys_fixed12_4(xs_3, ys_3),
            .zs = zlevel,
            .inv_wc = 1.0f,
            .varyings = { color.r, color.g, color.b }   // blue
        };

        clear_flush_count();
        cl_bin_one_frame(&binning_cl);

        clear_frame_count();
        cl_render_one_frame(&rendering_cl);

        theta += 0.01f;

        delay_ms(10);
    }
}
