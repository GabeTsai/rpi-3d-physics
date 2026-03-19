#include "rpi.h"
#include "mailbox-interface.h"
#include "cl-interface.h"
#include "cl.h"
#include "geom.h"
#include "nv.h"
#include "v3d.h"
#include "stdint.h"
#include "frag-shader-fixed-light.h"
#include "mailbox-interface.h"

void notmain(void) { 
    kmalloc_init(10);
    mbox_response_t response = RPI_qpu_enable(1);

    int p_width = 128;
    int p_height = 128;
    int width_tiles = p_width / 64;
    int height_tiles = p_height / 64;

    output("V3D_IDENT0: %b\n", GET32(V3D_IDENT0));
    fb_info_t fb = RPI_fb_init(p_width, p_height, p_width, p_height, 32);
    
    render_state_t render_state = {
        .fb_base_addr = fb.base_addr,
        .width_px = p_width,
        .height_px = p_height,
        .width_tiles = width_tiles,
        .height_tiles = height_tiles,
    };

    output("fb.base_addr: %x\n", fb.base_addr);
    output("fb cpu addr: %x\n", BUS_TO_CPU(fb.base_addr));
    uint32_t *fb_cpu = (uint32_t *)BUS_TO_CPU(fb.base_addr);
    output("before: %x %x %x %x\n", fb_cpu[0], fb_cpu[1], fb_cpu[2], fb_cpu[3]);

    cl_builder_t binning_cl;
    binning_state_t binning_state = cl_init_binning(&binning_cl, width_tiles, height_tiles, p_width, p_height, 0);
    
    uint32_t* frag_shader_code_addr = (uint32_t*) frag_shader_fixed_light;
    uint32_t frag_shader_code_addr_gpu = CPU_TO_BUS(frag_shader_code_addr);
    output("frag_shader_code_addr: %x\n", frag_shader_code_addr);
    output("frag_shader_code_addr_gpu: %x\n", frag_shader_code_addr_gpu);
    // assert(sizeof(nv_vertex_nch_nps_t) == 24);

    uint8_t vertex_data_stride = sizeof(nv_vertex_nch_nps_t);
    
    nv_vertex_nch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_nch_nps_t *) kmalloc_aligned(vertex_data_stride * 6, 16);
    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);
    
    uint16_t *indices = kmalloc_aligned(6 * sizeof(uint16_t), 4);

    float xs_1_f = 0.0f;
    float ys_1_f = -20.0f;
    float xs_2_f = -20.0f;
    float ys_2_f = 20.0f;
    float xs_3_f = 20.0f;
    float ys_3_f = 20.0f;

    uint16_t total_verts = 0;
    triangle tri = triangle_make_from_pts(xs_1_f, ys_1_f, xs_2_f, ys_2_f, xs_3_f, ys_3_f, 0.5f);
    mesh_geom mesh = mesh_geom_init_triangle(tri, 1.0f, 0.0f, 0.0f);
    put_mesh_geom_to_nv(mesh, indices, shaded_vertex_data_addr);
    total_verts += 3;

    float xs_1_f2 = 0.0f;
    float ys_1_f2 = -20.0f;
    float xs_2_f2 = -20.0f;
    float ys_2_f2 = 20.0f;
    float xs_3_f2 = 20.0f;
    float ys_3_f2 = 20.0f;

    triangle tri2 = triangle_make_from_pts(xs_1_f2, ys_1_f2, xs_2_f2, ys_2_f2, xs_3_f2, ys_3_f2, 0.2f);
    mesh_geom mesh2 = mesh_geom_init_triangle(tri2, 0.0f, 1.0f, 0.0f);
    put_triangle_to_nv(mesh2.triangles[0], total_verts, indices, shaded_vertex_data_addr, mesh2.r, mesh2.g, mesh2.b);
    total_verts += 3;

    cl_bin_primitives(&binning_cl, vertex_data_stride, frag_shader_code_addr_gpu, 
shaded_vertex_data_addr_gpu, indices, total_verts);
    
    output("flush count before: %x\n", get_flush_count());
    cl_bin_one_frame(&binning_cl);
    output("flush count after: %x\n", get_flush_count());

    // // RENDERING PASS
    cl_builder_t rendering_cl;
    cl_init_rendering(&rendering_cl, binning_state.tile_alloc_addr, render_state);

    output("frame count before: %x\n", get_frame_count());
    cl_render_one_frame(&rendering_cl);
    output("frame count after: %x\n", get_frame_count());

    delay_ms(5000);
}
