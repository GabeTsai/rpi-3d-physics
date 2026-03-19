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
#include "render.h"

/*
 This test renders 20 cubes in a line. camera looks straight at cubes.
*/
void notmain(void) { 
    kmalloc_init(20);
    mbox_response_t response = RPI_qpu_enable(1);

    int p_width = 1280;
    int p_height = 704;
    int width_tiles = p_width / 64;
    int height_tiles = p_height / 64;

    fb_info_t fb = RPI_fb_init(p_width, p_height, p_width, p_height, 32);
    
    camera cam;
    camera_init(&cam,
        vec3_make(0.0f, 0.0f, -1000.0f),      // offset above and to the side
        quat_from_euler(0.0f, 0.0f, 0.0f),    // tilt down, rotate left to face origin
        640.0f, 640.0f,
        0.0f, 0.0f, 
        1.0f, 5000.0f);     
    render_state_t render_state = {
        .fb_base_addr = fb.base_addr,
        .width_px = p_width,
        .height_px = p_height,
        .width_tiles = width_tiles,
        .height_tiles = height_tiles,
    };

    cl_builder_t binning_cl;
    binning_state_t binning_state = cl_init_binning(&binning_cl, width_tiles, height_tiles, p_width, p_height, 1);
    
    uint32_t* frag_shader_code_addr = (uint32_t*) frag_shader_fixed_light;
    uint32_t frag_shader_code_addr_gpu = CPU_TO_BUS(frag_shader_code_addr);

    uint8_t vertex_data_stride = sizeof(nv_vertex_nch_nps_t);
    
    uint16_t total_verts = 0;
    int num_bodies = 20;
    int cube_size = 20;
    int spacing = (p_width - num_bodies * 2 * cube_size) / (num_bodies - 1);

    rigid_body rbs[num_bodies];
    vec3 light_dir = vec3_make(0.0f, 0.0f, -1.0f);

    int total_vertices = NUM_TRIANGLES_PER_BOX * num_bodies * 3;
    nv_vertex_nch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_nch_nps_t *) kmalloc_aligned(total_vertices * sizeof(nv_vertex_nch_nps_t), 16);
    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);

    uint16_t *indices = kmalloc_aligned(total_vertices * sizeof(uint16_t), 4);
    int start_x = -p_width / 2 + cube_size;

    // mesh built once, all bodies share same triangle array pointer
    rigid_body_geom shared_geom;
    phys_geom_init_box(&shared_geom, cube_size, cube_size, cube_size, 1.0f);
    mesh_geom shared_mesh = mesh_geom_init_box(shared_geom.shape.box.hx, shared_geom.shape.box.hy, shared_geom.shape.box.hz, 1.0f, 0.0f, 0.0f);
    shared_geom.shape.mesh = shared_mesh;

    for (int i = 0; i < num_bodies; i++) { 
        phys_body_init(&rbs[i], &shared_geom, vec3_make(start_x + i * (2 * cube_size + spacing), 0.0f, 0.0f), quat_identity());
        total_verts += render_rigid_body(&rbs[i], &cam, light_dir, 1.0f, 0.0f, 0.0f, total_verts, indices, shaded_vertex_data_addr);
    }

    output("total_verts: %d\n", total_verts);
    cl_bin_primitives(&binning_cl, vertex_data_stride, frag_shader_code_addr_gpu, 
shaded_vertex_data_addr_gpu, indices, total_verts);
    cl_bin_one_frame(&binning_cl);
    // // RENDERING PASS
    cl_builder_t rendering_cl;
    cl_init_rendering(&rendering_cl, binning_state.tile_alloc_addr, render_state);
    cl_render_one_frame(&rendering_cl);

    delay_ms(5000);
}
