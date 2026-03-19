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
        vec3_make(0.0f, 0.0f, -800.0f),      // offset above and to the side
        quat_from_euler(0.0f, 0.0f, 0.0f),    // tilt down, rotate left to face origin
        640.0f, 640.0f,
        0.0f, 0.0f);     
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
    int num_bodies = 2;
    rigid_body rbs[num_bodies];
    vec3 light_dir = vec3_make(0.0f, 0.0f, -1.0f);

    int total_vertices = NUM_TRIANGLES_PER_BOX * num_bodies * 3;
    nv_vertex_nch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_nch_nps_t *) kmalloc_aligned(total_vertices * sizeof(nv_vertex_nch_nps_t), 16);
    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);

    uint16_t *indices = kmalloc_aligned(total_vertices * sizeof(uint16_t), 4);

    // for (int i = 0; i < num_bodies; i++) { 
    rigid_body_geom geom; 
    // initialize physical properties - dimension, mass
    phys_geom_init_box(&geom, 20.0f, 20.0f, 20.0f, 1.0f);
    // create the actual mesh geometry and give it a color, store in mesh_geom
    mesh_geom mesh = mesh_geom_init_box(geom.shape.box.hx, geom.shape.box.hy, geom.shape.box.hz, 1.0f, 0.0f, 0.0f);
    geom.shape.mesh = mesh;
    // initialize the rigid body with the geometry and position
    phys_body_init(&rbs[0], &geom, vec3_make(0 * 50.0f + 500.0f, 0.0f, 0.0f), quat_identity());
    // put the rigid body to the nv pipeline
    total_verts += render_rigid_body(&rbs[0], &cam, 0.8f, light_dir, 1.0f, 0.0f, 0.0f, total_verts, indices, shaded_vertex_data_addr);
    // }

    // for (int i = 0; i < num_bodies; i++) { 
    rigid_body_geom geom2; 
    // initialize physical properties - dimension, mass
    phys_geom_init_box(&geom2, 20.0f, 20.0f, 20.0f, 1.0f);
    // create the actual mesh geometry and give it a color, store in mesh_geom
    mesh_geom mesh2 = mesh_geom_init_box(geom2.shape.box.hx, geom2.shape.box.hy, geom2.shape.box.hz, 1.0f, 0.0f, 0.0f);
    geom2.shape.mesh = mesh2;
    // initialize the rigid body with the geometry and position
    phys_body_init(&rbs[1], &geom2, vec3_make(1 * 50.0f + 500.0f, 0.0f, 0.0f), quat_identity());
    // put the rigid body to the nv pipeline
    total_verts += render_rigid_body(&rbs[1], &cam, 0.1f, light_dir, 1.0f, 0.0f, 0.0f, total_verts, indices, shaded_vertex_data_addr);

    output("total_verts: %d\n", total_verts);
    cl_bin_primitives(&binning_cl, vertex_data_stride, frag_shader_code_addr_gpu, 
shaded_vertex_data_addr_gpu, indices, total_verts);
    
    cl_bin_one_frame(&binning_cl);

    // // RENDERING PASS
    cl_builder_t rendering_cl;
    cl_init_rendering(&rendering_cl, binning_state.tile_alloc_addr, render_state);
    cl_render_one_frame(&rendering_cl);

    while(1);
}
