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
 two rows of 10 icospheres: bottom row at y=0, top row at y=50, same x positions.
 each row is a separate draw call with its own vertex buffer and index list.
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
        vec3_make(0.0f, 0.0f, -350.0f),
        quat_from_euler(0.0f, 0.0f, 0.0f),
        900.0f, 900.0f,
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

    int batch_size = 10;
    int radius = 20;
    int subdivisions = 2;
    vec3 light_dir = vec3_make(0.0f, 0.0f, -1.0f);

    int verts_per_batch = NUM_TRIANGLES_PER_SPHERE(subdivisions) * batch_size * 3;

    // two independent vertex buffers — indices in each reset to 0
    nv_vertex_nch_nps_t *verts1 = (nv_vertex_nch_nps_t *) kmalloc_aligned(verts_per_batch * sizeof(nv_vertex_nch_nps_t), 16);
    nv_vertex_nch_nps_t *verts2 = (nv_vertex_nch_nps_t *) kmalloc_aligned(verts_per_batch * sizeof(nv_vertex_nch_nps_t), 16);
    uint16_t *idx1 = kmalloc_aligned(verts_per_batch * sizeof(uint16_t), 4);
    uint16_t *idx2 = kmalloc_aligned(verts_per_batch * sizeof(uint16_t), 4);

    // mesh built once, shared across all 20 bodies
    rigid_body_geom shared_geom;
    phys_geom_init_sphere(&shared_geom, radius, 1.0f);
    mesh_geom shared_mesh = mesh_geom_init_icosphere(radius, subdivisions, 1.0f, 0.0f, 0.0f);
    shared_geom.shape.mesh = shared_mesh;

    rigid_body rbs[batch_size * 2];

    // batch 1: bottom row at y = 0
    int count1 = 0;
    for (int i = 0; i < batch_size; i++) {
        phys_body_init(&rbs[i], &shared_geom, vec3_make(i * 2 * (radius + 2) - 200, 0.0f, 0.0f), quat_identity());
        count1 += render_rigid_body(&rbs[i], &cam, light_dir, 1.0f, 0.0f, 0.0f, count1, idx1, verts1);
    }

    // batch 2: top row at y = 50, vert_offset resets to 0 for the new buffer
    int count2 = 0;
    for (int i = 0; i < batch_size; i++) {
        phys_body_init(&rbs[batch_size + i], &shared_geom, vec3_make(i * 2 * (radius + 2) - 200, 50.0f, 0.0f), quat_identity());
        count2 += render_rigid_body(&rbs[batch_size + i], &cam, light_dir, 1.0f, 0.0f, 0.0f, count2, idx2, verts2);
    }

    output("count1: %d  count2: %d\n", count1, count2);
    cl_bin_primitives(&binning_cl, vertex_data_stride, frag_shader_code_addr_gpu, CPU_TO_BUS(verts1), idx1, count1);
    cl_bin_primitives(&binning_cl, vertex_data_stride, frag_shader_code_addr_gpu, CPU_TO_BUS(verts2), idx2, count2);

    cl_bin_one_frame(&binning_cl);
    // // RENDERING PASS
    cl_builder_t rendering_cl;
    cl_init_rendering(&rendering_cl, binning_state.tile_alloc_addr, render_state);
    cl_render_one_frame(&rendering_cl);

    while(1);
}
