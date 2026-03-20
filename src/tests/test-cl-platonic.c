#include "rpi.h"
#include "mailbox-interface.h"
#include "cl-interface.h"
#include "cl.h"
#include "geom.h"
#include "nv.h"
#include "v3d.h"
#include "stdint.h"
#include "frag-shader-fixed-light.h"
#include "render.h"

#define NUM_BODIES 5

static void spin_body(rigid_body *rb, vec3 axis, float angle_step) {
    quat dq = quat_from_axis_angle(axis, angle_step);
    rb->state.orientation = quat_normalize(quat_mul(dq, rb->state.orientation));
}

void notmain(void) {
    kmalloc_init(20);
    RPI_qpu_enable(1);

    const int p_width = 1280;
    const int p_height = 704;
    const int width_tiles = p_width / 64;
    const int height_tiles = p_height / 64;
    const int max_vertices = 80000;

    fb_info_t fb = RPI_fb_init(p_width, p_height, p_width, p_height, 32);

    camera cam;
    camera_init(&cam,
        vec3_make(0.0f, 0.0f, -140.0f),
        quat_from_euler(0.0f, 0.0f, 0.0f),
        640.0f, 640.0f,
        0.0f, 0.0f,
        5.0f, 5000.0f);

    render_state_t render_state = {
        .fb_base_addr = fb.base_addr,
        .width_px = p_width,
        .height_px = p_height,
        .width_tiles = width_tiles,
        .height_tiles = height_tiles,
    };

    cl_builder_t binning_cl;
    binning_state_t binning_state =
        cl_init_binning(&binning_cl, width_tiles, height_tiles, p_width, p_height, 0);

    cl_builder_t rendering_cl;
    cl_init_rendering(&rendering_cl, binning_state.tile_alloc_addr, render_state);

    uint32_t *frag_shader_code_addr = (uint32_t *)frag_shader_fixed_light;
    uint32_t frag_shader_code_addr_gpu = CPU_TO_BUS(frag_shader_code_addr);

    uint8_t vertex_data_stride = sizeof(nv_vertex_nch_nps_t);
    vec3 light_dir = vec3_make(0.0f, 0.0f, -1.0f);

    nv_vertex_nch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_nch_nps_t *)kmalloc_aligned(
            max_vertices * sizeof(nv_vertex_nch_nps_t), 16);
    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);

    uint16_t *indices =
        (uint16_t *)kmalloc_aligned(max_vertices * sizeof(uint16_t), 4);

    rigid_body rbs[NUM_BODIES];
    rigid_body_geom geoms[NUM_BODIES];
    rigid_body *scene_storage[MAX_RIGID_BODIES];
    scene sc;
    scene_init(&sc, scene_storage);
    scene_clear(&sc);

    mesh_geom tetra =
        mesh_geom_init_tetrahedron(16.0f, 0.9f, 0.2f, 0.2f, 1);
    mesh_geom cube =
        mesh_geom_init_box(9.23f, 9.23f, 9.23f, 0.2f, 0.9f, 0.2f, 1);
    mesh_geom octa =
        mesh_geom_init_octahedron(16.0f, 0.2f, 0.2f, 0.9f, 1);
    mesh_geom dodeca =
        mesh_geom_init_dodecahedron(16.0f, 0.9f, 0.8f, 0.2f, 1);
    mesh_geom icosa =
        mesh_geom_init_icosahedron(16.0f, 0.8f, 0.2f, 0.9f, 1);

    phys_geom_init(&geoms[0], tetra, 1.0f);
    phys_geom_init(&geoms[1], cube, 1.0f);
    phys_geom_init(&geoms[2], octa, 1.0f);
    phys_geom_init(&geoms[3], dodeca, 1.0f);
    phys_geom_init(&geoms[4], icosa, 1.0f);

    geoms[0].mesh = tetra;
    geoms[1].mesh = cube;
    geoms[2].mesh = octa;
    geoms[3].mesh = dodeca;
    geoms[4].mesh = icosa;

    phys_body_init(&rbs[0], &geoms[0], vec3_make(-80.0f, 0.0f, 0.0f), quat_identity());
    phys_body_init(&rbs[1], &geoms[1], vec3_make(-40.0f, 0.0f, 0.0f), quat_identity());
    phys_body_init(&rbs[2], &geoms[2], vec3_make(  0.0f, 0.0f, 0.0f), quat_identity());
    phys_body_init(&rbs[3], &geoms[3], vec3_make( 40.0f, 0.0f, 0.0f), quat_identity());
    phys_body_init(&rbs[4], &geoms[4], vec3_make( 80.0f, 0.0f, 0.0f), quat_identity());

    for (int i = 0; i < NUM_BODIES; i++) {
        rbs[i].state.linear_velocity = vec3_zero();
        rbs[i].state.angular_velocity = vec3_zero();
        scene_add_rigid_body(&sc, &rbs[i]);
    }

    int total_verts = render_scene(&sc,
                                   &cam,
                                   0.8f,
                                   light_dir,
                                   indices,
                                   shaded_vertex_data_addr);

    cl_bin_primitives(&binning_cl,
                      vertex_data_stride,
                      frag_shader_code_addr_gpu,
                      shaded_vertex_data_addr_gpu,
                      indices,
                      total_verts);

    while (1) {
        spin_body(&rbs[0], vec3_make(0.0f, 1.0f, 0.0f), 0.020f);
        spin_body(&rbs[1], vec3_make(1.0f, 1.0f, 0.0f), 0.018f);
        spin_body(&rbs[2], vec3_make(0.0f, 1.0f, 1.0f), 0.016f);
        spin_body(&rbs[3], vec3_make(1.0f, 0.0f, 1.0f), 0.014f);
        spin_body(&rbs[4], vec3_make(1.0f, 1.0f, 1.0f), 0.012f);

        total_verts = redraw_scene(&sc,
                                   &cam,
                                   0.8f,
                                   light_dir,
                                   indices,
                                   shaded_vertex_data_addr,
                                   max_vertices);

        clear_flush_count();
        cl_bin_one_frame(&binning_cl);

        clear_frame_count();
        cl_render_one_frame(&rendering_cl);
    }
}