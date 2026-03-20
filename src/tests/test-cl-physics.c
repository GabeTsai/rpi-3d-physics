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

// drop a cube on top of another on a floor 
void notmain(void) {
    kmalloc_init(20);
    RPI_qpu_enable(1);

    const int p_width = 1280;
    const int p_height = 704;
    const int width_tiles = p_width / 64;
    const int height_tiles = p_height / 64;

    fb_info_t fb = RPI_fb_init(p_width, p_height, p_width, p_height, 32);

    camera cam;
    camera_init(&cam,
        vec3_make(0.0f, 10.0f, -80.0f),
        quat_from_euler(0.0f, 0.0f, 0.0f),
        640.0f, 640.0f,
        0.0f, 0.0f, 1.0f, 5000.0f);

    render_state_t render_state = {
        .fb_base_addr = fb.base_addr,
        .width_px = p_width,
        .height_px = p_height,
        .width_tiles = width_tiles,
        .height_tiles = height_tiles,
    };

    cl_builder_t binning_cl;
    binning_state_t binning_state =
        cl_init_binning(&binning_cl, width_tiles, height_tiles, p_width, p_height, 1);

    cl_builder_t rendering_cl;
    cl_init_rendering(&rendering_cl, binning_state.tile_alloc_addr, render_state);

    uint32_t *frag_shader_code_addr = (uint32_t *)frag_shader_fixed_light;
    uint32_t frag_shader_code_addr_gpu = CPU_TO_BUS(frag_shader_code_addr);

    const uint8_t vertex_data_stride = sizeof(nv_vertex_nch_nps_t);
    vec3 light_dir = vec3_make(0.0f, 0.0f, -1.0f);

    const int max_vertices = NUM_TRIANGLES_PER_BOX * 3 * 3;

    nv_vertex_nch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_nch_nps_t *)kmalloc_aligned(
            max_vertices * sizeof(nv_vertex_nch_nps_t), 16);
    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);

    uint16_t *indices =
        (uint16_t *)kmalloc_aligned(max_vertices * sizeof(uint16_t), 4);

    rigid_body rbs[3];
    rigid_body *scene_storage[MAX_RIGID_BODIES];
    scene sc;
    scene_init(&sc, scene_storage);

    rigid_body_geom geom_top;
    rigid_body_geom geom_floor;
    rigid_body_geom geom_bottom;

    mesh_geom mesh_top =
        mesh_geom_init_box(2.0f, 2.0f, 2.0f, 1.0f, 0.0f, 0.0f, 1);
    mesh_geom mesh_floor =
        mesh_geom_init_box(50.0f, 1.0f, 50.0f, 0.0f, 1.0f, 0.0f, 1);
    mesh_geom mesh_bottom =
        mesh_geom_init_box(2.0f, 2.0f, 2.0f, 1.0f, 0.0f, 1.0f, 1);

    phys_geom_init(&geom_top, mesh_top, 1.0f);
    phys_geom_init(&geom_floor, mesh_floor, 10000.0f);
    phys_geom_init(&geom_bottom, mesh_bottom, 1.0f);

    geom_top.mesh = mesh_top;
    geom_floor.mesh = mesh_floor;
    geom_bottom.mesh = mesh_bottom;

    phys_body_init(&rbs[2], &geom_top,
                   vec3_make(0.0f, 10.0f, 0.0f), quat_identity());
    phys_body_init(&rbs[1], &geom_floor,
                   vec3_make(0.0f, -10.0f, 0.0f), quat_identity());
    phys_body_init(&rbs[0], &geom_bottom,
                   vec3_make(0.0f, -5.0f, 0.0f), quat_identity());

    rbs[0].state.linear_velocity = vec3_zero();
    rbs[0].state.angular_velocity = vec3_make(1.0f, 0.0f, 0.0f);

    rbs[1].state.linear_velocity = vec3_zero();
    rbs[1].state.angular_velocity = vec3_zero();

    rbs[2].state.linear_velocity = vec3_zero();
    rbs[2].state.angular_velocity = vec3_make(4.0f, 2.0f, 3.0f);

    scene_clear(&sc);
    scene_add_rigid_body(&sc, &rbs[0]);
    scene_add_rigid_body(&sc, &rbs[1]);
    scene_add_rigid_body(&sc, &rbs[2]);

    int total_verts = redraw_scene(&sc,
                                   &cam,
                                   0.8f,
                                   light_dir,
                                   indices,
                                   shaded_vertex_data_addr,
                                   max_vertices);

    cl_bin_primitives(&binning_cl,
                      vertex_data_stride,
                      frag_shader_code_addr_gpu,
                      shaded_vertex_data_addr_gpu,
                      indices,
                      total_verts);

    phys_body_add_force(&rbs[0], vec3_make(0.0f, -30.0f, 0.0f));
    phys_body_add_force(&rbs[2], vec3_make(0.0f, -30.0f, 0.0f));

    while (1) {
        const float dt = 20.0f * 0.001f;

        scene_integrate_all(&sc, dt);
        scene_resolve_all_collisions(&sc);

        total_verts = redraw_scene(&sc,
                                   &cam,
                                   0.8f,
                                   light_dir,
                                   indices,
                                   shaded_vertex_data_addr,
                                   max_vertices);

        cl_bin_primitives(&binning_cl,
                          vertex_data_stride,
                          frag_shader_code_addr_gpu,
                          shaded_vertex_data_addr_gpu,
                          indices,
                          total_verts);

        // printk("B verts=%d\n", total_verts);

        clear_flush_count();
        cl_bin_one_frame(&binning_cl);

        clear_frame_count();
        cl_render_one_frame(&rendering_cl);
        // printk("D\n");

        // delay_ms(5);
    }
}