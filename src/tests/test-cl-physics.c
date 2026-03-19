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
#include "dynamics.h"
#include "collision.h"

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
        vec3_make(0.0f, 0.0f, -800.0f),
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
    
    uint32_t *frag_shader_code_addr = (uint32_t *)frag_shader_fixed_light;
    uint32_t frag_shader_code_addr_gpu = CPU_TO_BUS(frag_shader_code_addr);

    uint8_t vertex_data_stride = sizeof(nv_vertex_nch_nps_t);

    int num_bodies = 3;
    rigid_body rbs[num_bodies];

    vec3 light_dir = vec3_make(0.0f, 0.0f, -1.0f);

    int max_vertices = NUM_TRIANGLES_PER_BOX * num_bodies * 3;

    nv_vertex_nch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_nch_nps_t *)kmalloc_aligned(
            max_vertices * sizeof(nv_vertex_nch_nps_t), 16);
    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);

    uint16_t *indices =
        (uint16_t *)kmalloc_aligned(max_vertices * sizeof(uint16_t), 4);

    rigid_body scene_storage[MAX_RIGID_BODIES];
    scene sc;
    scene_init(&sc, scene_storage);

    rigid_body_geom geom;
    mesh_geom mesh = mesh_geom_init_box(20.0f, 20.0f, 20.0f, 1.0f, 0.0f, 0.0f);
    phys_geom_init(&geom, mesh, 1.0f);

    geom.mesh = mesh;
    phys_body_init(&rbs[2], &geom, vec3_make(00.0f, 100.0f, 0.0f), quat_identity());

    rigid_body_geom geom0;
    mesh_geom mesh0 = mesh_geom_init_box(500.0f, 10.0f, 500.0f, 1.0f, 0.0f, 0.0f);
    phys_geom_init(&geom0, mesh0, 10000.0f);

    geom0.mesh = mesh0;
    phys_body_init(&rbs[1], &geom0, vec3_make(00.0f, -100.0f, 0.0f), quat_identity());

    rigid_body_geom geom1;
    mesh_geom mesh1 = mesh_geom_init_box(20.0f, 20.0f, 20.0f, 20.0f, 0.0f, 1.0f);
    phys_geom_init(&geom1, mesh1, 1.0f);
        
    geom1.mesh = mesh1;
    phys_body_init(&rbs[0], &geom1, vec3_make(100.0f, -50.0f, 0.0f), quat_identity());

    rbs[0].state.linear_velocity = (vec3){ .x = 0.0f, .y = 0.0f, .z = 0.0f };
    rbs[0].state.angular_velocity = (vec3){ .x = 0.1f, .y = 1.0f, .z = 0.0f };

    rbs[2].state.linear_velocity = (vec3){ .x = 0.0f, .y = 0.0f, .z = 0.0f };
    rbs[2].state.angular_velocity = (vec3){ .x = 4.0f, .y = 2.0f, .z = 3.0f };//{ .x = -2.0f, .y = -0.5f, .z = 0.0f };

    cl_builder_t rendering_cl;
    cl_init_rendering(&rendering_cl, binning_state.tile_alloc_addr, render_state);

    // Build scene once to get initial vertex/index data and primitive count.
    scene_clear(&sc);
    scene_add_rigid_body(&sc, &rbs[0]);
    scene_add_rigid_body(&sc, &rbs[1]);
    scene_add_rigid_body(&sc, &rbs[2]);

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

    phys_body_add_force(&rbs[0], vec3_make(0.0f, -20.0f, 0.0f));
    phys_body_add_force(&rbs[2], vec3_make(0.0f, -20.0f, 0.0f));

    while (1) {
        float dt = 20.0f * 0.001f;
        printk("ff\n");


        phys_body_integrate(&rbs[0], dt);
        printk("gg\n");
        phys_body_integrate(&rbs[1], dt);
        printk("hh\n");
        phys_body_integrate(&rbs[2], dt);
        printk("ii\n");

        printk("here\n");

        collision_result res = phys_collide_convex(&rbs[0], &rbs[1]);
            printk("here1.5\n");
        if (res.hit) {
                    printk("hit!\n");

            phys_resolve_collision_basic(&rbs[0], &rbs[1], &res);
        }

        printk("here2\n");


        scene_clear(&sc);
        scene_add_rigid_body(&sc, &rbs[0]);
        scene_add_rigid_body(&sc, &rbs[1]);
        scene_add_rigid_body(&sc, &rbs[2]);
        printk("here3\n");

        total_verts = render_scene(&sc,
                                   &cam,
                                   0.8f,
                                   light_dir,
                                   indices,
                                   shaded_vertex_data_addr);

        clear_flush_count();
        cl_bin_one_frame(&binning_cl);
        // printk("here4");

        clear_frame_count();
        cl_render_one_frame(&rendering_cl);

        printk("hit? %d\n", res.hit);
        printk("depth? %f\n", res.epa.depth);
        delay_ms(10);
    }
}