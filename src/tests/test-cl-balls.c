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
#include "mailbox-interface.h"
#include "vertex-transform.h"

#define MAX_BALLS 128

static uint32_t rng_state = 0x12345678u;

static uint32_t lcg_rand_u32(void) {
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

static float rand_unit_float(void) {
    uint32_t x = lcg_rand_u32();
    return (float)(x & 0x00FFFFFFu) / (float)0x01000000u;
}

static float rand_color_component(void) {
    return 0.2f + 0.8f * rand_unit_float();
}

void notmain(void) {
    kmalloc_init(20);

    // overclock the arm CPU to 1000 MHz
    output("ARM clock before: %d\n", RPI_clock_get_curhz(ARM_CLK));
    RPI_set_clock_hz(ARM_CLK, 950 * 1000000);
    output("ARM clock after: %d\n", RPI_clock_get_curhz(ARM_CLK));

    output("V3D clock before: %d\n", RPI_clock_get_curhz(V3D_CLK));
    RPI_set_clock_hz(V3D_CLK, 420 * 1000000);
    output("V3D clock after: %d\n", RPI_clock_get_curhz(V3D_CLK));

    RPI_qpu_enable(1);

    const int p_width = 1280;
    const int p_height = 704;
    const int width_tiles = p_width / 64;
    const int height_tiles = p_height / 64;
    const int max_vertices = 80000;
    const float dt = 50.0f * 0.001f;

    const int spawn_every_frames = (int)(1.0f / dt);

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

    uint8_t vertex_data_stride = sizeof(nv_vertex_nch_nps_t);
    vec3 light_dir = vec3_make(0.0f, 0.0f, -1.0f);

    nv_vertex_nch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_nch_nps_t *)kmalloc_aligned(
            max_vertices * sizeof(nv_vertex_nch_nps_t), 16);
    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);

    uint16_t *indices =
        (uint16_t *)kmalloc_aligned(max_vertices * sizeof(uint16_t), 4);

    rigid_body rbs[5 + MAX_BALLS];
    rigid_body *scene_storage[MAX_RIGID_BODIES];
    scene sc;
    scene_init(&sc, scene_storage);

    rigid_body_geom geom_floor;
    rigid_body_geom geom_wall_x;
    rigid_body_geom geom_wall_z;

    mesh_geom mesh_floor  = mesh_geom_init_box(50.0f, 1.0f, 50.0f, 1.0f, 1.0f, 1.0f, 1);
    mesh_geom mesh_wall_x = mesh_geom_init_box(1.0f, 30.0f, 50.0f, 0.8f, 0.8f, 0.8f, 1);
    mesh_geom mesh_wall_z = mesh_geom_init_box(50.0f, 30.0f, 1.0f, 0.8f, 0.8f, 0.8f, 0);

    phys_geom_init(&geom_floor,  mesh_floor,  1000000.0f);
    phys_geom_init(&geom_wall_x, mesh_wall_x, 1000000.0f);
    phys_geom_init(&geom_wall_z, mesh_wall_z, 1000000.0f);

    geom_floor.mesh  = mesh_floor;
    geom_wall_x.mesh = mesh_wall_x;
    geom_wall_z.mesh = mesh_wall_z;

    // floor
    phys_body_init(&rbs[0], &geom_floor, vec3_make(0.0f, -10.0f, 0.0f), quat_identity());

    // left / right walls
    phys_body_init(&rbs[1], &geom_wall_x, vec3_make(-50.0f, 10.0f, 0.0f), quat_identity());
    phys_body_init(&rbs[2], &geom_wall_x, vec3_make( 50.0f, 10.0f, 0.0f), quat_identity());

    // back / front walls
    phys_body_init(&rbs[3], &geom_wall_z, vec3_make(0.0f, 10.0f, -50.0f), quat_identity());
    phys_body_init(&rbs[4], &geom_wall_z, vec3_make(0.0f, 10.0f,  50.0f), quat_identity());

    for (int i = 0; i < 5; i++) {
        rbs[i].state.linear_velocity = vec3_zero();
        rbs[i].state.angular_velocity = vec3_zero();
    }

    scene_clear(&sc);
    scene_add_rigid_body(&sc, &rbs[0]);
    scene_add_rigid_body(&sc, &rbs[1]);
    scene_add_rigid_body(&sc, &rbs[2]);
    scene_add_rigid_body(&sc, &rbs[3]);
    scene_add_rigid_body(&sc, &rbs[4]);

    int ball_count = 0;
    int frame_counter = 0;

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


    unsigned fps_start_usec = timer_get_usec_raw();
    int fps_frames = 0;

    unsigned prof_spawn_usec  = 0;
    unsigned prof_phys_usec   = 0;
    unsigned prof_redraw_usec = 0;
    unsigned prof_bin_usec    = 0;
    unsigned prof_render_usec = 0;

    while (1) {
        if (frame_counter >= spawn_every_frames) {
            printk("SPAWN\n");
            frame_counter = 0;

            if (ball_count < MAX_BALLS && sc.num_rbds < MAX_RIGID_BODIES) {
                unsigned t0 = timer_get_usec_raw();

                int idx = 5 + ball_count;

                float r = rand_color_component();
                float g = rand_color_component();
                float b = rand_color_component();

                rigid_body_geom geom_ball;
                mesh_geom mesh_ball = mesh_geom_init_icosphere(4 * rand_unit_float() + 2, 1, r, g, b, 1);
                phys_geom_init(&geom_ball, mesh_ball, 1.0f);
                geom_ball.mesh = mesh_ball;

                phys_body_init(&rbs[idx], &geom_ball,
                            vec3_make(0.0f, 20.0f, 0.0f),
                            quat_identity());

                rbs[idx].state.linear_velocity = vec3_make(
                            100.0f * rand_unit_float() - 50.0f,
                            20.0f * rand_unit_float() - 10.0f,
                            100.0f * rand_unit_float() - 50.0f);
                rbs[idx].state.angular_velocity =
                    vec3_make(4.0f * rand_unit_float() - 2.0f,
                            4.0f * rand_unit_float() - 2.0f,
                            4.0f * rand_unit_float() - 2.0f);

                phys_body_add_force(&rbs[idx], vec3_make(0.0f, -10.0f, 0.0f));
                scene_add_rigid_body(&sc, &rbs[idx]);

                ball_count++;

                cl_bin_primitives(&binning_cl,
                                vertex_data_stride,
                                frag_shader_code_addr_gpu,
                                shaded_vertex_data_addr_gpu,
                                indices,
                                total_verts);

                prof_spawn_usec += timer_get_usec_raw() - t0;
            }
        }

        {
            unsigned t0 = timer_get_usec_raw();

            scene_integrate_all(&sc, dt);
            scene_resolve_all_collisions(&sc);

            prof_phys_usec += timer_get_usec_raw() - t0;
        }

        {
            unsigned t0 = timer_get_usec_raw();

            total_verts = redraw_scene(&sc,
                                    &cam,
                                    0.8f,
                                    light_dir,
                                    indices,
                                    shaded_vertex_data_addr,
                                    max_vertices);

            prof_redraw_usec += timer_get_usec_raw() - t0;
        }

        {
            unsigned t0 = timer_get_usec_raw();

            clear_flush_count();
            cl_bin_one_frame(&binning_cl);

            prof_bin_usec += timer_get_usec_raw() - t0;
        }

        {
            unsigned t0 = timer_get_usec_raw();

            clear_frame_count();
            cl_render_one_frame(&rendering_cl);

            prof_render_usec += timer_get_usec_raw() - t0;
        }

        {
            unsigned now = timer_get_usec_raw();
            fps_frames++;

            if ((now - fps_start_usec) >= 1000000) {
                unsigned total_profiled =
                    prof_spawn_usec +
                    prof_phys_usec +
                    prof_redraw_usec +
                    prof_bin_usec +
                    prof_render_usec;

                printk("FPS: %d\n", fps_frames);
                printk("bodies=%d verts=%d balls=%d\n", sc.num_rbds, total_verts, ball_count);

                if (total_profiled > 0) {
                    float inv_total = 100.0f / (float)total_profiled;

                    float spawn_pct  = (float)prof_spawn_usec  * inv_total;
                    float phys_pct   = (float)prof_phys_usec   * inv_total;
                    float redraw_pct = (float)prof_redraw_usec * inv_total;
                    float bin_pct    = (float)prof_bin_usec    * inv_total;
                    float render_pct = (float)prof_render_usec * inv_total;

                    printk("profile: spawn=%f phys=%f redraw=%f bin=%f render=%f\n",
                        spawn_pct, phys_pct, redraw_pct, bin_pct, render_pct);

                    printk("usec: spawn=%u phys=%u redraw=%u bin=%u render=%u total=%u\n",
                        prof_spawn_usec,
                        prof_phys_usec,
                        prof_redraw_usec,
                        prof_bin_usec,
                        prof_render_usec,
                        total_profiled);
                }

                fps_frames = 0;
                fps_start_usec = now;

                prof_spawn_usec  = 0;
                prof_phys_usec   = 0;
                prof_redraw_usec = 0;
                prof_bin_usec    = 0;
                prof_render_usec = 0;
            }
        }

        frame_counter++;
    }
}