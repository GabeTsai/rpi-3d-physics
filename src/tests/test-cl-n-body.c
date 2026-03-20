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
#include "keyboard.h"
#include "gpio.h"
#include "rpi-interrupts.h"

#define MAX_BALLS 128
#define STATIC_BODIES 0

#define CAM_STEP_TRANS 1.0f
#define CAM_STEP_ROT   0.05f

#define SHOT_BALL_RADIUS  3.0f
#define SHOT_BALL_SPEED   10.0f
#define SHOT_SPAWN_OFFSET 30.0f

#define GRAVITY_G               2000.0f
#define GRAVITY_SOFTENING       4.0f
#define GRAVITY_MIN_DIST2       1e-4f
#define GRAVITY_MAX_FORCE       500.0f

static int key_down_w = 0;
static int key_down_a = 0;
static int key_down_s = 0;
static int key_down_d = 0;
static int key_down_o = 0;
static int key_down_l = 0;
static int key_down_up = 0;
static int key_down_down = 0;
static int key_down_left = 0;
static int key_down_right = 0;

static int shoot_ball_requested = 0;

static void update_camera_key_state(key_event_t ev) {
    printk("ere\n");
    int is_down = (ev.action.what == KEY_PRESS);
    unsigned char key = ev.key.ch;

    if (key == 'a') {
        key_down_a = is_down;
    } else if (key == 'd') {
        key_down_d = is_down;
    } else if (key == 'w') {
        key_down_w = is_down;
    } else if (key == 's') {
        key_down_s = is_down;
    } else if (key == 'o') {
        key_down_o = is_down;
    } else if (key == 'l') {
        key_down_l = is_down;
    } else if (key == 'p') {
        if (is_down)
            shoot_ball_requested = 1;
    } else if (key == PS2_KEY_ARROW_UP) {
        key_down_up = is_down;
    } else if (key == PS2_KEY_ARROW_DOWN) {
        key_down_down = is_down;
    } else if (key == PS2_KEY_ARROW_LEFT) {
        key_down_left = is_down;
    } else if (key == PS2_KEY_ARROW_RIGHT) {
        key_down_right = is_down;
    }
}

static void apply_camera_movement(camera *cam) {
    if (key_down_a) {
        cam->position.x -= CAM_STEP_TRANS;
    }
    if (key_down_d) {
        cam->position.x += CAM_STEP_TRANS;
    }
    if (key_down_w) {
        cam->position.y += CAM_STEP_TRANS;
    }
    if (key_down_s) {
        cam->position.y -= CAM_STEP_TRANS;
    }
    if (key_down_o) {
        cam->position.z += CAM_STEP_TRANS;
    }
    if (key_down_l) {
        cam->position.z -= CAM_STEP_TRANS;
    }
    if (key_down_up) {
        quat dq = quat_from_axis_angle(vec3_make(1.0f, 0.0f, 0.0f), CAM_STEP_ROT);
        cam->rotation = quat_normalize(quat_mul(dq, cam->rotation));
    }
    if (key_down_down) {
        quat dq = quat_from_axis_angle(vec3_make(1.0f, 0.0f, 0.0f), -CAM_STEP_ROT);
        cam->rotation = quat_normalize(quat_mul(dq, cam->rotation));
    }
    if (key_down_left) {
        quat dq = quat_from_axis_angle(vec3_make(0.0f, 1.0f, 0.0f), -CAM_STEP_ROT);
        cam->rotation = quat_normalize(quat_mul(dq, cam->rotation));
    }
    if (key_down_right) {
        quat dq = quat_from_axis_angle(vec3_make(0.0f, 1.0f, 0.0f), CAM_STEP_ROT);
        cam->rotation = quat_normalize(quat_mul(dq, cam->rotation));
    }
}

static void poll_keyboard_and_move_camera(camera *cam) {
    int limit = 32;

    while (limit-- > 0 && keyboard_event_available()) {
        key_event_t ev = keyboard_read_event();
        update_camera_key_state(ev);
    }

    apply_camera_movement(cam);
}

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

static vec3 camera_forward_world(const camera *cam) {
    vec3 f = quat_rotate_vec3(cam->rotation, vec3_make(0.0f, 0.0f, 1.0f));
    return vec3_normalize(f);
}

static int spawn_shot_ball(scene *sc,
                           rigid_body *rbs,
                           rigid_body_geom *ball_geoms,
                           int *ball_count_io,
                           const camera *cam)
{
    if (*ball_count_io >= MAX_BALLS)
        return 0;
    if (sc->num_rbds >= MAX_RIGID_BODIES)
        return 0;

    int idx = STATIC_BODIES + *ball_count_io;

    float r = rand_color_component();
    float g = rand_color_component();
    float b = rand_color_component();

    mesh_geom mesh_ball = mesh_geom_init_icosphere(SHOT_BALL_RADIUS, 1, r, g, b, 1);
    phys_geom_init(&ball_geoms[*ball_count_io], mesh_ball, 1.0f);
    ball_geoms[*ball_count_io].mesh = mesh_ball;

    vec3 forward = camera_forward_world(cam);
    vec3 spawn_pos = vec3_add(cam->position, vec3_scale(forward, SHOT_SPAWN_OFFSET));
    vec3 shot_vel = vec3_scale(forward, SHOT_BALL_SPEED);

    phys_body_init(&rbs[idx], &ball_geoms[*ball_count_io], spawn_pos, quat_identity());

    rbs[idx].state.linear_velocity = shot_vel;
    rbs[idx].state.angular_velocity = vec3_make(
        4.0f * rand_unit_float() - 2.0f,
        4.0f * rand_unit_float() - 2.0f,
        4.0f * rand_unit_float() - 2.0f
    );

    // phys_body_add_force(&rbs[idx], vec3_make(0.0f, -10.0f, 0.0f));

    scene_add_rigid_body(sc, &rbs[idx]);
    (*ball_count_io)++;
    return 1;
}

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static void apply_pairwise_gravity(scene *sc) {
    for (int i = 0; i < sc->num_rbds; i++) {
        phys_body_set_force(sc->rbds[i], vec3_make(0.0f, 0.0f, 0.0f));
    }
    for (int i = 0; i < sc->num_rbds; i++) {
        rigid_body *a = sc->rbds[i];

        for (int j = i + 1; j < sc->num_rbds; j++) {
            rigid_body *b = sc->rbds[j];

            vec3 delta = vec3_sub(b->state.position, a->state.position);
            float dist2 = vec3_dot(delta, delta);

            if (dist2 < GRAVITY_MIN_DIST2)
                dist2 = GRAVITY_MIN_DIST2;

            /* softening so force does not explode at tiny separation */
            float softened_dist2 = dist2 + GRAVITY_SOFTENING * GRAVITY_SOFTENING;
            float dist = sqrtf(softened_dist2);

            if (dist <= 1e-6f)
                continue;

            vec3 dir = vec3_scale(delta, 1.0f / dist);

            /* F = G m1 m2 / r^2 */
            float m1 = a->geom.mass;
            float m2 = b->geom.mass;
            float fmag = (GRAVITY_G * m1 * m2) / softened_dist2;

            /* optional safety clamp */
            fmag = clampf(fmag, 0.0f, GRAVITY_MAX_FORCE);

            vec3 f = vec3_scale(dir, fmag);

            phys_body_add_force(a, f);
            phys_body_add_force(b, vec3_scale(f, -1.0f));
        }
    }
}

void notmain(void) {
    kmalloc_init(20);
    RPI_qpu_enable(1);

    const int p_width = 1280;
    const int p_height = 704;
    const int width_tiles = p_width / 64;
    const int height_tiles = p_height / 64;
    const int max_vertices = 80000;
    const float dt = 20.0f * 0.001f;

    fb_info_t fb = RPI_fb_init(p_width, p_height, p_width, p_height, 32);

    camera cam;
    camera_init(&cam,
        vec3_make(0.0f, 10.0f, -80.0f),
        quat_from_euler(0.0f, 0.0f, 0.0f),
        640.0f, 640.0f,
        0.0f, 0.0f, 5.0f, 5000.0f);

    interrupt_init();
    keyboard_init(KEYBOARD_CLOCK, KEYBOARD_DATA);
    keyboard_use_interrupts();
    enable_interrupts();

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

    rigid_body rbs[STATIC_BODIES + MAX_BALLS];
    rigid_body_geom ball_geoms[MAX_BALLS];

    rigid_body *scene_storage[MAX_RIGID_BODIES];
    scene sc;
    scene_init(&sc, scene_storage);

    scene_clear(&sc);


    int ball_count = 0;

    int total_verts = render_scene(&sc,
                                &cam,
                                0.8f,
                                light_dir,
                                indices,
                                shaded_vertex_data_addr);

    if (total_verts > 0) {
        cl_bin_primitives(&binning_cl,
                        vertex_data_stride,
                        frag_shader_code_addr_gpu,
                        shaded_vertex_data_addr_gpu,
                        indices,
                        total_verts);
    }
    unsigned fps_start_usec = timer_get_usec_raw();
    int fps_frames = 0;

    unsigned prof_spawn_usec  = 0;
    unsigned prof_phys_usec   = 0;
    unsigned prof_redraw_usec = 0;
    unsigned prof_bin_usec    = 0;
    unsigned prof_render_usec = 0;

    while (1) {
        poll_keyboard_and_move_camera(&cam);

        if (shoot_ball_requested) {
            shoot_ball_requested = 0;

            if (spawn_shot_ball(&sc, rbs, ball_geoms, &ball_count, &cam)) {
                unsigned t0 = timer_get_usec_raw();

                total_verts = redraw_scene(&sc,
                                           &cam,
                                           0.8f,
                                           light_dir,
                                           indices,
                                           shaded_vertex_data_addr,
                                           max_vertices);
                if (total_verts > 0) {
                    cl_bin_primitives(&binning_cl,
                                    vertex_data_stride,
                                    frag_shader_code_addr_gpu,
                                    shaded_vertex_data_addr_gpu,
                                    indices,
                                    total_verts);
                }

                prof_spawn_usec += timer_get_usec_raw() - t0;
            }
        }

        {
            unsigned t0 = timer_get_usec_raw();

            apply_pairwise_gravity(&sc);
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
    }
}