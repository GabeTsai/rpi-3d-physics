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
#include "physics.h"

#include "suzanne.h"

// render a rotating Suzanne mesh
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
        vec3_make(0.0f, 0.0f, -6.0f),
        quat_from_euler(0.0f, 0.0f, 0.0f),
        640.0f, 640.0f,
        0.0f, 0.0f, 0.1f, 5000.0f);

    render_state_t render_state = {
        .fb_base_addr = fb.base_addr,
        .width_px = p_width,
        .height_px = p_height,
        .width_tiles = width_tiles,
        .height_tiles = height_tiles,
    };

    cl_builder_t binning_cl;
    binning_state_t binning_state =
        cl_init_binning(&binning_cl, width_tiles, height_tiles,
                        p_width, p_height, 1);

    cl_builder_t rendering_cl;
    cl_init_rendering(&rendering_cl,
                      binning_state.tile_alloc_addr,
                      render_state);

    uint32_t *frag_shader_code_addr = (uint32_t *)frag_shader_fixed_light;
    uint32_t frag_shader_code_addr_gpu = CPU_TO_BUS(frag_shader_code_addr);

    const uint8_t vertex_data_stride = sizeof(nv_vertex_nch_nps_t);
    vec3 light_dir = vec3_make(0.0f, 0.0f, -1.0f);

    mesh_geom mesh_obj =
        mesh_geom_init_obj_from_memory(test_obj, 1.0f, 0.7f, 0.2f, 1);

    if (!mesh_obj.triangles || mesh_obj.triangle_count <= 0) {
        printk("OBJ parse failed\n");
        clean_reboot();
    }

    printk("OBJ loaded: triangles=%d\n", mesh_obj.triangle_count);

    rigid_body rb;
    rigid_body_geom geom_obj;

    phys_geom_init(&geom_obj, mesh_obj, 1.0f);
    geom_obj.mesh = mesh_obj;

    phys_body_init(&rb, &geom_obj,
                   vec3_make(0.0f, 0.0f, 0.0f),
                   quat_identity());

    rb.state.linear_velocity = vec3_zero();
    rb.state.angular_velocity = vec3_zero();

    rigid_body *scene_storage[MAX_RIGID_BODIES];
    scene sc;
    scene_init(&sc, scene_storage);
    scene_clear(&sc);
    scene_add_rigid_body(&sc, &rb);

    const int max_vertices = mesh_obj.triangle_count * 3;

    nv_vertex_nch_nps_t *shaded_vertex_data_addr =
        (nv_vertex_nch_nps_t *)kmalloc_aligned(
            max_vertices * sizeof(nv_vertex_nch_nps_t), 16);
    if (!shaded_vertex_data_addr) {
        printk("failed to alloc shaded vertex buffer\n");
        clean_reboot();
    }

    uint32_t shaded_vertex_data_addr_gpu = CPU_TO_BUS(shaded_vertex_data_addr);

    uint16_t *indices =
        (uint16_t *)kmalloc_aligned(max_vertices * sizeof(uint16_t), 4);
    if (!indices) {
        printk("failed to alloc index buffer\n");
        clean_reboot();
    }

    int total_verts = redraw_scene(&sc,
                                   &cam,
                                   0.8f,
                                   light_dir,
                                   indices,
                                   shaded_vertex_data_addr,
                                   max_vertices);

    printk("initial verts=%d\n", total_verts);

    cl_bin_primitives(&binning_cl,
                      vertex_data_stride,
                      frag_shader_code_addr_gpu,
                      shaded_vertex_data_addr_gpu,
                      indices,
                      total_verts);

    float angle_y = 0.0f;
    rb.state.angular_velocity = vec3_make(0.0f, 6.0f, 1.0f);
    while (1) {
        const float dt = 30.0f * 0.001f;
        const float spin_speed = 1.0f;

        scene_integrate_all(&sc, dt);
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

        clear_flush_count();
        cl_bin_one_frame(&binning_cl);

        clear_frame_count();
        cl_render_one_frame(&rendering_cl);
    }
}