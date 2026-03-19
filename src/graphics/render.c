#include "render.h"

bool proj_tri_to_screen(triangle tri, vec3 *world_points, 
                        float *proj_xs, float *proj_ys, float *proj_zs,
                        const camera *cam, quat orientation, vec3 position) {
    vec3 local[3] = { tri.v0, tri.v1, tri.v2 }; // coords relative to rigid body center
    for (int i = 0; i < 3; i++) {
        world_points[i] = vec3_add(quat_rotate_vec3(orientation, local[i]), position);
        if (!camera_project_point(cam, world_points[i], &proj_xs[i], &proj_ys[i], &proj_zs[i])) {
            return false;
        }
    }
    return true;
}

int render_rigid_body(const rigid_body *rb, const camera *cam,
                      vec3 light_dir,
                      float r, float g, float b,
                      int vert_offset,
                      uint16_t *vert_index_list,
                      nv_vertex_nch_nps_t *shaded_vertex_data_addr) {
    mesh_geom mesh = rb->geom.mesh;
    quat q = rb->state.orientation;
    vec3 p = rb->state.position;

    vec3 rgb = vec3_make(r, g, b);
    int count = 0;
    for (int i = 0; i < mesh.triangle_count; i++) {
        vec3 world[3];
        float proj_xs[3], proj_ys[3], proj_zs[3];
        if (!proj_tri_to_screen(mesh.triangles[i], world, proj_xs, proj_ys, proj_zs, cam, q, p))
            continue;

        vec3 intensity_rgb = vec3_sun_intensity_rgb(light_dir, rgb, world[0], world[1], world[2], AMBIENT_INTENSITY_DEF);
        put_proj_tri_to_nv(proj_xs, proj_ys, proj_zs,
                           intensity_rgb.r, intensity_rgb.g, intensity_rgb.b,
                           vert_offset + count, vert_index_list, shaded_vertex_data_addr);
        count += 3;
    }

    return count;
}

void scene_clear(scene *s) {
    if (!s) return;
    s->num_rbds = 0;
}

int scene_add_rigid_body(scene *s, const rigid_body *rb) {
    if (!s || !rb) return -1;
    if (s->num_rbds >= MAX_RIGID_BODIES) return -1;

    s->rbds[s->num_rbds] = *rb;
    s->num_rbds++;
    return 0;
}

int render_scene(const scene *s,
                 const camera *cam,
                 float zs,
                 vec3 light_dir,
                 uint16_t *vert_index_list,
                 nv_vertex_nch_nps_t *shaded_vertex_data_addr) {
    if (!s || !cam || !vert_index_list || !shaded_vertex_data_addr)
        return -1;

    int total_verts = 0;

    for (int i = 0; i < s->num_rbds; i++) {
        int written = render_rigid_body(&s->rbds[i],
                                                 cam,
                                                 light_dir, 1.0f, 0.0f, 0.0f,
                                                 total_verts,
                                                 vert_index_list,
                                                 shaded_vertex_data_addr);
        if (written < 0)
            return -1;

        total_verts += written;
    }

    return total_verts;
}