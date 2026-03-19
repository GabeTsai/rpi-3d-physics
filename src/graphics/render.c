#include "render.h"

bool proj_tri_to_screen(triangle tri, vec3 *world_points, float *proj_xs, float *proj_ys, 
                        const camera *cam, quat orientation, vec3 position) {
    vec3 local[3] = { tri.v0, tri.v1, tri.v2 }; // coords relative to rigid body center
    for (int i = 0; i < 3; i++) {
        world_points[i] = vec3_add(quat_rotate_vec3(orientation, local[i]), position);
        if (!camera_project_point(cam, world_points[i], &proj_xs[i], &proj_ys[i])) {
            return false;
        }
    }
    return true;
}


int render_rigid_body(const rigid_body *rb, const camera *cam, float zs,
                      vec3 light_dir,
                      float r, float g, float b,
                      int vert_offset,
                      uint16_t *vert_index_list,
                      nv_vertex_nch_nps_t *shaded_vertex_data_addr) {
    mesh_geom mesh = rb->geom.shape.mesh;
    quat q = rb->state.orientation;
    vec3 p = rb->state.position;

    vec3 rgb = vec3_make(r, g, b);
    int count = 0;
    for (int i = 0; i < mesh.triangle_count; i++) {
        vec3 world[3];
        float proj_xs[3], proj_ys[3];
        if (!proj_tri_to_screen(mesh.triangles[i], world, proj_xs, proj_ys, cam, q, p))
            continue;

        vec3 intensity_rgb = vec3_sun_intensity_rgb(light_dir, rgb, world[0], world[1], world[2], AMBIENT_INTENSITY_DEF);
        put_proj_tri_to_nv(proj_xs, proj_ys, zs,
                           intensity_rgb.r, intensity_rgb.g, intensity_rgb.b,
                           vert_offset + count, vert_index_list, shaded_vertex_data_addr);
        count += 3;
    }

    return count;
}