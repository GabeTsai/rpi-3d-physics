#include "render.h"

bool proj_tri_to_screen(triangle tri, vec3 *world_points,
                        float *proj_xs, float *proj_ys, float *proj_zs,
                        const camera *cam, quat orientation, vec3 position) {
    vec3 local[3] = { tri.v0, tri.v1, tri.v2 };

    for (int i = 0; i < 3; i++) {
        world_points[i] = vec3_add(quat_rotate_vec3(orientation, local[i]), position);
        if (!camera_project_point(cam, world_points[i],
                                  &proj_xs[i], &proj_ys[i], &proj_zs[i])) {
            return false;
        }
    }
    return true;
}

// Projects a triangle using precomputed combined rotation and translation (see render_rigid_body):
// Per vertex:  p_cam = rotate(combined_q, local_v) + body_pos_cam
// Then perspective divide. World_points still filled for flat-shading callers.
static bool proj_tri_to_screen_fast(triangle tri, vec3 *world_points,
                                    float *proj_xs, float *proj_ys, float *proj_zs,
                                    const camera *cam,
                                    quat combined_q, vec3 body_pos_cam,
                                    quat orientation, vec3 position) {
    for (int i = 0; i < 3; i++) {
        const vec3 *lv = (i == 0) ? &tri.v0 : (i == 1) ? &tri.v1 : &tri.v2;

        vec3 p_cam = vec3_add(quat_rotate_vec3(combined_q, *lv), body_pos_cam);

        float z = p_cam.z;
        if (z <= cam->znear || z >= cam->zfar)
            return false;

        float inv_z = 1.0f / z;
        proj_xs[i] = cam->fx * (p_cam.x * inv_z) + cam->cx;
        proj_ys[i] = cam->cy - cam->fy * (p_cam.y * inv_z);
        proj_zs[i] = (z - cam->znear) * inv_z;

        // world point is only needed for flat shading; Gouraud shading uses object-space normals
        if (world_points)
            world_points[i] = vec3_add(quat_rotate_vec3(orientation, *lv), position);
    }
    return true;
}

int render_rigid_body(const rigid_body *rb, const camera *cam,
                      vec3 light_dir,
                      float r, float g, float b,
                      int vert_offset,
                      uint16_t *vert_index_list,
                      nv_vertex_nch_nps_t *shaded_vertex_data_addr) {
    if (!rb->geom.mesh.visible) return 0;

    mesh_geom mesh = rb->geom.mesh;
    quat q = rb->state.orientation;
    vec3 p = rb->state.position;
    vec3 rgb = vec3_make(r, g, b);

    // Precompute combined body+camera transform once per body
    // combined_q   = cam_inv * body_q  so that rotate(combined_q, local_v) gives camera-space coords
    // body_pos_cam = rotate(cam_inv, body_pos - cam_pos)  is the body origin in camera space
    // collapses two rotations per vertex into one
    quat combined_q   = quat_mul(cam->inv_rotation, q);
    vec3 body_pos_cam = quat_rotate_vec3(cam->inv_rotation, vec3_sub(p, cam->position));

#if GOURAUD_SHADING
    vec3 light_obj = vec3_zero();
    if (mesh.smooth_normals)
        light_obj = quat_rotate_vec3(quat_conjugate(q), vec3_normalize(light_dir));
#endif

    int count = 0;

    // Gouraud-shaded bodies lighting is computed in object space so world positions not needed
    bool needs_world =
#if GOURAUD_SHADING
        !mesh.smooth_normals;
#else
        true;
#endif

    for (int i = 0; i < mesh.triangle_count; i++) {
        vec3 world[3];
        float proj_xs[3], proj_ys[3], proj_zs[3];

        if (!proj_tri_to_screen_fast(mesh.triangles[i], needs_world ? world : NULL,
                                     proj_xs, proj_ys, proj_zs,
                                     cam, combined_q, body_pos_cam, q, p))
            continue;

        vec3 c0, c1, c2;

#if GOURAUD_SHADING
        if (mesh.smooth_normals) {
            triangle tri = mesh.triangles[i];
            c0 = vec3_scale(rgb, fmaxf(AMBIENT_INTENSITY_DEF, vec3_dot(light_obj, tri.n0)));
            c1 = vec3_scale(rgb, fmaxf(AMBIENT_INTENSITY_DEF, vec3_dot(light_obj, tri.n1)));
            c2 = vec3_scale(rgb, fmaxf(AMBIENT_INTENSITY_DEF, vec3_dot(light_obj, tri.n2)));
        } else
#endif
        {
            c0 = c1 = c2 = vec3_sun_intensity_rgb(
                light_dir, rgb, world[0], world[1], world[2], AMBIENT_INTENSITY_DEF);
        }

        put_proj_tri_to_nv(proj_xs, proj_ys, proj_zs,
                           c0, c1, c2,
                           vert_offset + count,
                           vert_index_list,
                           shaded_vertex_data_addr);
        count += 3;
    }

    return count;
}

void scene_clear(scene *s) {
    if (!s) return;
    s->num_rbds = 0;
}

int scene_add_rigid_body(scene *s, rigid_body *rb) {
    if (!s || !rb) return -1;
    if (s->num_rbds >= MAX_RIGID_BODIES) return -1;

    s->rbds[s->num_rbds] = rb;
    s->num_rbds++;
    return 0;
}

int render_scene(const scene *s,
                 const camera *cam,
                 float zs,
                 vec3 light_dir,
                 uint16_t *vert_index_list,
                 nv_vertex_nch_nps_t *shaded_vertex_data_addr) {
    (void)zs;

    if (!s || !cam || !vert_index_list || !shaded_vertex_data_addr)
        return -1;

    int total_verts = 0;

    for (int i = 0; i < s->num_rbds; i++) {
        rigid_body *rb = s->rbds[i];
        if (!rb)
            continue;

        int written = render_rigid_body(rb,
                                        cam,
                                        light_dir,
                                        rb->geom.mesh.r,
                                        rb->geom.mesh.g,
                                        rb->geom.mesh.b,
                                        total_verts,
                                        vert_index_list,
                                        shaded_vertex_data_addr);
        if (written < 0)
            return -1;

        total_verts += written;
    }

    return total_verts;
}

int redraw_scene(const scene *s,
                 const camera *cam,
                 float zs,
                 vec3 light_dir,
                 uint16_t *vert_index_list,
                 nv_vertex_nch_nps_t *shaded_vertex_data_addr,
                 int max_vertices) {
    if (!s || !cam || !vert_index_list || !shaded_vertex_data_addr || max_vertices <= 0)
        return -1;

    return render_scene(s, cam, zs, light_dir, vert_index_list, shaded_vertex_data_addr);
}

int scene_resolve_collision_pair(rigid_body *a, rigid_body *b) {
    if (!a || !b) return 0;

    collision_result res = phys_collide_convex(a, b);
    if (res.hit) {
        phys_resolve_collision_basic(a, b, &res);
        return 1;
    }
    return 0;
}

int scene_resolve_all_collisions(scene *s) {
    if (!s) return -1;

    int hits = 0;

    for (int i = 0; i < s->num_rbds; i++) {
        rigid_body *a = s->rbds[i];
        if (!a) continue;

        for (int j = i + 1; j < s->num_rbds; j++) {
            rigid_body *b = s->rbds[j];
            if (!b) continue;

            hits += scene_resolve_collision_pair(a, b);
        }
    }

    return hits;
}

void scene_integrate_all(scene *s, float dt) {
    if (!s) return;

    for (int i = 0; i < s->num_rbds; i++) {
        rigid_body *rb = s->rbds[i];
        if (!rb) continue;
        phys_body_integrate(rb, dt);
    }
}