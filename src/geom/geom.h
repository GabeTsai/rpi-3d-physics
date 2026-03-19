#pragma once

#include "rpi.h"
#include "rpi-math.h"
#include "vec3.h"
#include "quat.h"

#define NUM_TRIANGLES_PER_BOX 12
#define NUM_TRIANGLES_PER_ICOSPHERE 20

#define NUM_TRIANGLES_PER_SPHERE(subdivisions) (NUM_TRIANGLES_PER_ICOSPHERE * (int)powf(4, subdivisions))

typedef struct {
    vec3 v0, v1, v2;
} triangle;

typedef struct {
    int id;
    int triangle_count;
    triangle *triangles;
    float r, g, b;
} mesh_geom;

static inline float triangle_area(triangle t) {
    vec3 ab = vec3_sub(t.v1, t.v0);
    vec3 ac = vec3_sub(t.v2, t.v0);
    return 0.5f * vec3_norm(vec3_cross(ab, ac));
}

static inline vec3 triangle_centroid(triangle t) {
    return vec3_scale(vec3_add(vec3_add(t.v0, t.v1), t.v2), 1.0f / 3.0f);
}

triangle triangle_make_from_pts(float xs_1, float ys_1, float xs_2, float ys_2, float xs_3, float ys_3, float zs);

static inline int mesh_surface_com(const mesh_geom *mesh, float total_mass, vec3 *out_com, float *out_total_area) {
    if (!mesh || !out_com || !out_total_area || !mesh->triangles || mesh->triangle_count <= 0)
        return -1;

    float total_area = 0.0f;
    vec3 weighted = vec3_zero();

    for (int i = 0; i < mesh->triangle_count; i++) {
        triangle t = mesh->triangles[i];
        float A = triangle_area(t);
        vec3 c = triangle_centroid(t);

        total_area += A;
        weighted = vec3_add(weighted, vec3_scale(c, A));
    }

    if (total_area <= 1e-8f)
        return -1;

    *out_com = vec3_scale(weighted, 1.0f / total_area);
    *out_total_area = total_area;
    return 0;
}

static vec3 mesh_compute_surface_inertia_body(const mesh_geom *mesh, float total_mass) {
    vec3 inertia = vec3_zero();
    vec3 com;
    float total_area;

    if (!mesh || !mesh->triangles || mesh->triangle_count <= 0 || total_mass <= 0.0f)
        return inertia;

    if (mesh_surface_com(mesh, total_mass, &com, &total_area) < 0)
        return inertia;

    for (int i = 0; i < mesh->triangle_count; i++) {
        triangle t = mesh->triangles[i];
        float A = triangle_area(t);
        if (A <= 1e-8f)
            continue;

        float mt = total_mass * (A / total_area);
        float mp = mt / 3.0f;

        vec3 verts[3] = { t.v0, t.v1, t.v2 };

        for (int k = 0; k < 3; k++) {
            vec3 r = vec3_sub(verts[k], com);
            float x = r.x, y = r.y, z = r.z;

            inertia.x += mp * (y * y + z * z);
            inertia.y += mp * (x * x + z * z);
            inertia.z += mp * (x * x + y * y);
        }
    }

    return inertia;
}

mesh_geom mesh_geom_init_custom_take(int triangle_count, triangle *triangles, float r, float g, float b);

mesh_geom mesh_geom_init_custom_copy(int triangle_count, const triangle *triangles, float r, float g, float b);

mesh_geom mesh_geom_init_triangle(triangle tri, float r, float g, float b);

mesh_geom mesh_geom_init_box(float hx, float hy, float hz, float r, float g, float b);

mesh_geom mesh_geom_init_plane(float hx, float hz, float r, float g, float b);

mesh_geom mesh_geom_init_tetrahedron(float radius, float r, float g, float b);

mesh_geom mesh_geom_init_icosahedron(float radius, float r, float g, float b);

mesh_geom mesh_geom_init_icosphere(float radius, int subdivisions, float r, float g, float b);