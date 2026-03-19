#pragma once

#include "rpi.h"
#include "rpi-math.h"
#include "vec3.h"
#include "quat.h"

#define NUM_TRIANGLES_PER_BOX 12

typedef struct {
    vec3 v0, v1, v2;
} triangle;

typedef struct {
    int id;
    int triangle_count;
    triangle *triangles;
    float r, g, b;
} mesh_geom;

triangle triangle_make_from_pts(float xs_1, float ys_1, float xs_2, float ys_2, float xs_3, float ys_3, float zs);

mesh_geom mesh_geom_init_custom_take(int triangle_count, triangle *triangles, float r, float g, float b);

mesh_geom mesh_geom_init_custom_copy(int triangle_count, const triangle *triangles, float r, float g, float b);

mesh_geom mesh_geom_init_triangle(triangle tri, float r, float g, float b);

mesh_geom mesh_geom_init_box(float hx, float hy, float hz, float r, float g, float b);

mesh_geom mesh_geom_init_plane(float hx, float hz, float r, float g, float b);

mesh_geom mesh_geom_init_tetrahedron(float radius, float r, float g, float b);

mesh_geom mesh_geom_init_icosahedron(float radius, float r, float g, float b);

mesh_geom mesh_geom_init_icosphere(float radius, int subdivisions, float r, float g, float b);