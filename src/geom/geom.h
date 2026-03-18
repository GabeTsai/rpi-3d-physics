#pragma once

#include "rpi.h"
#include "rpi-math.h"
#include "vec3.h"
#include "quat.h"

typedef struct {
    vec3 v0, v1, v2;
} triangle;

typedef struct {
    int id;
    int triangle_count;
    triangle *triangles;
    vec3 position;
    quat orientation;
} mesh_geom;

mesh_geom mesh_geom_init_custom_take(int triangle_count, triangle *triangles, vec3 position, quat orientation);

mesh_geom mesh_geom_init_custom_copy(int triangle_count, const triangle *triangles, vec3 position, quat orientation);

mesh_geom mesh_geom_init_box(float hx, float hy, float hz, vec3 position, quat orientation);

mesh_geom mesh_geom_init_plane(float hx, float hz, vec3 position, quat orientation);

mesh_geom mesh_geom_init_tetrahedron(float radius, vec3 position, quat orientation);

mesh_geom mesh_geom_init_icosahedron(float radius, vec3 position, quat orientation);

mesh_geom mesh_geom_init_icosphere(float radius, int subdivisions, vec3 position, quat orientation);