#pragma once

#include <math.h>
#include <string.h>

typedef struct {
    float x, y, z;
} vec3;

typedef struct {
    vec3 v0, v1, v2;
} tri3;

typedef struct {
    triangle *triangles;
    int triangle_count;
} mesh_geom;

typedef struct {
    float w, x, y, z;
} quat;

typedef struct {
    enum {
        PHYS_SHAPE_NONE = 0,
        PHYS_SHAPE_SPHERE,
        PHYS_SHAPE_BOX,
        PHYS_SHAPE_CAPSULE,
        PHYS_SHAPE_MESH
    } type;

    union {
        struct { float radius; } sphere;
        struct { float hx, hy, hz; } box;
        struct { float radius, half_height; } capsule;
        mesh_geom mesh;
    } shape;

    float mass;
    vec3 inertia_body;
    vec3 inv_inertia_body;
    float inv_mass;
} rigid_body_geom;

typedef struct {
    vec3 position;
    quat orientation;

    vec3 linear_velocity;
    vec3 angular_velocity;

    vec3 linear_acceleration;
    vec3 angular_acceleration;
} rigid_body_state;

typedef struct {
    rigid_body_geom geom;
    rigid_body_state state;

    vec3 force;
    vec3 torque;
} rigid_body;

int phys_body_clear_forces(phys_body *b);
int phys_body_add_force(phys_body *b, vec3 f);
int phys_body_add_torque(phys_body *b, vec3 t);
int phys_body_integrate(phys_body *b, float dt);