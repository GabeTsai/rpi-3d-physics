#pragma once

#include "rpi.h"
#include "rpi-math.h"
#include <string.h>
#include "vec3.h"
#include "quat.h"
#include "geom.h"

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

int phys_geom_init_sphere(rigid_body_geom *g, float radius, float mass);
int phys_geom_init_box(rigid_body_geom *g, float hx, float hy, float hz, float mass);
int phys_geom_init_capsule(rigid_body_geom *g, float radius, float half_height, float mass);

/* if you do not want to auto-compute mesh inertia yet */
int phys_geom_init_mesh(rigid_body_geom *g, triangle *tris, int tri_count, float mass,
                        vec3 inertia_body);

int phys_body_init(rigid_body *b, const rigid_body_geom *g,
                   vec3 position, quat orientation);
