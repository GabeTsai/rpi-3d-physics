#pragma once

#include "rpi.h"
#include "rpi-math.h"
#include <string.h>
#include "vec3.h"
#include "quat.h"
#include "geom.h"

typedef struct {
    mesh_geom mesh;
    
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


/* if you do not want to auto-compute mesh inertia yet */
int phys_geom_init(rigid_body_geom *g, mesh_geom mesh, float mass);

int phys_body_init(rigid_body *b, const rigid_body_geom *g,
                   vec3 position, quat orientation);
