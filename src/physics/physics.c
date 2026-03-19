#include "physics.h"
#include "vec3.h"
#include "quat.h"
#include "geom.h"

static vec3 phys_safe_inv_vec3(vec3 v) {
    vec3 out = {.x = 0.0f, .y = 0.0f, .z = 0.0f};

    if (v.x != 0.0f) out.x = 1.0f / v.x;
    if (v.y != 0.0f) out.y = 1.0f / v.y;
    if (v.z != 0.0f) out.z = 1.0f / v.z;

    return out;
}

static int phys_geom_set_mass_properties(rigid_body_geom *g, float mass, vec3 inertia_body) {
    if (!g) return -1;

    g->mass = mass;

    if (mass <= 0.0f) {
        g->inv_mass = 0.0f;
        g->inertia_body = vec3_zero();
        g->inv_inertia_body = vec3_zero();
        return 0;
    }

    g->inv_mass = 1.0f / mass;
    g->inertia_body = inertia_body;
    g->inv_inertia_body = phys_safe_inv_vec3(inertia_body);
    
    return 0;
}

int phys_geom_init(rigid_body_geom *g, mesh_geom mesh, float mass) {
    vec3 inertia_body;

    if (!g) return -1;

    memset(g, 0, sizeof(*g));
    g->mesh = mesh;

    if (mass <= 0.0f) {
        return phys_geom_set_mass_properties(g, mass, vec3_zero());
    }

    inertia_body = mesh_compute_surface_inertia_body(&mesh, mass);

    if (inertia_body.x <= 0.0f || inertia_body.y <= 0.0f || inertia_body.z <= 0.0f)
        return -1;

    return phys_geom_set_mass_properties(g, mass, inertia_body);
}

int phys_body_init(rigid_body *b, const rigid_body_geom *g,
                   vec3 position, quat orientation) {
    if (!b || !g) return -1;

    memset(b, 0, sizeof(*b));

    b->geom = *g;

    b->state.position = position;
    b->state.orientation = quat_normalize(orientation);

    b->state.linear_velocity = vec3_zero();
    b->state.angular_velocity = vec3_zero();
    b->state.linear_acceleration = vec3_zero();
    b->state.angular_acceleration = vec3_zero();

    b->force = vec3_zero();
    b->torque = vec3_zero();

    return 0;
}