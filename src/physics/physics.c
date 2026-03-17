#include "physics.h"
#include "vec_helpers.h"
#include "quat_helpers.h"

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

int phys_geom_init_sphere(rigid_body_geom *g, float radius, float mass) {
    if (!g || radius <= 0.0f) return -1;

    memset(g, 0, sizeof(*g));

    g->type = PHYS_SHAPE_SPHERE;
    g->shape.sphere.radius = radius;

    if (mass <= 0.0f) {
        return phys_geom_set_mass_properties(g, mass, vec3_zero());
    }

    float Isphere = (2.0f / 5.0f) * mass * radius * radius;
    vec3 inertia = {.x = Isphere, .y = Isphere, .z = Isphere };

    return phys_geom_set_mass_properties(g, mass, inertia);
}

int phys_geom_init_box(rigid_body_geom *g, float hx, float hy, float hz, float mass) {
    if (!g || hx <= 0.0f || hy <= 0.0f || hz <= 0.0f) return -1;

    memset(g, 0, sizeof(*g));

    g->type = PHYS_SHAPE_BOX;
    g->shape.box.hx = hx;
    g->shape.box.hy = hy;
    g->shape.box.hz = hz;

    if (mass <= 0.0f) {
        return phys_geom_set_mass_properties(g, mass, vec3_zero());
    }

    vec3 inertia = {
        .x = (1.0f / 3.0f) * mass * (hy * hy + hz * hz),
        .y = (1.0f / 3.0f) * mass * (hx * hx + hz * hz),
        .z =(1.0f / 3.0f) * mass * (hx * hx + hy * hy)
    };

    return phys_geom_set_mass_properties(g, mass, inertia);
}

int phys_geom_init_capsule(rigid_body_geom *g, float radius, float half_height, float mass) {
    if (!g || radius <= 0.0f || half_height < 0.0f) return -1;

    memset(g, 0, sizeof(*g));

    g->type = PHYS_SHAPE_CAPSULE;
    g->shape.capsule.radius = radius;
    g->shape.capsule.half_height = half_height;

    if (mass <= 0.0f) {
        return phys_geom_set_mass_properties(g, mass, vec3_zero());
    }

    /*
     * Approximate capsule inertia using a solid cylinder whose half-height
     * is (half_height + radius), i.e. total height = 2*(half_height + radius).
     * This is simple and stable even if not exact.
     */
    float approx_half_height = half_height + radius;
    float h = 2.0f * approx_half_height;

    vec3 inertia = {
        .x = (1.0f / 12.0f) * mass * (3.0f * radius * radius + h * h),
        .y =(1.0f / 12.0f) * mass * (3.0f * radius * radius + h * h),
        .z = 0.5f * mass * radius * radius
    };

    return phys_geom_set_mass_properties(g, mass, inertia);
}

int phys_geom_init_mesh(rigid_body_geom *g, triangle *tris, int tri_count, float mass,
                        vec3 inertia_body) {
    if (!g || !tris || tri_count <= 0) return -1;

    memset(g, 0, sizeof(*g));

    g->type = PHYS_SHAPE_MESH;
    g->shape.mesh.triangles = tris;
    g->shape.mesh.triangle_count = tri_count;

    if (mass <= 0.0f) {
        return phys_geom_set_mass_properties(g, mass, vec3_zero());
    }

    if (inertia_body.x < 0.0f || inertia_body.y < 0.0f || inertia_body.z < 0.0f)
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
