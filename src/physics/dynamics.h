
#include "vec3.h"
#include "quat.h"

static vec3 phys_compute_linear_acceleration(const rigid_body *b) {
    return vec3_scale(b->force, b->geom.inv_mass);
}

static vec3 phys_compute_angular_acceleration(const rigid_body *b) {
    return vec3_hadamard(b->torque, b->geom.inv_inertia_body);
}

int phys_body_clear_forces(rigid_body *b) {
    if (!b) return -1;

    b->force = vec3_zero();
    b->torque = vec3_zero();
    return 0;
}

int phys_body_add_force(rigid_body *b, vec3 f) {
    if (!b) return -1;

    b->force = vec3_add(b->force, f);
    return 0;
}

int phys_body_add_torque(rigid_body *b, vec3 t) {
    if (!b) return -1;

    b->torque = vec3_add(b->torque, t);
    return 0;
}

int phys_body_integrate(rigid_body *b, float dt) {
    if (!b || dt <= 0.0f) return -1;

    b->state.linear_acceleration = phys_compute_linear_acceleration(b);
    b->state.angular_acceleration = phys_compute_angular_acceleration(b);

    b->state.linear_velocity =
        vec3_add(b->state.linear_velocity,
                 vec3_scale(b->state.linear_acceleration, dt));

    b->state.angular_velocity =
        vec3_add(b->state.angular_velocity,
                 vec3_scale(b->state.angular_acceleration, dt));

    b->state.position =
        vec3_add(b->state.position,
                 vec3_scale(b->state.linear_velocity, dt));

    quat omega = quat_from_angular_velocity(b->state.angular_velocity);
    quat qdot = quat_scale(quat_mul(omega, b->state.orientation), 0.5f);

    b->state.orientation =
        quat_normalize(
            quat_add(b->state.orientation,
                     quat_scale(qdot, dt))
        );

    return 0;
}