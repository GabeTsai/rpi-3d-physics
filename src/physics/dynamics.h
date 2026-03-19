
#include "vec3.h"
#include "quat.h"
#include "collision.h"

static vec3 phys_compute_linear_acceleration(const rigid_body *b) {
    return vec3_scale(b->force, b->geom.inv_mass);
}

static vec3 phys_apply_inv_inertia_world(const rigid_body *b, vec3 v_world) {
    quat q = b->state.orientation;
    quat qc = quat_conjugate(q);

    vec3 v_body = quat_rotate_vec3(qc, v_world);
    vec3 out_body = vec3_hadamard(v_body, b->geom.inv_inertia_body);
    return quat_rotate_vec3(q, out_body);
}

static vec3 phys_compute_angular_acceleration(const rigid_body *b) {
    return phys_apply_inv_inertia_world(b, b->torque);
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

printk("lin vel = (%f, %f, %f)\n",
    b->state.linear_velocity.x,
    b->state.linear_velocity.y,
    b->state.linear_velocity.z);

    printk("ang vel = (%f, %f, %f)\n",
    b->state.angular_velocity.x,
    b->state.angular_velocity.y,
    b->state.angular_velocity.z);

    b->state.linear_velocity =
        vec3_add(b->state.linear_velocity,
                 vec3_scale(b->state.linear_acceleration, dt));

    b->state.angular_velocity =
        vec3_add(b->state.angular_velocity,
                 vec3_scale(b->state.angular_acceleration, dt));

    b->state.position =
        vec3_add(b->state.position,
                 vec3_scale(b->state.linear_velocity, dt));
    printk("hi\n");
    quat omega = quat_from_angular_velocity(b->state.angular_velocity);
    quat qdot = quat_scale(quat_mul(omega, b->state.orientation), 0.5f);
    printk("hi2\n");

quat temp = b->state.orientation;
quat delta = quat_scale(qdot, dt);
quat sum = quat_add(temp, delta);

printk("temp=(%f,%f,%f,%f)\n", temp.w, temp.x, temp.y, temp.z);
printk("qdot=(%f,%f,%f,%f)\n", qdot.w, qdot.x, qdot.y, qdot.z);
printk("dt=%f\n", dt);
printk("sum=(%f,%f,%f,%f)\n", sum.w, sum.x, sum.y, sum.z);

float nsq = quat_norm_sq(sum);
printk("nsq=%f\n", nsq);

float n = sqrtf(nsq);
printk("n=%f\n", n);

quat res = quat_normalize(sum);
printk("res=(%f,%f,%f,%f)\n", res.w, res.x, res.y, res.z);

b->state.orientation = res;
printk("hi3\n");
        
    return 0;
}

// int phys_resolve_collision_basic(rigid_body *a, rigid_body *b,
//                                  const collision_result *res) {
//     if (!a || !b || !res || !res->hit) return -1;

//     vec3 normal;
//     float depth;

//     if (res->epa.hit && res->epa.depth > 1e-6f &&
//         vec3_norm_sq(res->epa.normal) > 1e-8f) {
//         normal = vec3_normalize(res->epa.normal);
//         depth = res->epa.depth;
//     } else {
//         normal = vec3_sub(b->state.position, a->state.position);
//         if (vec3_norm_sq(normal) < 1e-8f)
//             normal = vec3_make(1.0f, 0.0f, 0.0f);
//         else
//             normal = vec3_normalize(normal);
//         depth = 0.01f;
//     }

//     {
//         float inv_mass_a = a->geom.inv_mass;
//         float inv_mass_b = b->geom.inv_mass;
//         float inv_mass_sum = inv_mass_a + inv_mass_b;

//         if (inv_mass_sum > 0.0f) {
//             float percent = 0.8f;
//             float slop = 0.001f;
//             float corr_mag = percent * fmaxf(depth - slop, 0.0f) / inv_mass_sum;
//             vec3 correction = vec3_scale(normal, corr_mag);

//             a->state.position = vec3_sub(a->state.position,
//                                          vec3_scale(correction, inv_mass_a));
//             b->state.position = vec3_add(b->state.position,
//                                          vec3_scale(correction, inv_mass_b));
            

//         }
//     }

//     {
//         vec3 rv = vec3_sub(b->state.linear_velocity, a->state.linear_velocity);
//         float vel_along_normal = vec3_dot(rv, normal);

//         if (vel_along_normal < 0.0f) {
//             float e = 0.4f;
//             float inv_mass_sum = a->geom.inv_mass + b->geom.inv_mass;
//             if (inv_mass_sum > 0.0f) {
//                 float j = -(1.0f + e) * vel_along_normal / inv_mass_sum;
//                 vec3 impulse = vec3_scale(normal, j);

//                 a->state.linear_velocity =
//                     vec3_sub(a->state.linear_velocity,
//                              vec3_scale(impulse, a->geom.inv_mass));
//                 b->state.linear_velocity =
//                     vec3_add(b->state.linear_velocity,
//                              vec3_scale(impulse, b->geom.inv_mass));
//             }
//         }
//     }

//     return 0;
// }
int phys_resolve_collision_basic(rigid_body *a, rigid_body *b,
                                 const collision_result *res) {
    if (!a || !b || !res || !res->hit) return -1;
    printk("hello\n");

    vec3 normal;
    float depth;
    vec3 contact_a;
    vec3 contact_b;

    if (res->epa.hit &&
        res->epa.depth > 1e-6f &&
        vec3_norm_sq(res->epa.normal) > 1e-8f) {
        normal = vec3_normalize(res->epa.normal);
        depth = res->epa.depth;
        contact_a = res->epa.contact_a;
        contact_b = res->epa.contact_b;
    } else {
        normal = vec3_sub(b->state.position, a->state.position);
        if (vec3_norm_sq(normal) < 1e-8f)
            normal = vec3_make(1.0f, 0.0f, 0.0f);
        else
            normal = vec3_normalize(normal);

        depth = 0.01f;

        contact_a = a->state.position;
        contact_b = b->state.position;
    }

    /* positional correction */
    {
        float inv_mass_a = a->geom.inv_mass;
        float inv_mass_b = b->geom.inv_mass;
        float inv_mass_sum = inv_mass_a + inv_mass_b;

        if (inv_mass_sum > 0.0f) {
            float percent = 0.8f;
            float slop = 0.001f;
            float corr_mag = percent * fmaxf(depth - slop, 0.0f) / inv_mass_sum;
            vec3 correction = vec3_scale(normal, corr_mag);

            a->state.position = vec3_sub(a->state.position,
                                         vec3_scale(correction, inv_mass_a));
            b->state.position = vec3_add(b->state.position,
                                         vec3_scale(correction, inv_mass_b));
        }
    }

    /* impulse response with angular effects */
    {
        vec3 ra = vec3_sub(contact_a, a->state.position);
        vec3 rb = vec3_sub(contact_b, b->state.position);

        vec3 va = vec3_add(a->state.linear_velocity,
                           vec3_cross(a->state.angular_velocity, ra));
        vec3 vb = vec3_add(b->state.linear_velocity,
                           vec3_cross(b->state.angular_velocity, rb));

        vec3 rv = vec3_sub(vb, va);
        float vel_along_normal = vec3_dot(rv, normal);

        if (vel_along_normal < 0.0f) {
            float e = 0.4f;

            vec3 ra_x_n = vec3_cross(ra, normal);
            vec3 rb_x_n = vec3_cross(rb, normal);

            vec3 invI_ra_x_n = phys_apply_inv_inertia_world(a, ra_x_n);
            vec3 invI_rb_x_n = phys_apply_inv_inertia_world(b, rb_x_n);

            float ang_term_a = vec3_dot(vec3_cross(invI_ra_x_n, ra), normal);
            float ang_term_b = vec3_dot(vec3_cross(invI_rb_x_n, rb), normal);

            float denom = a->geom.inv_mass + b->geom.inv_mass + ang_term_a + ang_term_b;

            if (denom > 1e-8f) {
                float j = -(1.0f + e) * vel_along_normal / denom;
                vec3 impulse = vec3_scale(normal, j);

                a->state.linear_velocity =
                    vec3_sub(a->state.linear_velocity,
                             vec3_scale(impulse, a->geom.inv_mass));
                b->state.linear_velocity =
                    vec3_add(b->state.linear_velocity,
                             vec3_scale(impulse, b->geom.inv_mass));

                a->state.angular_velocity =
                    vec3_sub(a->state.angular_velocity,
                             phys_apply_inv_inertia_world(a, vec3_cross(ra, impulse)));
                b->state.angular_velocity =
                    vec3_add(b->state.angular_velocity,
                             phys_apply_inv_inertia_world(b, vec3_cross(rb, impulse)));
            }
        }
    }

    return 0;
}