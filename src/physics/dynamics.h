
#include "vec3.h"
#include "quat.h"
#include "collision.h"

static unsigned canary1 = 0x11111111;
static unsigned canary2 = 0x22222222;

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

// static inline int float_is_nan(float x) {
//     unsigned u = *(unsigned *)&x;
//     return ((u & 0x7f800000) == 0x7f800000) &&  // exponent all 1s
//            ((u & 0x007fffff) != 0);             // mantissa non-zero
// }

int phys_body_integrate(rigid_body *b, float dt) {
    volatile unsigned canary_a = 0x11111111;
    volatile unsigned canary_b = 0x22222222;

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

    // printk("I0\n");
    quat omega = quat_from_angular_velocity(b->state.angular_velocity);
    // printk("I1\n");
    quat qdot  = quat_scale(quat_mul(omega, b->state.orientation), 0.5f);
    // printk("I2\n");

    volatile unsigned canary_c = 0x33333333;
    volatile unsigned canary_d = 0x44444444;

    quat temp  = b->state.orientation;
    // printk("I3\n");
    quat delta = quat_scale(qdot, dt);
    // printk("I4\n");
    quat sum   = quat_add(temp, delta);
    // printk("I5\n");

    if (canary_a != 0x11111111) panic("stack smash a");
    if (canary_b != 0x22222222) panic("stack smash b");
    if (canary_c != 0x33333333) panic("stack smash c");
    if (canary_d != 0x44444444) panic("stack smash d");

    // printk_float("sum.w", sum.w);
    // printk_float("sum.x", sum.x);
    // printk_float("sum.y", sum.y);
    // printk_float("sum.z", sum.z);

    // printk("N0\n");
    volatile float w = sum.w;
    // printk("N1\n");
    volatile float x = sum.x;
    // printk("N2\n");
    volatile float y = sum.y;
    // printk("N3\n");
    volatile float z = sum.z;
    // printk("N4\n");

    volatile float ww = w * w;
    // printk("N5\n");
    volatile float xx = x * x;
    volatile float yed = sum.y;
    if (yed < 1e-20f && yed> -1e-20f) yed = 0.0f;
    // printk("N6\n");
    volatile float yy = yed * yed;
    // printk("N7\n");
    volatile float zz = z * z;
    // printk("N8\n");

    volatile float s1 = ww + xx;
    // printk("N9\n");
    volatile float s2 = yy + zz;
    // printk("N10\n");
    volatile float nsq = s1 + s2;
    // printk("N11\n");
    // printk_float("nsq", nsq);

    if (canary_a != 0x11111111) panic("stack smash a2");
    if (canary_b != 0x22222222) panic("stack smash b2");
    if (canary_c != 0x33333333) panic("stack smash c2");
    if (canary_d != 0x44444444) panic("stack smash d2");

    if (nsq <= 1e-20f) {
        printk("N12 fallback\n");
        b->state.orientation = (quat){1.0f, 0.0f, 0.0f, 0.0f};
        return 0;
    }

    // printk("N13\n");
    volatile float root = sqrtf(nsq);
    // printk("N14\n");
    // printk_float("root", root);

    volatile float invn = 1.0f / root;
    // printk("N15\n");
    // printk_float("invn", invn);

    quat res;
    res.w = w * invn;
    // printk("N16\n");
    res.x = x * invn;
    // printk("N17\n");
    res.y = y * invn;
    // printk("N18\n");
    res.z = z * invn;
    // printk("N19\n");

    printk_float("res.w", res.w);
    printk_float("res.x", res.x);
    printk_float("res.y", res.y);
    printk_float("res.z", res.z);

    b->state.orientation = res;
    printk("N20\n");
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