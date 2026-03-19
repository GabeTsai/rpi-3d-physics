#include "dynamics.h"
#include "vec3.h"
#include "quat.h"
#include "collision.h"

#include "rpi.h"
#include "rpi-math.h"
#include "string.h"

int phys_body_clear_forces(rigid_body *b) {
    if (!b) return -1;
    b->force  = vec3_zero();
    b->torque = vec3_zero();
    return 0;
}

int phys_body_add_force(rigid_body *b, vec3 f) {
    if (!b) return -1;
    b->force = dyn_fv3(vec3_add(b->force, f));
    return 0;
}

int phys_body_add_torque(rigid_body *b, vec3 t) {
    if (!b) return -1;
    b->torque = dyn_fv3(vec3_add(b->torque, t));
    return 0;
}

int phys_body_integrate(rigid_body *b, float dt) {
    if (!b || !isfinite(dt) || dt <= 0.0f) return -1;
    if (dt > DYN_MAX_DT) dt = DYN_MAX_DT;

    // clean up any non-finite values
    b->state.position         = dyn_fv3(b->state.position);
    b->state.linear_velocity  = dyn_fv3(b->state.linear_velocity);
    b->state.angular_velocity = dyn_clav(dyn_fv3(b->state.angular_velocity));
    b->state.orientation      = dyn_fqn(b->state.orientation);
    b->force                  = dyn_fv3(b->force);
    b->torque                 = dyn_fv3(b->torque);

    // linear — semi-implicit Euler
    vec3 lin_acc = dyn_fv3(vec3_scale(b->force, b->geom.inv_mass));
    b->state.linear_acceleration = lin_acc;
    b->state.linear_velocity = dyn_fv3(
        vec3_add(b->state.linear_velocity, vec3_scale(lin_acc, dt)));
    b->state.position = dyn_fv3(
        vec3_add(b->state.position, vec3_scale(b->state.linear_velocity, dt)));

    // angular — semi-implicit Euler for velocity
    vec3 ang_acc = dyn_fv3(dyn_iIw(b, b->torque));
    b->state.angular_acceleration = ang_acc;
    b->state.angular_velocity = dyn_clav(dyn_fv3(
        vec3_add(b->state.angular_velocity, vec3_scale(ang_acc, dt))));

    // orientation — exact axis-angle: q_new = exp(w*dt/2) * q
    {
        vec3  w  = b->state.angular_velocity;
        float w2 = w.x*w.x + w.y*w.y + w.z*w.z;
        if (w2 > 1e-12f) {
            float wmag = sqrtf(w2);
            float half = 0.5f * wmag * dt;
            float sinc = sinf(half) / wmag;
            quat  dq   = {cosf(half), w.x*sinc, w.y*sinc, w.z*sinc};
            b->state.orientation = dyn_fqn(quat_mul(dq, b->state.orientation));
        }
    }

    return 0;
}

int phys_resolve_collision_basic(rigid_body *a, rigid_body *b,
                                 const collision_result *res) {
    if (!a || !b || !res || !res->hit) return -1;

    /* --- collision normal, depth, contact points --- */
    vec3  n  = {.x=0.0f, .y=1.0f, .z=0.0f};
    float d  = 0.0f;
    vec3  ca = a->state.position;
    vec3  cb = b->state.position;

    if (res->epa.hit && isfinite(res->epa.depth) && res->epa.depth > 1e-6f) {
        float nn = res->epa.normal.x*res->epa.normal.x
                 + res->epa.normal.y*res->epa.normal.y
                 + res->epa.normal.z*res->epa.normal.z;
        if (isfinite(nn) && nn > 1e-8f) {
            float inv = 1.0f / sqrtf(nn);
            n.x = res->epa.normal.x * inv;
            n.y = res->epa.normal.y * inv;
            n.z = res->epa.normal.z * inv;
            d   = res->epa.depth;
            if (isfinite(res->epa.contact_a.x) &&
                isfinite(res->epa.contact_a.y) &&
                isfinite(res->epa.contact_a.z))
                ca = res->epa.contact_a;
            if (isfinite(res->epa.contact_b.x) &&
                isfinite(res->epa.contact_b.y) &&
                isfinite(res->epa.contact_b.z))
                cb = res->epa.contact_b;
        }
    } else {
        vec3  sep = vec3_sub(b->state.position, a->state.position);
        float sn  = sep.x*sep.x + sep.y*sep.y + sep.z*sep.z;
        if (isfinite(sn) && sn > 1e-8f) {
            float inv = 1.0f / sqrtf(sn);
            n.x = sep.x*inv; n.y = sep.y*inv; n.z = sep.z*inv;
        }
        d = 0.01f;
    }

    // positional correction
    float total_im = a->geom.inv_mass + b->geom.inv_mass;
    if (isfinite(total_im) && total_im > 0.0f) {
        float corr = 0.8f * fmaxf(d - 0.001f, 0.0f) / total_im;
        if (isfinite(corr) && corr > 0.0f) {
            a->state.position = dyn_fv3(vec3_sub(a->state.position,
                vec3_scale(n, corr * a->geom.inv_mass)));
            b->state.position = dyn_fv3(vec3_add(b->state.position,
                vec3_scale(n, corr * b->geom.inv_mass)));
        }
    }

    // impulse-based response
    vec3 ra = dyn_fv3(vec3_sub(ca, a->state.position));
    vec3 rb = dyn_fv3(vec3_sub(cb, b->state.position));

    vec3 va = dyn_fv3(vec3_add(a->state.linear_velocity,
                               vec3_cross(a->state.angular_velocity, ra)));
    vec3 vb = dyn_fv3(vec3_add(b->state.linear_velocity,
                               vec3_cross(b->state.angular_velocity, rb)));

    float vrel = vec3_dot(vec3_sub(vb, va), n);
    if (!isfinite(vrel) || vrel >= 0.0f) return 0;

    vec3 ra_x_n = vec3_cross(ra, n);
    vec3 rb_x_n = vec3_cross(rb, n);

    vec3 iIa_ran = dyn_iIw(a, ra_x_n);
    vec3 iIb_rbn = dyn_iIw(b, rb_x_n);

    float ang_a = vec3_dot(vec3_cross(iIa_ran, ra), n);
    float ang_b = vec3_dot(vec3_cross(iIb_rbn, rb), n);
    if (!isfinite(ang_a)) ang_a = 0.0f;
    if (!isfinite(ang_b)) ang_b = 0.0f;

    float denom = a->geom.inv_mass + b->geom.inv_mass + ang_a + ang_b;
    if (!isfinite(denom) || denom < 1e-8f) return 0;

    float j = -(1.0f + 0.6f) * vrel / denom;
    if (!isfinite(j)) return 0;

    vec3 imp = vec3_scale(n, j);

    a->state.linear_velocity = dyn_fv3(vec3_sub(a->state.linear_velocity,
        vec3_scale(imp, a->geom.inv_mass)));
    b->state.linear_velocity = dyn_fv3(vec3_add(b->state.linear_velocity,
        vec3_scale(imp, b->geom.inv_mass)));

    vec3 ai = dyn_fv3(dyn_iIw(a, vec3_cross(ra, imp)));
    vec3 bi = dyn_fv3(dyn_iIw(b, vec3_cross(rb, imp)));

    a->state.angular_velocity = dyn_clav(dyn_fv3(
        vec3_sub(a->state.angular_velocity, ai)));
    b->state.angular_velocity = dyn_clav(dyn_fv3(
        vec3_add(b->state.angular_velocity, bi)));

    return 0;
}
