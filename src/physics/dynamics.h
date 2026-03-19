#pragma once

#include "vec3.h"
#include "quat.h"
#include "collision.h"

#include "rpi.h"
#include "rpi-math.h"
#include "string.h"

#ifndef isfinite
#define isfinite(x) ((x) - (x) == 0.0f)
#endif

#define DYN_MAX_AV  50.0f
#define DYN_MAX_DT  0.1f

/* zero vector if any component is non-finite */
static inline vec3 dyn_fv3(vec3 v) {
    if (!isfinite(v.x) || !isfinite(v.y) || !isfinite(v.z))
        return vec3_zero();
    return v;
}

/* normalize quaternion; return identity on degenerate input.
 * noinline + volatile reads: same workaround as quat_dot — the ARM VFP
 * compiler can emit VLDMD (8-byte-aligned double loads) for a quat struct,
 * which data-aborts when the stack slot is only 4-byte aligned.
 * Forcing a real call boundary (noinline) makes the ABI pass the four
 * floats in s0-s3, and volatile reads inside force individual VLDR. */
static __attribute__((noinline)) quat dyn_fqn(quat q) {
    volatile float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    float n2 = qw*qw + qx*qx + qy*qy + qz*qz;
    if (!isfinite(n2) || n2 < 1e-12f)
        return (quat){1.0f, 0.0f, 0.0f, 0.0f};
    float inv = 1.0f / sqrtf(n2);
    quat r = {qw*inv, qx*inv, qy*inv, qz*inv};
    if (!isfinite(r.w) || !isfinite(r.x) || !isfinite(r.y) || !isfinite(r.z))
        return (quat){1.0f, 0.0f, 0.0f, 0.0f};
    return r;
}

/* clamp angular velocity to DYN_MAX_AV; zero non-finite */
static inline vec3 dyn_clav(vec3 av) {
    if (!isfinite(av.x) || !isfinite(av.y) || !isfinite(av.z))
        return vec3_zero();
    float sq = av.x*av.x + av.y*av.y + av.z*av.z;
    if (sq > DYN_MAX_AV * DYN_MAX_AV) {
        float s = DYN_MAX_AV / sqrtf(sq);
        av.x *= s; av.y *= s; av.z *= s;
    }
    return av;
}

/* apply inverse inertia tensor in world space: R * I_b^{-1} * R^T * v */
static inline vec3 dyn_iIw(const rigid_body *b, vec3 v) {
    quat q  = dyn_fqn(b->state.orientation);
    quat qc = quat_conjugate(q);
    vec3 vb = quat_rotate_vec3(qc, v);
    vb = vec3_hadamard(vb, b->geom.inv_inertia_body);
    return dyn_fv3(quat_rotate_vec3(q, vb));
}

/* ------------------------------------------------------------------ */
/* public API                                                           */
/* ------------------------------------------------------------------ */

int phys_body_clear_forces(rigid_body *b);
int phys_body_add_force(rigid_body *b, vec3 f);
int phys_body_add_torque(rigid_body *b, vec3 t);
int phys_body_integrate(rigid_body *b, float dt);
int phys_resolve_collision_basic(rigid_body *a, rigid_body *b,
                                 const collision_result *res);
