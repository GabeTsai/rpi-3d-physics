#include "collision.h"
#include "vec3.h"
#include "quat.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "rpi.h"

#define GJK_MAX_ITERS 32
#define EPA_MAX_ITERS 64
#define EPA_MAX_FACES 128
#define EPA_MAX_LOOSE_EDGES 64

#define COLL_EPS 1e-6f
#define GJK_DUP_EPS 1e-6f
#define EPA_TOL 1e-4f

typedef struct {
    int a, b;
} edge2i;

typedef struct {
    int a, b, c;
    vec3 normal;
    float dist;
    int alive;
} epa_face;

/* -------------------- small vec helpers -------------------- */


/* -------------------- transforms -------------------- */

static vec3 rotate_vec(const quat *q, vec3 v) {
    return quat_rotate_vec3(*q, v);
}

static vec3 inv_rotate_vec(const quat *q, vec3 v) {
    quat qc = quat_conjugate(*q);
    return quat_rotate_vec3(qc, v);
}

// static vec3 local_to_world_point(const rigid_body *b, vec3 p_local) {
//     return vec3_add(rotate_vec(&b->state.orientation, p_local), b->state.position);
// }

static vec3 local_to_world_point(const rigid_body *b, vec3 p_local) {
    // printk("LW0\n");
    // printk("LW0a b=%x\n", (unsigned)b);

    // printk_float("lw.pos.x", b->state.position.x);
    // printk_float("lw.pos.y", b->state.position.y);
    // printk_float("lw.pos.z", b->state.position.z);

    // printk_float("lw.ori.w", b->state.orientation.w);
    // printk_float("lw.ori.x", b->state.orientation.x);
    // printk_float("lw.ori.y", b->state.orientation.y);
    // printk_float("lw.ori.z", b->state.orientation.z);

    // printk_float("lw.local.x", p_local.x);
    // printk_float("lw.local.y", p_local.y);
    // printk_float("lw.local.z", p_local.z);

    // printk("LW1\n");
    vec3 r = rotate_vec(&b->state.orientation, p_local);
    // printk("LW2\n");
    // printk_float("lw.rot.x", r.x);
    // printk_float("lw.rot.y", r.y);
    // printk_float("lw.rot.z", r.z);

    vec3 out = vec3_add(r, b->state.position);
    // printk("LW3\n");
    // printk_float("lw.out.x", out.x);
    // printk_float("lw.out.y", out.y);
    // printk_float("lw.out.z", out.z);

    return out;
}

/* -------------------- support mapping -------------------- */

static vec3 support_local_mesh(const mesh_geom *m, vec3 dir_local) {
    vec3 best = {.x=0,.y=0,.z=0};
    float best_dot = -FLT_MAX;
    int first = 1;

    for (int i = 0; i < m->triangle_count; ++i) {
        const triangle *t = &m->triangles[i];
        const vec3 verts[3] = { t->v0, t->v1, t->v2 };
        for (int k = 0; k < 3; ++k) {
            float d = vec3_dot(verts[k], dir_local);
            if (first || d > best_dot) {
                best_dot = d;
                best = verts[k];
                first = 0;
            }
        }
    }
    return best;
}

static vec3 support_local_shape(const rigid_body_geom *g, vec3 dir_local) {
    return support_local_mesh(&g->mesh, dir_local);
}

static vec3 support_world_shape(const rigid_body *b, vec3 dir_world) {
    vec3 dir_local = inv_rotate_vec(&b->state.orientation, dir_world);
    vec3 p_local = support_local_shape(&b->geom, dir_local);
    return local_to_world_point(b, p_local);
}

static support_point support_minkowski(const rigid_body *a,
                                       const rigid_body *b,
                                       vec3 dir_world) {
    support_point sp;
    sp.supp_a = support_world_shape(a, dir_world);
    sp.supp_b = support_world_shape(b, vec3_neg(dir_world));
    sp.p = vec3_sub(sp.supp_a, sp.supp_b);
    return sp;
}

/* -------------------- AABB -------------------- */
static void expand_aabb_point(aabb3 *a, vec3 p) {
    // printk("EA0\n");
    // printk_float("p.x", p.x);
    // printk_float("p.y", p.y);
    // printk_float("p.z", p.z);

    // printk("EA1\n");
    a->min = vec3_min(a->min, p);
    // printk("EA2\n");
    a->max = vec3_max(a->max, p);
    // printk("EA3\n");
}



aabb3 phys_body_compute_aabb(const rigid_body *b) {
    aabb3 out;
    // printk("AB0 enter\n");
    // printk("AB0a b=%x\n", (unsigned)b);

    out.min.x = out.min.y = out.min.z = FLT_MAX;
    out.max.x = out.max.y = out.max.z = -FLT_MAX;
    // printk("AB1 init\n");

    if (!b) {
        // printk("AB2 null body\n");
        vec3 z = {.x = 0, .y = 0, .z = 0};
        out.min = z;
        out.max = z;
        return out;
    }

    // printk_float("ab.pos.x", b->state.position.x);
    // printk_float("ab.pos.y", b->state.position.y);
    // printk_float("ab.pos.z", b->state.position.z);

    // printk_float("ab.ori.w", b->state.orientation.w);
    // printk_float("ab.ori.x", b->state.orientation.x);
    // printk_float("ab.ori.y", b->state.orientation.y);
    // printk_float("ab.ori.z", b->state.orientation.z);

    const mesh_geom *m = &b->geom.mesh;
    // printk("AB3 mesh ptr=%x tris=%x tri_count=%d\n",
    //        (unsigned)m, (unsigned)m->triangles, m->triangle_count);

    if (!m->triangles || m->triangle_count <= 0) {
        // printk("AB4 degenerate mesh\n");
        out.min = b->state.position;
        out.max = b->state.position;
        return out;
    }

    for (int i = 0; i < m->triangle_count; ++i) {
        // printk("AB5 tri i=%d\n", i);
        // printk_float("t.v0.x", m->triangles[i].v0.x);
        // printk_float("t.v0.y", m->triangles[i].v0.y);
        // printk_float("t.v0.z", m->triangles[i].v0.z);

        // printk_float("t.v1.x", m->triangles[i].v1.x);
        // printk_float("t.v1.y", m->triangles[i].v1.y);
        // printk_float("t.v1.z", m->triangles[i].v1.z);

        // printk_float("t.v2.x", m->triangles[i].v2.x);
        // printk_float("t.v2.y", m->triangles[i].v2.y);
        // printk_float("t.v2.z", m->triangles[i].v2.z);

        // printk("AB6 v0\n");
        expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v0));
        // printk("AB7 v1\n");
        expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v1));
        // printk("AB8 v2\n");
        expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v2));
        // printk("AB9 tri done\n");
    }

    // printk("AB10 done\n");
    // printk_float("out.min.x", out.min.x);
    // printk_float("out.min.y", out.min.y);
    // printk_float("out.min.z", out.min.z);
    // printk_float("out.max.x", out.max.x);
    // printk_float("out.max.y", out.max.y);
    // printk_float("out.max.z", out.max.z);

    return out;
}

int phys_aabb_overlap(aabb3 a, aabb3 b) {
    if (a.max.x < b.min.x || b.max.x < a.min.x) return 0;
    if (a.max.y < b.min.y || b.max.y < a.min.y) return 0;
    if (a.max.z < b.min.z || b.max.z < a.min.z) return 0;
    return 1;
}

/* -------------------- GJK simplex handling -------------------- */

static int simplex_has_duplicate(const simplex3 *s, support_point p) {
    for (int i = 0; i < s->count; ++i) {
        if (vec3_norm_sq(vec3_sub(s->pts[i].p, p.p)) < GJK_DUP_EPS)
            return 1;
    }
    return 0;
}

static int do_simplex_line(simplex3 *s, vec3 *dir) {
    // printk("line\n");
    /* A = newest = pts[1], B = pts[0] */
    vec3 A = s->pts[1].p;
    vec3 B = s->pts[0].p;
    vec3 AO = vec3_neg(A);
    vec3 AB = vec3_sub(B, A);

    if (vec3_dot(AB, AO) > 0.0f) {
        vec3 new_dir = vec3_cross(vec3_cross(AB, AO), AB);

        /* Origin lies on the line AB: the simplex already contains it. */
        if (vec3_norm_sq(new_dir) <= COLL_EPS * COLL_EPS) {
            return 1;
        }

        *dir = new_dir;
    } else {
        s->pts[0] = s->pts[1];
        s->count = 1;
        *dir = AO;
    }

    return 0;
}

static int do_simplex_triangle(simplex3 *s, vec3 *dir) {
    /* A newest = pts[2], B = pts[1], C = pts[0] */
    support_point Ap = s->pts[2];
    support_point Bp = s->pts[1];
    support_point Cp = s->pts[0];

    vec3 A = Ap.p;
    vec3 B = Bp.p;
    vec3 C = Cp.p;

    vec3 AO = vec3_neg(A);
    vec3 AB = vec3_sub(B, A);
    vec3 AC = vec3_sub(C, A);

    /* Check vertex region A first */
    float d1 = vec3_dot(AB, AO);
    float d2 = vec3_dot(AC, AO);
    if (d1 <= 0.0f && d2 <= 0.0f) {
        s->pts[0] = Ap;
        s->count = 1;
        *dir = AO;
        return 0;
    }

    /* Check edge region AB */
    vec3 BO = vec3_neg(B);
    vec3 BA = vec3_sub(A, B);
    vec3 BC = vec3_sub(C, B);
    float d3 = vec3_dot(BA, BO);
    float d4 = vec3_dot(BC, BO);

    if (d3 > 0.0f && d4 <= 0.0f) {
        s->pts[0] = Bp;
        s->pts[1] = Ap;
        s->count = 2;

        vec3 nd = vec3_cross(vec3_cross(AB, AO), AB);
        if (vec3_norm_sq(nd) <= COLL_EPS * COLL_EPS)
            return 1;
        *dir = nd;
        return 0;
    }

    /* Check edge region AC */
    vec3 CO = vec3_neg(C);
    vec3 CA = vec3_sub(A, C);
    vec3 CB = vec3_sub(B, C);
    float d5 = vec3_dot(CA, CO);
    float d6 = vec3_dot(CB, CO);

    if (d5 > 0.0f && d6 <= 0.0f) {
        s->pts[0] = Cp;
        s->pts[1] = Ap;
        s->count = 2;

        vec3 nd = vec3_cross(vec3_cross(AC, AO), AC);
        if (vec3_norm_sq(nd) <= COLL_EPS * COLL_EPS)
            return 1;
        *dir = nd;
        return 0;
    }

    /* Otherwise origin is in face region of triangle ABC */
    vec3 ABC = vec3_cross(AB, AC);
    if (vec3_norm_sq(ABC) <= COLL_EPS * COLL_EPS)
        return 1;

    if (vec3_dot(ABC, AO) > 0.0f) {
        s->pts[0] = Cp;
        s->pts[1] = Bp;
        s->pts[2] = Ap;
        s->count = 3;
        *dir = ABC;
    } else {
        s->pts[0] = Bp;
        s->pts[1] = Cp;
        s->pts[2] = Ap;
        s->count = 3;
        *dir = vec3_neg(ABC);
    }

    return 0;
}

static int do_simplex_tetra(simplex3 *s, vec3 *dir) {
    /* A newest = pts[3], B = pts[2], C = pts[1], D = pts[0] */
    // printk("tetra\n");
    support_point Ap = s->pts[3];
    support_point Bp = s->pts[2];
    support_point Cp = s->pts[1];
    support_point Dp = s->pts[0];

    vec3 A = Ap.p;
    vec3 B = Bp.p;
    vec3 C = Cp.p;
    vec3 D = Dp.p;

    vec3 AO = vec3_neg(A);

    vec3 AB = vec3_sub(B, A);
    vec3 AC = vec3_sub(C, A);
    vec3 AD = vec3_sub(D, A);

    vec3 ABC = vec3_cross(AB, AC);
    vec3 ACD = vec3_cross(AC, AD);
    vec3 ADB = vec3_cross(AD, AB);

    /* orient outward */
    if (vec3_dot(ABC, AD) > 0.0f) ABC = vec3_neg(ABC);
    if (vec3_dot(ACD, AB) > 0.0f) ACD = vec3_neg(ACD);
    if (vec3_dot(ADB, AC) > 0.0f) ADB = vec3_neg(ADB);

    if (vec3_dot(ABC, AO) > 0.0f) {
        s->pts[0] = Cp;   // C
        s->pts[1] = Bp;   // B
        s->pts[2] = Ap;   // A
        s->count = 3;
        *dir = ABC;
        return 0;
    }

    if (vec3_dot(ACD, AO) > 0.0f) {
        s->pts[0] = Dp;   // D
        s->pts[1] = Cp;   // C
        s->pts[2] = Ap;   // A
        s->count = 3;
        *dir = ACD;
        return 0;
    }

    if (vec3_dot(ADB, AO) > 0.0f) {
        s->pts[0] = Bp;   // B
        s->pts[1] = Dp;   // D
        s->pts[2] = Ap;   // A
        s->count = 3;
        *dir = ADB;
        return 0;
    }

    return 1;
}

static int do_simplex(simplex3 *s, vec3 *dir) {
    switch (s->count) {
    case 1:
        *dir = vec3_neg(s->pts[0].p);
        return 0;
    case 2:
        return do_simplex_line(s, dir);
    case 3:
        return do_simplex_triangle(s, dir);
    case 4:
        return do_simplex_tetra(s, dir);
    default:
        return 0;
    }
}

/* -------------------- GJK -------------------- */

gjk_result phys_gjk_intersect(const rigid_body *a, const rigid_body *b) {
    gjk_result out;
    memset(&out, 0, sizeof(out));

    vec3 dir = vec3_sub(b->state.position, a->state.position);
    if (vec3_norm_sq(dir) < COLL_EPS) dir = (vec3){.x=1,.y=0,.z=0};

    // printk("<%f, %f, %f>", dir.x, dir.y, dir.z);

    simplex3 s;
    memset(&s, 0, sizeof(s));

    support_point first = support_minkowski(a, b, dir);
    s.pts[0] = first;
    s.count = 1;
    dir = vec3_neg(first.p);

    for (int iter = 0; iter < GJK_MAX_ITERS; ++iter) {
        // printk("%d\n", iter);
        if (vec3_norm_sq(dir) < COLL_EPS) {
            out.hit = 1;
            out.simplex = s;
            return out;
        }

        support_point p = support_minkowski(a, b, dir);

        if (simplex_has_duplicate(&s, p)) {
            // printk("uh o!h");
            out.hit = 0;
            out.simplex = s;
            out.search_dir = dir;
            return out;
        }

        if (vec3_dot(p.p, dir) < COLL_EPS) {
            out.hit = 0;
            out.simplex = s;
            out.search_dir = dir;
            return out;
        }

        if (s.count >= 4) {
            out.hit = 0;
            out.simplex = s;
            out.search_dir = dir;
            return out;
        }
        s.pts[s.count++] = p;
        if (do_simplex(&s, &dir)) {
            out.hit = 1;
            out.simplex = s;
            return out;
        }
    }

    out.hit = 0;
    out.simplex = s;
    out.search_dir = dir;
    return out;
}

/* -------------------- EPA -------------------- */

static void epa_make_face(epa_face *f, int a, int b, int c,
                          const support_point *verts) {
    f->a = a;
    f->b = b;
    f->c = c;
    f->alive = 1;

    vec3 A = verts[a].p;
    vec3 B = verts[b].p;
    vec3 C = verts[c].p;

    vec3 n = vec3_cross(vec3_sub(B, A), vec3_sub(C, A));
    float n2 = vec3_norm_sq(n);
    if (n2 <= COLL_EPS * COLL_EPS) {
        f->alive = 0;
        f->normal = (vec3){.x=0,.y=0,.z=0};
        f->dist = 0.0f;
        return;
    }
    n = vec3_scale(n, 1.0f / sqrtf(n2));

    if (vec3_dot(n, A) < 0.0f) {
        int t = f->b;
        f->b = f->c;
        f->c = t;

        B = verts[f->b].p;
        C = verts[f->c].p;
        n = vec3_cross(vec3_sub(B, A), vec3_sub(C, A));
        n2 = vec3_norm_sq(n);
        if (n2 <= COLL_EPS * COLL_EPS) {
            f->alive = 0;
            f->normal = (vec3){.x=0,.y=0,.z=0};
            f->dist = 0.0f;
            return;
        }
        n = vec3_scale(n, 1.0f / sqrtf(n2));
    }

    f->normal = n;
    f->dist = vec3_dot(n, A);
}

static int epa_find_closest_face(epa_face *faces, int face_count) {
    int best = -1;
    float best_dist = FLT_MAX;

    for (int i = 0; i < face_count; ++i) {
        if (!faces[i].alive) continue;
        if (faces[i].dist < best_dist) {
            best_dist = faces[i].dist;
            best = i;
        }
    }
    return best;
}

static void epa_add_edge(edge2i *edges, int *edge_count, int a, int b) {
    for (int i = 0; i < *edge_count; ++i) {
        if (edges[i].a == b && edges[i].b == a) {
            edges[i] = edges[--(*edge_count)];
            return;
        }
    }
    if (*edge_count < EPA_MAX_LOOSE_EDGES) {
        edges[*edge_count].a = a;
        edges[*edge_count].b = b;
        (*edge_count)++;
    }
}

static vec3 pick_any_perp(vec3 v) {
    vec3 axis = (fabsf(v.x) < 0.577f) ? (vec3){.x=1,.y=0,.z=0}
              : (fabsf(v.y) < 0.577f) ? (vec3){.x=0,.y=1,.z=0}
                                      : (vec3){.x=0,.y=0,.z=1};
    return vec3_cross(v, axis);
}

static int same_support_point(support_point a, support_point b) {
    return vec3_norm_sq(vec3_sub(a.p, b.p)) <= GJK_DUP_EPS;
}

static int seed_epa_simplex(const rigid_body *a, const rigid_body *b,
                            const simplex3 *gjk_simplex,
                            support_point *verts, int *vert_count) {
    if (!gjk_simplex || gjk_simplex->count < 2)
        return 0;

    *vert_count = gjk_simplex->count;
    for (int i = 0; i < gjk_simplex->count; ++i)
        verts[i] = gjk_simplex->pts[i];

    if (*vert_count == 4)
        return 1;

    if (*vert_count == 3) {
        vec3 A = verts[0].p;
        vec3 B = verts[1].p;
        vec3 C = verts[2].p;

        vec3 n = vec3_cross(vec3_sub(B, A), vec3_sub(C, A));
        if (vec3_norm_sq(n) <= COLL_EPS * COLL_EPS)
            return 0;

        n = vec3_normalize(n);

        support_point p1 = support_minkowski(a, b, n);
        support_point p2 = support_minkowski(a, b, vec3_neg(n));

        float d1 = fabsf(vec3_dot(p1.p, n));
        float d2 = fabsf(vec3_dot(p2.p, n));

        support_point p = d1 > d2 ? p1 : p2;

        if (same_support_point(p, verts[0]) ||
            same_support_point(p, verts[1]) ||
            same_support_point(p, verts[2]))
            return 0;

        if (fabsf(vec3_dot(vec3_sub(p.p, A), n)) <= EPA_TOL)
            return 0;

        verts[3] = p;
        *vert_count = 4;
        return 1;
    }

    if (*vert_count == 2) {
        vec3 A = verts[0].p;
        vec3 B = verts[1].p;
        vec3 AB = vec3_sub(B, A);

        if (vec3_norm_sq(AB) <= COLL_EPS * COLL_EPS)
            return 0;

        /* First, grow the line segment into a triangle. */
        vec3 n1 = pick_any_perp(AB);
        if (vec3_norm_sq(n1) <= COLL_EPS * COLL_EPS)
            return 0;
        n1 = vec3_normalize(n1);

        support_point p2 = support_minkowski(a, b, n1);

        if (same_support_point(p2, verts[0]) || same_support_point(p2, verts[1])) {
            p2 = support_minkowski(a, b, vec3_neg(n1));
            if (same_support_point(p2, verts[0]) || same_support_point(p2, verts[1]))
                return 0;
        }

        vec3 AP2 = vec3_sub(p2.p, A);
        vec3 tri_n = vec3_cross(AB, AP2);
        if (vec3_norm_sq(tri_n) <= COLL_EPS * COLL_EPS)
            return 0;
        tri_n = vec3_normalize(tri_n);

        /* Then, grow the triangle into a tetrahedron using the triangle normal. */
        support_point p3a = support_minkowski(a, b, tri_n);
        support_point p3b = support_minkowski(a, b, vec3_neg(tri_n));

        float da = fabsf(vec3_dot(vec3_sub(p3a.p, A), tri_n));
        float db = fabsf(vec3_dot(vec3_sub(p3b.p, A), tri_n));

        support_point p3 = da > db ? p3a : p3b;

        if (same_support_point(p3, verts[0]) ||
            same_support_point(p3, verts[1]) ||
            same_support_point(p3, p2))
            return 0;

        /* Make sure the 4 points form a real tetrahedron, not a coplanar set. */
        float volume6 = vec3_dot(vec3_sub(p3.p, A), tri_n);
        if (fabsf(volume6) <= EPA_TOL)
            return 0;

        verts[2] = p2;
        verts[3] = p3;

        // for (int i = 0; i < 4; i++) {
        //     printk("verts[%d].p = (%f, %f, %f)\n",
        //         i,
        //         verts[i].p.x,
        //         verts[i].p.y,
        //         verts[i].p.z);
        // }

        *vert_count = 4;
        return 1;
    }

    return 0;
}

static void barycentric_on_triangle(vec3 A, vec3 B, vec3 C, vec3 P,
                                    float *wa, float *wb, float *wc) {
    vec3 v0 = vec3_sub(B, A);
    vec3 v1 = vec3_sub(C, A);
    vec3 v2 = vec3_sub(P, A);

    float d00 = vec3_dot(v0, v0);
    float d01 = vec3_dot(v0, v1);
    float d11 = vec3_dot(v1, v1);
    float d20 = vec3_dot(v2, v0);
    float d21 = vec3_dot(v2, v1);

    float denom = d00 * d11 - d01 * d01;
    if (fabsf(denom) <= COLL_EPS) {
        *wa = 1.0f;
        *wb = 0.0f;
        *wc = 0.0f;
        return;
    }

    *wb = (d11 * d20 - d01 * d21) / denom;
    *wc = (d00 * d21 - d01 * d20) / denom;
    *wa = 1.0f - *wb - *wc;
}

static void epa_fill_contact_result(epa_result *out,
                                    const epa_face *face,
                                    const support_point *verts) {
    const support_point *sa = &verts[face->a];
    const support_point *sb = &verts[face->b];
    const support_point *sc = &verts[face->c];

    /*
     * Closest point on the face to the origin.
     * For the converged EPA face, this lies approximately at
     * normal * dist.
     */
    vec3 p_closest = vec3_scale(face->normal, face->dist);

    float wa, wb, wc;
    barycentric_on_triangle(sa->p, sb->p, sc->p, p_closest, &wa, &wb, &wc);

    out->contact_a = vec3_lerp3(sa->supp_a, sb->supp_a, sc->supp_a, wa, wb, wc);
    out->contact_b = vec3_lerp3(sa->supp_b, sb->supp_b, sc->supp_b, wa, wb, wc);
    out->contact = vec3_scale(vec3_add(out->contact_a, out->contact_b), 0.5f);
}

epa_result phys_epa_from_gjk(const rigid_body *a, const rigid_body *b,
                             const simplex3 *gjk_simplex) {
    epa_result out;
    memset(&out, 0, sizeof(out));
    // printk("hello here\n");

    support_point verts[EPA_MAX_FACES];
    int vert_count = 0;

    if (!seed_epa_simplex(a, b, gjk_simplex, verts, &vert_count))
        return out;

    if (vert_count < 4)
        return out;

    epa_face faces[EPA_MAX_FACES];
    int face_count = 0;

    epa_make_face(&faces[face_count++], 0, 1, 2, verts);
    epa_make_face(&faces[face_count++], 0, 3, 1, verts);
    epa_make_face(&faces[face_count++], 0, 2, 3, verts);
    epa_make_face(&faces[face_count++], 1, 3, 2, verts);

    for (int iter = 0; iter < EPA_MAX_ITERS; ++iter) {
        int cf = epa_find_closest_face(faces, face_count);
        if (cf < 0)
            return out;

        epa_face *face = &faces[cf];
        support_point p = support_minkowski(a, b, face->normal);
        float d = vec3_dot(p.p, face->normal);

        int duplicate = 0;
        for (int i = 0; i < vert_count; ++i) {
            if (same_support_point(verts[i], p)) {
                duplicate = 1;
                break;
            }
        }

        // if (duplicate || d <= face->dist + EPA_TOL) {
        //     out.hit = 1;
        //     out.normal = face->normal;
        //     out.depth = face->dist;
        //     return out;
        // }

        if (duplicate || d <= face->dist + EPA_TOL) {
            out.hit = 1;
            out.normal = face->normal;
            out.depth = face->dist;
            epa_fill_contact_result(&out, face, verts);
            return out;
        }

        if (vert_count >= EPA_MAX_FACES)
            return out;

        int new_idx = vert_count++;
        verts[new_idx] = p;

        edge2i loose[EPA_MAX_LOOSE_EDGES];
        int loose_count = 0;

        for (int i = 0; i < face_count; ++i) {
            if (!faces[i].alive)
                continue;

            vec3 A = verts[faces[i].a].p;
            if (vec3_dot(faces[i].normal, vec3_sub(p.p, A)) > 0.0f) {
                faces[i].alive = 0;
                epa_add_edge(loose, &loose_count, faces[i].a, faces[i].b);
                epa_add_edge(loose, &loose_count, faces[i].b, faces[i].c);
                epa_add_edge(loose, &loose_count, faces[i].c, faces[i].a);
            }
        }

        if (loose_count == 0)
            return out;

        for (int i = 0; i < loose_count; ++i) {
            if (face_count >= EPA_MAX_FACES)
                return out;
            epa_make_face(&faces[face_count++], loose[i].a, loose[i].b, new_idx, verts);
        }
    }

    return out;
}

/* -------------------- full pipeline -------------------- */


collision_result phys_collide_convex(const rigid_body *a, const rigid_body *b) {
    collision_result out;
    memset(&out, 0, sizeof(out));

    // printk("CC0 enter\n");
    // printk("CC0a a=%x b=%x\n", (unsigned)a, (unsigned)b);

    aabb3 aa = phys_body_compute_aabb(a);
    // printk("CC1 got aa\n");

    aabb3 bb = phys_body_compute_aabb(b);
    // printk("CC2 got bb\n");

    // printk_float("aa.min.x", aa.min.x);
    // printk_float("aa.min.y", aa.min.y);
    // printk_float("aa.min.z", aa.min.z);
    // printk_float("aa.max.x", aa.max.x);
    // printk_float("aa.max.y", aa.max.y);
    // printk_float("aa.max.z", aa.max.z);

    // printk_float("bb.min.x", bb.min.x);
    // printk_float("bb.min.y", bb.min.y);
    // printk_float("bb.min.z", bb.min.z);
    // printk_float("bb.max.x", bb.max.x);
    // printk_float("bb.max.y", bb.max.y);
    // printk_float("bb.max.z", bb.max.z);

    // printk("CC3 before aabb overlap\n");
    int overlap = phys_aabb_overlap(aa, bb);
    // printk("CC4 overlap=%d\n", overlap);

    if (!overlap) {
        // printk("CC5 return miss from aabb\n");
        return out;
    }

    // printk("CC6 before gjk\n");
    out.gjk = phys_gjk_intersect(a, b);
    // printk("CC7 after gjk\n");
    // printk("CC7a gjk.hit=%d\n", out.gjk.hit);
    // printk("CC7b gjk.simplex.count=%d\n", out.gjk.simplex.count);

    if (!out.gjk.hit) {
        // printk("CC8 return miss from gjk\n");
        return out;
    }

    // printk("CC9 gjk says hit\n");

    out.hit = 1;
    // printk("CC10 out.hit set\n");

    // printk("CC11 before epa\n");
    out.epa = phys_epa_from_gjk(a, b, &out.gjk.simplex);
    // printk("CC12 after epa\n");

    // printk("CC12a epa.hit=%d\n", out.epa.hit);
    // printk_float("epa.depth", out.epa.depth);
    // printk_float("epa.normal.x", out.epa.normal.x);
    // printk_float("epa.normal.y", out.epa.normal.y);
    // printk_float("epa.normal.z", out.epa.normal.z);

    // printk_float("epa.contact_a.x", out.epa.contact_a.x);
    // printk_float("epa.contact_a.y", out.epa.contact_a.y);
    // printk_float("epa.contact_a.z", out.epa.contact_a.z);

    // printk_float("epa.contact_b.x", out.epa.contact_b.x);
    // printk_float("epa.contact_b.y", out.epa.contact_b.y);
    // printk_float("epa.contact_b.z", out.epa.contact_b.z);

    // printk_float("epa.contact.x", out.epa.contact.x);
    // printk_float("epa.contact.y", out.epa.contact.y);
    // printk_float("epa.contact.z", out.epa.contact.z);

    // printk("CC13 return hit\n");
    return out;
}