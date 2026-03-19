#include "collision.h"
#include "vec3.h"
#include "quat.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "rpi.h"

#define GJK_MAX_ITERS          32
#define EPA_MAX_ITERS          64
#define EPA_MAX_VERTS          128
#define EPA_MAX_FACES          128
#define EPA_MAX_LOOSE_EDGES    64

#define COLL_EPS               1e-6f
#define GJK_DUP_EPS            1e-6f
#define EPA_TOL                1e-4f
#define FINITE_LIMIT           1e30f

typedef struct {
    int a, b;
} edge2i;

typedef struct {
    int a, b, c;
    vec3 normal;
    float dist;
    int alive;
} epa_face;

static int finite_f(float x) {
    // NaN check: NaN != NaN
    if (x != x)
        return 0;

    // Infinity check: compare against large bound
    if (x > 1e30f || x < -1e30f)
        return 0;

    return 1;
}

static int finite_vec3(vec3 v) {
    return finite_f(v.x) && finite_f(v.y) && finite_f(v.z);
}

static int finite_quat(quat q) {
    return finite_f(q.w) && finite_f(q.x) && finite_f(q.y) && finite_f(q.z);
}

static quat quat_identity_safe(void) {
    quat q = {1.0f, 0.0f, 0.0f, 0.0f};
    return q;
}

static float safe_norm_sq(vec3 v) {
    if (!finite_vec3(v))
        return 0.0f;
    return vec3_dot(v, v);
}

static vec3 safe_normalize_vec3(vec3 v) {
    float n2 = safe_norm_sq(v);
    if (n2 <= COLL_EPS * COLL_EPS)
        return vec3_zero();

    float invn = 1.0f / sqrtf(n2);
    if (!finite_f(invn))
        return vec3_zero();

    v = vec3_scale(v, invn);
    if (!finite_vec3(v))
        return vec3_zero();

    return v;
}

static int safe_support_index_ok(int idx, int count) {
    return idx >= 0 && idx < count;
}

static int body_basic_ok(const rigid_body *b) {
    if (!b)
        return 0;
    if (!finite_vec3(b->state.position))
        return 0;
    if (!finite_quat(b->state.orientation))
        return 0;
    return 1;
}

static int mesh_ok(const mesh_geom *m) {
    if (!m)
        return 0;
    if (!m->triangles)
        return 0;
    if (m->triangle_count <= 0)
        return 0;
    return 1;
}

static int support_point_ok(support_point p) {
    return finite_vec3(p.p) && finite_vec3(p.supp_a) && finite_vec3(p.supp_b);
}

static vec3 safe_quat_rotate_vec3(quat q, vec3 v) {
    if (!finite_quat(q) || !finite_vec3(v))
        return vec3_zero();

    vec3 out = quat_rotate_vec3(q, v);
    if (!finite_vec3(out))
        return vec3_zero();
    return out;
}

static int simplex_push(simplex3 *s, support_point p) {
    if (!s)
        return 0;
    if (s->count < 0 || s->count > 4)
        return 0;
    if (s->count >= 4)
        return 0;
    if (!support_point_ok(p))
        return 0;

    s->pts[s->count++] = p;
    return 1;
}

static int epa_vert_push(support_point *verts, int *vert_count, support_point p) {
    if (!verts || !vert_count)
        return 0;
    if (*vert_count < 0 || *vert_count >= EPA_MAX_VERTS)
        return 0;
    if (!support_point_ok(p))
        return 0;

    verts[*vert_count] = p;
    (*vert_count)++;
    return 1;
}

static int epa_face_push(epa_face *faces, int *face_count, epa_face f) {
    if (!faces || !face_count)
        return 0;
    if (*face_count < 0 || *face_count >= EPA_MAX_FACES)
        return 0;

    faces[*face_count] = f;
    (*face_count)++;
    return 1;
}

/* ============================================================
 * transforms
 * ============================================================ */

static vec3 rotate_vec(const quat *q, vec3 v) {
    if (!q)
        return vec3_zero();
    return safe_quat_rotate_vec3(*q, v);
}

static vec3 inv_rotate_vec(const quat *q, vec3 v) {
    if (!q || !finite_quat(*q) || !finite_vec3(v))
        return vec3_zero();

    quat qc = quat_conjugate(*q);
    if (!finite_quat(qc))
        return vec3_zero();

    return safe_quat_rotate_vec3(qc, v);
}

static vec3 local_to_world_point(const rigid_body *b, vec3 p_local) {
    if (!body_basic_ok(b) || !finite_vec3(p_local))
        return vec3_zero();

    vec3 r = rotate_vec(&b->state.orientation, p_local);
    vec3 out = vec3_add(r, b->state.position);

    if (!finite_vec3(out))
        return vec3_zero();
    return out;
}

/* ============================================================
 * support mapping
 * ============================================================ */

static vec3 support_local_mesh(const mesh_geom *m, vec3 dir_local) {
    vec3 best = vec3_zero();
    float best_dot = -FLT_MAX;
    int found = 0;

    if (!mesh_ok(m) || !finite_vec3(dir_local))
        return best;

    for (int i = 0; i < m->triangle_count; ++i) {
        const triangle *t = &m->triangles[i];
        const vec3 verts[3] = { t->v0, t->v1, t->v2 };

        for (int k = 0; k < 3; ++k) {
            if (!finite_vec3(verts[k]))
                continue;

            float d = vec3_dot(verts[k], dir_local);
            if (!finite_f(d))
                continue;

            if (!found || d > best_dot) {
                best_dot = d;
                best = verts[k];
                found = 1;
            }
        }
    }

    return found ? best : vec3_zero();
}

static vec3 support_local_shape(const rigid_body_geom *g, vec3 dir_local) {
    if (!g)
        return vec3_zero();
    return support_local_mesh(&g->mesh, dir_local);
}

static vec3 support_world_shape(const rigid_body *b, vec3 dir_world) {
    if (!body_basic_ok(b) || !finite_vec3(dir_world))
        return vec3_zero();

    vec3 dir_local = inv_rotate_vec(&b->state.orientation, dir_world);
    vec3 p_local = support_local_shape(&b->geom, dir_local);
    vec3 p_world = local_to_world_point(b, p_local);

    return finite_vec3(p_world) ? p_world : vec3_zero();
}

static support_point support_minkowski(const rigid_body *a,
                                       const rigid_body *b,
                                       vec3 dir_world) {
    support_point sp;
    memset(&sp, 0, sizeof(sp));

    if (!body_basic_ok(a) || !body_basic_ok(b) || !finite_vec3(dir_world))
        return sp;

    sp.supp_a = support_world_shape(a, dir_world);
    sp.supp_b = support_world_shape(b, vec3_neg(dir_world));
    sp.p = vec3_sub(sp.supp_a, sp.supp_b);

    if (!support_point_ok(sp))
        memset(&sp, 0, sizeof(sp));

    return sp;
}

/* ============================================================
 * AABB
 * ============================================================ */

static void expand_aabb_point(aabb3 *a, vec3 p) {
    if (!a || !finite_vec3(p))
        return;

    a->min = vec3_min(a->min, p);
    a->max = vec3_max(a->max, p);
}

aabb3 phys_body_compute_aabb(const rigid_body *b) {
    aabb3 out;
    memset(&out, 0, sizeof(out));

    out.min.x = out.min.y = out.min.z = FLT_MAX;
    out.max.x = out.max.y = out.max.z = -FLT_MAX;

    if (!body_basic_ok(b)) {
        out.min = vec3_zero();
        out.max = vec3_zero();
        return out;
    }

    const mesh_geom *m = &b->geom.mesh;
    if (!mesh_ok(m)) {
        out.min = b->state.position;
        out.max = b->state.position;
        return out;
    }

    for (int i = 0; i < m->triangle_count; ++i) {
        expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v0));
        expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v1));
        expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v2));
    }

    if (!finite_vec3(out.min) || !finite_vec3(out.max)) {
        out.min = b->state.position;
        out.max = b->state.position;
    }

    return out;
}

int phys_aabb_overlap(aabb3 a, aabb3 b) {
    if (!finite_vec3(a.min) || !finite_vec3(a.max) ||
        !finite_vec3(b.min) || !finite_vec3(b.max))
        return 0;

    if (a.max.x < b.min.x || b.max.x < a.min.x) return 0;
    if (a.max.y < b.min.y || b.max.y < a.min.y) return 0;
    if (a.max.z < b.min.z || b.max.z < a.min.z) return 0;
    return 1;
}

/* ============================================================
 * GJK simplex handling
 * ============================================================ */

static int simplex_has_duplicate(const simplex3 *s, support_point p) {
    if (!s || !support_point_ok(p))
        return 1;

    for (int i = 0; i < s->count; ++i) {
        vec3 d = vec3_sub(s->pts[i].p, p.p);
        if (safe_norm_sq(d) <= GJK_DUP_EPS)
            return 1;
    }
    return 0;
}

static int do_simplex_line(simplex3 *s, vec3 *dir) {
    vec3 A = s->pts[1].p;
    vec3 B = s->pts[0].p;
    vec3 AO = vec3_neg(A);
    vec3 AB = vec3_sub(B, A);

    if (vec3_dot(AB, AO) > 0.0f) {
        vec3 new_dir = vec3_cross(vec3_cross(AB, AO), AB);
        if (safe_norm_sq(new_dir) <= COLL_EPS * COLL_EPS)
            return 1;
        *dir = new_dir;
    } else {
        s->pts[0] = s->pts[1];
        s->count = 1;
        *dir = AO;
    }

    return 0;
}

static int do_simplex_triangle(simplex3 *s, vec3 *dir) {
    support_point Ap = s->pts[2];
    support_point Bp = s->pts[1];
    support_point Cp = s->pts[0];

    vec3 A = Ap.p;
    vec3 B = Bp.p;
    vec3 C = Cp.p;

    vec3 AO = vec3_neg(A);
    vec3 AB = vec3_sub(B, A);
    vec3 AC = vec3_sub(C, A);

    float d1 = vec3_dot(AB, AO);
    float d2 = vec3_dot(AC, AO);

    if (d1 <= 0.0f && d2 <= 0.0f) {
        s->pts[0] = Ap;
        s->count = 1;
        *dir = AO;
        return 0;
    }

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
        if (safe_norm_sq(nd) <= COLL_EPS * COLL_EPS)
            return 1;
        *dir = nd;
        return 0;
    }

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
        if (safe_norm_sq(nd) <= COLL_EPS * COLL_EPS)
            return 1;
        *dir = nd;
        return 0;
    }

    vec3 ABC = vec3_cross(AB, AC);
    if (safe_norm_sq(ABC) <= COLL_EPS * COLL_EPS)
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

    if (vec3_dot(ABC, AD) > 0.0f) ABC = vec3_neg(ABC);
    if (vec3_dot(ACD, AB) > 0.0f) ACD = vec3_neg(ACD);
    if (vec3_dot(ADB, AC) > 0.0f) ADB = vec3_neg(ADB);

    if (vec3_dot(ABC, AO) > 0.0f) {
        s->pts[0] = Cp;
        s->pts[1] = Bp;
        s->pts[2] = Ap;
        s->count = 3;
        *dir = ABC;
        return 0;
    }

    if (vec3_dot(ACD, AO) > 0.0f) {
        s->pts[0] = Dp;
        s->pts[1] = Cp;
        s->pts[2] = Ap;
        s->count = 3;
        *dir = ACD;
        return 0;
    }

    if (vec3_dot(ADB, AO) > 0.0f) {
        s->pts[0] = Bp;
        s->pts[1] = Dp;
        s->pts[2] = Ap;
        s->count = 3;
        *dir = ADB;
        return 0;
    }

    return 1;
}

static int do_simplex(simplex3 *s, vec3 *dir) {
    if (!s || !dir)
        return 0;

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

/* ============================================================
 * GJK
 * ============================================================ */

gjk_result phys_gjk_intersect(const rigid_body *a, const rigid_body *b) {
    gjk_result out;
    memset(&out, 0, sizeof(out));

    if (!body_basic_ok(a) || !body_basic_ok(b))
        return out;

    vec3 dir = vec3_sub(b->state.position, a->state.position);
    if (safe_norm_sq(dir) < COLL_EPS)
        dir = (vec3){ .x = 1.0f, .y = 0.0f, .z = 0.0f };

    simplex3 s;
    memset(&s, 0, sizeof(s));

    support_point first = support_minkowski(a, b, dir);
    if (!support_point_ok(first))
        return out;

    s.pts[0] = first;
    s.count = 1;
    dir = vec3_neg(first.p);

    for (int iter = 0; iter < GJK_MAX_ITERS; ++iter) {
        if (safe_norm_sq(dir) < COLL_EPS) {
            out.hit = 1;
            out.simplex = s;
            return out;
        }

        support_point p = support_minkowski(a, b, dir);
        if (!support_point_ok(p)) {
            out.hit = 0;
            out.simplex = s;
            out.search_dir = dir;
            return out;
        }

        if (simplex_has_duplicate(&s, p)) {
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

        if (!simplex_push(&s, p)) {
            out.hit = 0;
            out.simplex = s;
            out.search_dir = dir;
            return out;
        }

        if (do_simplex(&s, &dir)) {
            out.hit = 1;
            out.simplex = s;
            return out;
        }

        if (!finite_vec3(dir)) {
            out.hit = 0;
            out.simplex = s;
            return out;
        }
    }

    out.hit = 0;
    out.simplex = s;
    out.search_dir = dir;
    return out;
}

/* ============================================================
 * EPA
 * ============================================================ */

static epa_face epa_face_zero(void) {
    epa_face f;
    memset(&f, 0, sizeof(f));
    return f;
}

static epa_face epa_make_face_value(int a, int b, int c,
                                    const support_point *verts,
                                    int vert_count) {
    epa_face f = epa_face_zero();

    if (!verts)
        return f;
    if (!safe_support_index_ok(a, vert_count) ||
        !safe_support_index_ok(b, vert_count) ||
        !safe_support_index_ok(c, vert_count))
        return f;

    f.a = a;
    f.b = b;
    f.c = c;
    f.alive = 1;

    vec3 A = verts[a].p;
    vec3 B = verts[b].p;
    vec3 C = verts[c].p;

    vec3 n = vec3_cross(vec3_sub(B, A), vec3_sub(C, A));
    float n2 = safe_norm_sq(n);
    if (n2 <= COLL_EPS * COLL_EPS) {
        f.alive = 0;
        return f;
    }

    n = vec3_scale(n, 1.0f / sqrtf(n2));
    if (!finite_vec3(n)) {
        f.alive = 0;
        return f;
    }

    if (vec3_dot(n, A) < 0.0f) {
        int t = f.b;
        f.b = f.c;
        f.c = t;

        B = verts[f.b].p;
        C = verts[f.c].p;
        n = vec3_cross(vec3_sub(B, A), vec3_sub(C, A));
        n2 = safe_norm_sq(n);
        if (n2 <= COLL_EPS * COLL_EPS) {
            f.alive = 0;
            return f;
        }

        n = vec3_scale(n, 1.0f / sqrtf(n2));
        if (!finite_vec3(n)) {
            f.alive = 0;
            return f;
        }
    }

    f.normal = n;
    f.dist = vec3_dot(n, A);

    if (!finite_f(f.dist)) {
        f.alive = 0;
        f.dist = 0.0f;
    }

    return f;
}

static int epa_find_closest_face(epa_face *faces, int face_count) {
    int best = -1;
    float best_dist = FLT_MAX;

    if (!faces || face_count <= 0)
        return -1;

    for (int i = 0; i < face_count; ++i) {
        if (!faces[i].alive)
            continue;
        if (!finite_f(faces[i].dist))
            continue;
        if (faces[i].dist < best_dist) {
            best_dist = faces[i].dist;
            best = i;
        }
    }
    return best;
}

static void epa_add_edge(edge2i *edges, int *edge_count, int a, int b) {
    if (!edges || !edge_count)
        return;

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
    vec3 axis =
        (fabsf(v.x) < 0.577f) ? (vec3){.x=1,.y=0,.z=0} :
        (fabsf(v.y) < 0.577f) ? (vec3){.x=0,.y=1,.z=0} :
                                (vec3){.x=0,.y=0,.z=1};

    vec3 out = vec3_cross(v, axis);
    return finite_vec3(out) ? out : vec3_zero();
}

static int same_support_point(support_point a, support_point b) {
    vec3 d = vec3_sub(a.p, b.p);
    return safe_norm_sq(d) <= GJK_DUP_EPS;
}

static int seed_epa_simplex(const rigid_body *a, const rigid_body *b,
                            const simplex3 *gjk_simplex,
                            support_point *verts, int *vert_count) {
    if (!a || !b || !gjk_simplex || !verts || !vert_count)
        return 0;

    if (gjk_simplex->count < 2 || gjk_simplex->count > 4)
        return 0;

    *vert_count = 0;
    for (int i = 0; i < gjk_simplex->count; ++i) {
        if (!epa_vert_push(verts, vert_count, gjk_simplex->pts[i]))
            return 0;
    }

    if (*vert_count == 4)
        return 1;

    if (*vert_count == 3) {
        vec3 A = verts[0].p;
        vec3 B = verts[1].p;
        vec3 C = verts[2].p;

        vec3 n = vec3_cross(vec3_sub(B, A), vec3_sub(C, A));
        if (safe_norm_sq(n) <= COLL_EPS * COLL_EPS)
            return 0;

        n = safe_normalize_vec3(n);
        if (safe_norm_sq(n) <= COLL_EPS * COLL_EPS)
            return 0;

        support_point p1 = support_minkowski(a, b, n);
        support_point p2 = support_minkowski(a, b, vec3_neg(n));

        float d1 = fabsf(vec3_dot(p1.p, n));
        float d2 = fabsf(vec3_dot(p2.p, n));
        support_point p = (d1 > d2) ? p1 : p2;

        if (same_support_point(p, verts[0]) ||
            same_support_point(p, verts[1]) ||
            same_support_point(p, verts[2]))
            return 0;

        if (fabsf(vec3_dot(vec3_sub(p.p, A), n)) <= EPA_TOL)
            return 0;

        return epa_vert_push(verts, vert_count, p);
    }

    if (*vert_count == 2) {
        vec3 A = verts[0].p;
        vec3 B = verts[1].p;
        vec3 AB = vec3_sub(B, A);

        if (safe_norm_sq(AB) <= COLL_EPS * COLL_EPS)
            return 0;

        vec3 n1 = pick_any_perp(AB);
        if (safe_norm_sq(n1) <= COLL_EPS * COLL_EPS)
            return 0;
        n1 = safe_normalize_vec3(n1);

        support_point p2 = support_minkowski(a, b, n1);
        if (same_support_point(p2, verts[0]) || same_support_point(p2, verts[1])) {
            p2 = support_minkowski(a, b, vec3_neg(n1));
            if (same_support_point(p2, verts[0]) || same_support_point(p2, verts[1]))
                return 0;
        }

        vec3 AP2 = vec3_sub(p2.p, A);
        vec3 tri_n = vec3_cross(AB, AP2);
        if (safe_norm_sq(tri_n) <= COLL_EPS * COLL_EPS)
            return 0;
        tri_n = safe_normalize_vec3(tri_n);

        support_point p3a = support_minkowski(a, b, tri_n);
        support_point p3b = support_minkowski(a, b, vec3_neg(tri_n));

        float da = fabsf(vec3_dot(vec3_sub(p3a.p, A), tri_n));
        float db = fabsf(vec3_dot(vec3_sub(p3b.p, A), tri_n));
        support_point p3 = (da > db) ? p3a : p3b;

        if (same_support_point(p3, verts[0]) ||
            same_support_point(p3, verts[1]) ||
            same_support_point(p3, p2))
            return 0;

        float volume6 = vec3_dot(vec3_sub(p3.p, A), tri_n);
        if (fabsf(volume6) <= EPA_TOL)
            return 0;

        if (!epa_vert_push(verts, vert_count, p2))
            return 0;
        if (!epa_vert_push(verts, vert_count, p3))
            return 0;

        *vert_count = 4;
        return 1;
    }

    return 0;
}

static void barycentric_on_triangle(vec3 A, vec3 B, vec3 C, vec3 P,
                                    float *wa, float *wb, float *wc) {
    if (!wa || !wb || !wc) return;

    vec3 v0 = vec3_sub(B, A);
    vec3 v1 = vec3_sub(C, A);
    vec3 v2 = vec3_sub(P, A);

    float d00 = vec3_dot(v0, v0);
    float d01 = vec3_dot(v0, v1);
    float d11 = vec3_dot(v1, v1);
    float d20 = vec3_dot(v2, v0);
    float d21 = vec3_dot(v2, v1);

    float denom = d00 * d11 - d01 * d01;
    if (!finite_f(denom) || fabsf(denom) <= COLL_EPS) {
        *wa = 1.0f;
        *wb = 0.0f;
        *wc = 0.0f;
        return;
    }

    *wb = (d11 * d20 - d01 * d21) / denom;
    *wc = (d00 * d21 - d01 * d20) / denom;
    *wa = 1.0f - *wb - *wc;

    if (!finite_f(*wa) || !finite_f(*wb) || !finite_f(*wc)) {
        *wa = 1.0f;
        *wb = 0.0f;
        *wc = 0.0f;
    }
}

static void epa_fill_contact_result(epa_result *out,
                                    const epa_face *face,
                                    const support_point *verts,
                                    int vert_count) {
    if (!out || !face || !verts)
        return;
    if (!safe_support_index_ok(face->a, vert_count) ||
        !safe_support_index_ok(face->b, vert_count) ||
        !safe_support_index_ok(face->c, vert_count))
        return;

    const support_point *sa = &verts[face->a];
    const support_point *sb = &verts[face->b];
    const support_point *sc = &verts[face->c];

    vec3 p_closest = vec3_scale(face->normal, face->dist);

    float wa, wb, wc;
    barycentric_on_triangle(sa->p, sb->p, sc->p, p_closest, &wa, &wb, &wc);

    out->contact_a = vec3_lerp3(sa->supp_a, sb->supp_a, sc->supp_a, wa, wb, wc);
    out->contact_b = vec3_lerp3(sa->supp_b, sb->supp_b, sc->supp_b, wa, wb, wc);
    out->contact   = vec3_scale(vec3_add(out->contact_a, out->contact_b), 0.5f);

    if (!finite_vec3(out->contact_a)) out->contact_a = vec3_zero();
    if (!finite_vec3(out->contact_b)) out->contact_b = vec3_zero();
    if (!finite_vec3(out->contact))   out->contact   = vec3_zero();
}

epa_result phys_epa_from_gjk(const rigid_body *a, const rigid_body *b,
                             const simplex3 *gjk_simplex) {
    epa_result out;
    memset(&out, 0, sizeof(out));

    if (!body_basic_ok(a) || !body_basic_ok(b) || !gjk_simplex)
        return out;

    support_point verts[EPA_MAX_VERTS];
    memset(verts, 0, sizeof(verts));
    int vert_count = 0;

    if (!seed_epa_simplex(a, b, gjk_simplex, verts, &vert_count))
        return out;
    if (vert_count < 4)
        return out;

    epa_face faces[EPA_MAX_FACES];
    memset(faces, 0, sizeof(faces));
    int face_count = 0;

    {
        epa_face f;
        f = epa_make_face_value(0, 1, 2, verts, vert_count); if (!epa_face_push(faces, &face_count, f)) return out;
        f = epa_make_face_value(0, 3, 1, verts, vert_count); if (!epa_face_push(faces, &face_count, f)) return out;
        f = epa_make_face_value(0, 2, 3, verts, vert_count); if (!epa_face_push(faces, &face_count, f)) return out;
        f = epa_make_face_value(1, 3, 2, verts, vert_count); if (!epa_face_push(faces, &face_count, f)) return out;
    }

    for (int iter = 0; iter < EPA_MAX_ITERS; ++iter) {
        int cf = epa_find_closest_face(faces, face_count);
        if (cf < 0)
            return out;

        epa_face *face = &faces[cf];
        if (!face->alive || !finite_vec3(face->normal) || !finite_f(face->dist))
            return out;

        support_point p = support_minkowski(a, b, face->normal);
        if (!support_point_ok(p))
            return out;

        float d = vec3_dot(p.p, face->normal);
        if (!finite_f(d))
            return out;

        int duplicate = 0;
        for (int i = 0; i < vert_count; ++i) {
            if (same_support_point(verts[i], p)) {
                duplicate = 1;
                break;
            }
        }

        if (duplicate || d <= face->dist + EPA_TOL) {
            out.hit = 1;
            out.normal = face->normal;
            out.depth = face->dist;
            epa_fill_contact_result(&out, face, verts, vert_count);

            if (!finite_vec3(out.normal)) out.normal = vec3_zero();
            if (!finite_f(out.depth))     out.depth = 0.0f;
            return out;
        }

        if (!epa_vert_push(verts, &vert_count, p))
            return out;
        int new_idx = vert_count - 1;

        edge2i loose[EPA_MAX_LOOSE_EDGES];
        memset(loose, 0, sizeof(loose));
        int loose_count = 0;

        for (int i = 0; i < face_count; ++i) {
            if (!faces[i].alive)
                continue;

            if (!safe_support_index_ok(faces[i].a, vert_count) ||
                !safe_support_index_ok(faces[i].b, vert_count) ||
                !safe_support_index_ok(faces[i].c, vert_count))
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
            epa_face f = epa_make_face_value(loose[i].a, loose[i].b, new_idx,
                                             verts, vert_count);
            if (!epa_face_push(faces, &face_count, f))
                return out;
        }
    }

    return out;
}

/* ============================================================
 * optimizations
 * ============================================================ */

static collision_result phys_collide_sphere_sphere(const rigid_body *a,
                                                   const rigid_body *b) {
    collision_result out;
    memset(&out, 0, sizeof(out));

    if (!body_basic_ok(a) || !body_basic_ok(b))
        return out;

    vec3 pa = a->state.position;
    vec3 pb = b->state.position;

    float ra = a->geom.mesh.radius;
    float rb = b->geom.mesh.radius;

    vec3 d = vec3_sub(pb, pa);
    float d2 = vec3_dot(d, d);
    float rsum = ra + rb;
    float rsum2 = rsum * rsum;

    if (!finite_f(d2) || !finite_f(rsum))
        return out;

    if (d2 > rsum2)
        return out;

    out.hit = 1;
    out.gjk.hit = 1;
    out.epa.hit = 1;

    if (d2 > 1e-12f) {
        float dist = sqrtf(d2);
        float invd = 1.0f / dist;
        vec3 n = vec3_scale(d, invd);

        out.epa.normal = n;
        out.epa.depth = rsum - dist;

        out.epa.contact_a = vec3_add(pa, vec3_scale(n, ra));
        out.epa.contact_b = vec3_sub(pb, vec3_scale(n, rb));
        out.epa.contact = vec3_scale(vec3_add(out.epa.contact_a, out.epa.contact_b), 0.5f);
    } else {
        /* coincident centers: choose an arbitrary stable normal */
        vec3 n = vec3_make(0.0f, 1.0f, 0.0f);

        out.epa.normal = n;
        out.epa.depth = rsum;

        out.epa.contact_a = vec3_add(pa, vec3_scale(n, ra));
        out.epa.contact_b = vec3_sub(pb, vec3_scale(n, rb));
        out.epa.contact = vec3_scale(vec3_add(out.epa.contact_a, out.epa.contact_b), 0.5f);
    }

    return out;
}

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static collision_result phys_collide_box_sphere(const rigid_body *box,
                                                const rigid_body *sphere) {
    collision_result out;
    memset(&out, 0, sizeof(out));

    if (!body_basic_ok(box) || !body_basic_ok(sphere))
        return out;

    vec3 box_pos = box->state.position;
    quat box_q   = box->state.orientation;
    quat box_qc  = quat_conjugate(box_q);

    vec3 sph_pos = sphere->state.position;
    float r = sphere->geom.mesh.radius;

    float hx = box->geom.mesh.hx;
    float hy = box->geom.mesh.hy;
    float hz = box->geom.mesh.hz;

    /* sphere center in box local coordinates */
    vec3 rel = vec3_sub(sph_pos, box_pos);
    vec3 p_local = quat_rotate_vec3(box_qc, rel);

    /* closest point on box in local coords */
    vec3 c_local = {
        .x = clampf(p_local.x, -hx, hx),
        .y = clampf(p_local.y, -hy, hy),
        .z = clampf(p_local.z, -hz, hz)
    };

    vec3 delta_local = vec3_sub(p_local, c_local);
    float d2 = vec3_dot(delta_local, delta_local);

    if (!finite_f(d2) || !finite_f(r))
        return out;

    /* no collision */
    if (d2 > r * r)
        return out;

    out.hit = 1;
    out.gjk.hit = 1;
    out.epa.hit = 1;

    /* sphere center outside or on surface */
    if (d2 > 1e-12f) {
        float dist = sqrtf(d2);
        float invd = 1.0f / dist;

        vec3 n_local = vec3_scale(delta_local, invd);
        vec3 n_world = quat_rotate_vec3(box_q, n_local);

        vec3 c_world = vec3_add(box_pos, quat_rotate_vec3(box_q, c_local));

        out.epa.normal = n_world;
        out.epa.depth = r - dist;

        /* contact on box */
        out.epa.contact_a = c_world;

        /* contact on sphere */
        out.epa.contact_b = vec3_sub(sph_pos, vec3_scale(n_world, r));

        out.epa.contact = vec3_scale(
            vec3_add(out.epa.contact_a, out.epa.contact_b), 0.5f);

        return out;
    }

    /* sphere center is inside the box */
    {
        float dx = hx - fabsf(p_local.x);
        float dy = hy - fabsf(p_local.y);
        float dz = hz - fabsf(p_local.z);

        vec3 n_local;
        vec3 c_local_face = p_local;

        if (dx <= dy && dx <= dz) {
            float sx = (p_local.x >= 0.0f) ? 1.0f : -1.0f;
            n_local = vec3_make(sx, 0.0f, 0.0f);
            c_local_face.x = sx * hx;
        } else if (dy <= dx && dy <= dz) {
            float sy = (p_local.y >= 0.0f) ? 1.0f : -1.0f;
            n_local = vec3_make(0.0f, sy, 0.0f);
            c_local_face.y = sy * hy;
        } else {
            float sz = (p_local.z >= 0.0f) ? 1.0f : -1.0f;
            n_local = vec3_make(0.0f, 0.0f, sz);
            c_local_face.z = sz * hz;
        }

        vec3 n_world = quat_rotate_vec3(box_q, n_local);
        vec3 c_world = vec3_add(box_pos, quat_rotate_vec3(box_q, c_local_face));

        /* penetration to nearest face + sphere radius */
        float face_dist = dx;
        if (dy < face_dist) face_dist = dy;
        if (dz < face_dist) face_dist = dz;

        out.epa.normal = n_world;
        out.epa.depth = r + face_dist;

        out.epa.contact_a = c_world;
        out.epa.contact_b = vec3_sub(sph_pos, vec3_scale(n_world, r));
        out.epa.contact = vec3_scale(
            vec3_add(out.epa.contact_a, out.epa.contact_b), 0.5f);

        return out;
    }
}

/* ============================================================
 * full pipeline
 * ============================================================ */

collision_result phys_collide_convex(const rigid_body *a, const rigid_body *b) {
    
    collision_result out;
    memset(&out, 0, sizeof(out));

    if (!body_basic_ok(a) || !body_basic_ok(b))
        return out;

    if (a->geom.mesh.type == SHAPE_SPHERE && b->geom.mesh.type == SHAPE_SPHERE) return phys_collide_sphere_sphere(a, b);
    if (a->geom.mesh.type == SHAPE_BOX && b->geom.mesh.type == SHAPE_SPHERE) return phys_collide_box_sphere(a, b);
    if (a->geom.mesh.type == SHAPE_SPHERE && b->geom.mesh.type == SHAPE_BOX) return phys_collide_box_sphere(b, a);
    aabb3 aa = phys_body_compute_aabb(a);
    aabb3 bb = phys_body_compute_aabb(b);

    if (!phys_aabb_overlap(aa, bb))
        return out;

    out.gjk = phys_gjk_intersect(a, b);
    if (!out.gjk.hit)
        return out;

    out.hit = 1;
    out.epa = phys_epa_from_gjk(a, b, &out.gjk.simplex);

    if (out.epa.hit) {
        if (!finite_vec3(out.epa.normal)) out.epa.normal = vec3_zero();
        if (!finite_f(out.epa.depth))     out.epa.depth = 0.0f;
        if (!finite_vec3(out.epa.contact_a)) out.epa.contact_a = vec3_zero();
        if (!finite_vec3(out.epa.contact_b)) out.epa.contact_b = vec3_zero();
        if (!finite_vec3(out.epa.contact))   out.epa.contact   = vec3_zero();
    }

    return out;
}

