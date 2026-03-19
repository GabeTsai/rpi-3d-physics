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

static void printk_float(const char *name, float v) {
    int sign = v < 0;
    if(sign) v = -v;

    int whole = (int)v;
    int frac  = (int)((v - whole) * 1000);   // 3 decimal places

    if(sign)
        printk("%s = -%d.%d\n", name, whole, frac);
    else
        printk("%s = %d.%d\n", name, whole, frac);
}

/* -------------------- transforms -------------------- */

static vec3 rotate_vec(const quat *q, vec3 v) {
    return quat_rotate_vec3(*q, v);
}

static vec3 inv_rotate_vec(const quat *q, vec3 v) {
    quat qc = quat_conjugate(*q);
    return quat_rotate_vec3(qc, v);
}

static vec3 local_to_world_point(const rigid_body *b, vec3 p_local) {
    return vec3_add(rotate_vec(&b->state.orientation, p_local), b->state.position);
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
    a->min = vec3_min(a->min, p);
    a->max = vec3_max(a->max, p);
}

aabb3 phys_body_compute_aabb(const rigid_body *b) {
    aabb3 out;
    out.min.x = out.min.y = out.min.z = FLT_MAX;
    out.max.x = out.max.y = out.max.z = -FLT_MAX;

    if (!b) {
        vec3 z = {.x = 0, .y = 0, .z = 0};
        out.min = z;
        out.max = z;
        return out;
    }

    const mesh_geom *m = &b->geom.mesh;

    if (!m->triangles || m->triangle_count <= 0) {
        out.min = b->state.position;
        out.max = b->state.position;
        return out;
    }

    for (int i = 0; i < m->triangle_count; ++i) {
        expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v0));
        expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v1));
        expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v2));
    }

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
    // printk("hi next\n");

    aabb3 aa = phys_body_compute_aabb(a);
    aabb3 bb = phys_body_compute_aabb(b);

    // printk("aa:\n");
    // printk_float("  min.x", aa.min.x);
    // printk_float("  min.y", aa.min.y);
    // printk_float("  min.z", aa.min.z);
    // printk_float("  max.x", aa.max.x);
    // printk_float("  max.y", aa.max.y);
    // printk_float("  max.z", aa.max.z);

    // printk("bb:\n");
    // printk_float("  min.x", bb.min.x);
    // printk_float("  min.y", bb.min.y);
    // printk_float("  min.z", bb.min.z);
    // printk_float("  max.x", bb.max.x);
    // printk_float("  max.y", bb.max.y);
    // printk_float("  max.z", bb.max.z);

    // printk("hi next 2\n");



    if (!phys_aabb_overlap(aa, bb))
        return out;

    // printk("hi next 3\n");

    // printk("hi next \n");

    out.gjk = phys_gjk_intersect(a, b);
    if (!out.gjk.hit)
        return out;

    // printk("hit \n");

    /* GJK already proved overlap.  Never let EPA failure erase that. */
    out.hit = 1;

    /*
     * EPA needs a tetrahedron (4-point simplex) enclosing the origin.
     * Your current GJK can report hit earlier from degenerate line/triangle
     * cases, so only run EPA when the simplex is big enough.
     */
    // if (out.gjk.simplex.count < 4) {
    //     return out;
    // }

    out.epa = phys_epa_from_gjk(a, b, &out.gjk.simplex);

    /*
     * Keep overlap=true even if EPA fails to converge or returns empty.
     * EPA only refines penetration normal/depth.
     */
    return out;
}