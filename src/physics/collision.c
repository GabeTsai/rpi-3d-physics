#include "collision.h"
#include "vec3.h"
#include "quat.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "rpi.h"

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

#define GJK_MAX_ITERS 32
#define EPA_MAX_ITERS 64
#define EPA_MAX_FACES 128
#define EPA_MAX_LOOSE_EDGES 64

#define COLL_EPS 1e-6f
#define GJK_DUP_EPS 1e-8f
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
    switch (g->type) {
    case PHYS_SHAPE_SPHERE: {
        vec3 n = vec3_normalize(dir_local);
        return vec3_scale(n, g->shape.sphere.radius);
    }
    case PHYS_SHAPE_BOX: {
        vec3 p = {
            .x=(dir_local.x >= 0.0f ? g->shape.box.hx : -g->shape.box.hx),
            .y=(dir_local.y >= 0.0f ? g->shape.box.hy : -g->shape.box.hy),
            .z=(dir_local.z >= 0.0f ? g->shape.box.hz : -g->shape.box.hz)
        };
        return p;
    }
    case PHYS_SHAPE_CAPSULE: {
        /* Assume capsule axis is local +Y/-Y */
        float r = g->shape.capsule.radius;
        float hh = g->shape.capsule.half_height;

        vec3 n = vec3_normalize(dir_local);
        vec3 cap_center = {.x=0.0f, .y=(dir_local.y >= 0.0f ? hh : -hh), .z=0.0f};
        return vec3_add(cap_center, vec3_scale(n, r));
    }
    case PHYS_SHAPE_MESH:
        return support_local_mesh(&g->shape.mesh, dir_local);
    default: {
        vec3 z = {.x=0,.y=0,.z=0};
        return z;
    }
    }
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
        vec3 z = {.x=0,.y=0,.z=0};
        out.min = z;
        out.max = z;
        return out;
    }

    switch (b->geom.type) {
    case PHYS_SHAPE_SPHERE: {
        float r = b->geom.shape.sphere.radius;
        vec3 e = {.x=r,.y=r,.z=r};
        out.min = vec3_sub(b->state.position, e);
        out.max = vec3_add(b->state.position, e);
        break;
    }
    case PHYS_SHAPE_BOX: {
        float hx = b->geom.shape.box.hx;
        float hy = b->geom.shape.box.hy;
        float hz = b->geom.shape.box.hz;
        vec3 corners[8] = {
            {.x=-hx,.y=-hy,-hz}, {.x=hx,.y=-hy,.z=-hz}, {.x=-hx,.y=hy,.z=-hz}, {.x=hx,.y=hy,.z=-hz},
            {.x=-hx,.y=-hy,.z=hz}, {.x=hx,.y=-hy,.z=hz}, {.x=-hx,.y=hy,.z=hz}, {.x=hx,.y=hy,.z=hz}
        };
        for (int i = 0; i < 8; ++i)
            expand_aabb_point(&out, local_to_world_point(b, corners[i]));
        break;
    }
    case PHYS_SHAPE_CAPSULE: {
        /* conservative AABB using support in cardinal directions */
        vec3 dirs[6] = {
            {.x=1,.y=0,.z=0}, {.x=-1,.y=0,.z=0},
            {.x=0,.y=1,.z=0}, {.x=0,.y=-1,.z=0},
            {.x=0,.y=0,.z=1}, {.x=0,.y=0,.z=-1}
        };
        for (int i = 0; i < 6; ++i)
            expand_aabb_point(&out, support_world_shape(b, dirs[i]));
        break;
    }
    case PHYS_SHAPE_MESH: {
        const mesh_geom *m = &b->geom.shape.mesh;
        for (int i = 0; i < m->triangle_count; ++i) {
            expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v0));
            expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v1));
            expand_aabb_point(&out, local_to_world_point(b, m->triangles[i].v2));
        }
        break;
    }
    default: {
        out.min = b->state.position;
        out.max = b->state.position;
        break;
    }
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
    vec3 A = s->pts[2].p;
    vec3 B = s->pts[1].p;
    vec3 C = s->pts[0].p;

    vec3 AO = vec3_neg(A);
    vec3 AB = vec3_sub(B, A);
    vec3 AC = vec3_sub(C, A);
    vec3 ABC = vec3_cross(AB, AC);

    vec3 ab_perp = vec3_cross(ABC, AB);
    if (vec3_dot(ab_perp, AO) > 0.0f) {
        s->pts[0] = s->pts[1];
        s->pts[1] = s->pts[2];
        s->count = 2;
        *dir = vec3_cross(vec3_cross(AB, AO), AB);
        return 0;
    }

    vec3 ac_perp = vec3_cross(AC, ABC);
    if (vec3_dot(ac_perp, AO) > 0.0f) {
        s->pts[1] = s->pts[2];
        s->count = 2;
        *dir = vec3_cross(vec3_cross(AC, AO), AC);
        return 0;
    }

    if (vec3_dot(ABC, AO) > 0.0f) {
        *dir = ABC;
    } else {
        support_point tmp = s->pts[0];
        s->pts[0] = s->pts[1];
        s->pts[1] = tmp;
        *dir = vec3_neg(ABC);
    }
    return 0;
}

static int do_simplex_tetra(simplex3 *s, vec3 *dir) {
    /* A newest = pts[3], B = pts[2], C = pts[1], D = pts[0] */
    vec3 A = s->pts[3].p;
    vec3 B = s->pts[2].p;
    vec3 C = s->pts[1].p;
    vec3 D = s->pts[0].p;

    vec3 AO = vec3_neg(A);

    vec3 AB = vec3_sub(B, A);
    vec3 AC = vec3_sub(C, A);
    vec3 AD = vec3_sub(D, A);

    vec3 ABC = vec3_cross(AB, AC);
    vec3 ACD = vec3_cross(AC, AD);
    vec3 ADB = vec3_cross(AD, AB);

    if (vec3_dot(ABC, AO) > 0.0f) {
        s->pts[0] = s->pts[1];
        s->pts[1] = s->pts[2];
        s->pts[2] = s->pts[3];
        s->count = 3;
        *dir = ABC;
        return 0;
    }

    if (vec3_dot(ACD, AO) > 0.0f) {
        s->pts[1] = s->pts[3];
        s->count = 3;
        *dir = ACD;
        return 0;
    }

    if (vec3_dot(ADB, AO) > 0.0f) {
        support_point oldD = s->pts[0];
        support_point oldB = s->pts[2];
        support_point oldA = s->pts[3];
        s->pts[0] = oldD;
        s->pts[1] = oldB;
        s->pts[2] = oldA;
        s->count = 3;
        *dir = ADB;
        return 0;
    }

    return 1; /* origin enclosed */
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

    simplex3 s;
    memset(&s, 0, sizeof(s));

    support_point first = support_minkowski(a, b, dir);
    s.pts[0] = first;
    s.count = 1;
    dir = vec3_neg(first.p);

    for (int iter = 0; iter < GJK_MAX_ITERS; ++iter) {
        if (vec3_norm_sq(dir) < COLL_EPS) {
            out.hit = 1;
            out.simplex = s;
            return out;
        }

        support_point p = support_minkowski(a, b, dir);

        if (simplex_has_duplicate(&s, p)) {
            out.hit = 0;
            out.simplex = s;
            out.search_dir = dir;
            return out;
        }

        if (vec3_dot(p.p, dir) < 0.0f) {
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
    n = vec3_normalize(n);


    if (vec3_dot(n, A) < 0.0f) {
        int t = f->b;
        f->b = f->c;
        f->c = t;

        B = verts[f->b].p;
        C = verts[f->c].p;
        n = vec3_cross(vec3_sub(B, A), vec3_sub(C, A));
        n = vec3_normalize(n);
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

        for (int i = 0; i < 4; i++) {
            printk("verts[%d].p = (%f, %f, %f)\n",
                i,
                verts[i].p.x,
                verts[i].p.y,
                verts[i].p.z);
        }

        *vert_count = 4;
        return 1;
    }

    return 0;
}

epa_result phys_epa_from_gjk(const rigid_body *a, const rigid_body *b,
                             const simplex3 *gjk_simplex) {
    epa_result out;
    memset(&out, 0, sizeof(out));
    printk("hello here\n");

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

        if (duplicate || d <= face->dist + EPA_TOL) {
            out.hit = 1;
            out.normal = face->normal;
            out.depth = face->dist;
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

    aabb3 aa = phys_body_compute_aabb(a);
    aabb3 bb = phys_body_compute_aabb(b);

    if (!phys_aabb_overlap(aa, bb))
        return out;

    out.gjk = phys_gjk_intersect(a, b);
    if (!out.gjk.hit)
        return out;

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