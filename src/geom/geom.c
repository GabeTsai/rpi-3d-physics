// geom.c
#include "geom.h"
#include "graphics-settings.h"

static int g_next_mesh_geom_id = 0;

triangle triangle_make_from_pts(float xs_1, float ys_1, float xs_2, float ys_2, float xs_3, float ys_3, float zs) {
    return (triangle){
        .v0 = vec3_make(xs_1, ys_1, zs),
        .v1 = vec3_make(xs_2, ys_2, zs),
        .v2 = vec3_make(xs_3, ys_3, zs)
    };
}

static triangle *mesh_alloc_triangles(int triangle_count) {
    if (triangle_count <= 0) return 0;
    return (triangle *)kmalloc(sizeof(triangle) * triangle_count);
}

static mesh_geom mesh_geom_make(
        int triangle_count, 
        triangle *triangles, 
        float r, float g, float b, int visible) {
    mesh_geom mesh;
    mesh.id = g_next_mesh_geom_id++;
    mesh.triangle_count = triangle_count;
    mesh.triangles = triangles;
    mesh.smooth_normals = false;
    mesh.r = r;
    mesh.g = g;
    mesh.b = b;
    mesh.visible = visible;
    return mesh;
}

// we can do this optimization for icospheres because
// all vertices lie on the sphere surface, so the normal 
// is just the vector from the center of the sphere to the vertex, normalized
void mesh_geom_compute_smooth_normals(mesh_geom *mesh) {
    int triangle_count = mesh->triangle_count;
    for (int i = 0; i < triangle_count; i++) {
        triangle *tri = &mesh->triangles[i];
        tri->n0 = vec3_normalize(tri->v0);
        tri->n1 = vec3_normalize(tri->v1);
        tri->n2 = vec3_normalize(tri->v2);
    }
    mesh->smooth_normals = true;
}

mesh_geom mesh_geom_init_custom_take(int triangle_count, triangle *triangles, float r, float g, float b, int visible) {
    return mesh_geom_make(triangle_count, triangles, r, g, b, visible);
}

mesh_geom mesh_geom_init_custom_copy(int triangle_count, const triangle *triangles, float r, float g, float b, int visible) {
    triangle *dst = mesh_alloc_triangles(triangle_count);
    for (int i = 0; i < triangle_count; i++)
        dst[i] = triangles[i];

    return mesh_geom_make(triangle_count, dst, r, g, b, visible);
}

mesh_geom mesh_geom_init_triangle(triangle tri, float r, float g, float b, int visible) {
    triangle *tris = mesh_alloc_triangles(1);
    tris[0] = tri;
    return mesh_geom_make(1, tris, r, g, b, visible);
}

mesh_geom mesh_geom_init_box(float hx, float hy, float hz,
                             float r, float g, float b, int visible) {
    triangle *tris = mesh_alloc_triangles(12);

    vec3 p000 = vec3_make(-hx, -hy, -hz);
    vec3 p001 = vec3_make(-hx, -hy,  hz);
    vec3 p010 = vec3_make(-hx,  hy, -hz);
    vec3 p011 = vec3_make(-hx,  hy,  hz);
    vec3 p100 = vec3_make( hx, -hy, -hz);
    vec3 p101 = vec3_make( hx, -hy,  hz);
    vec3 p110 = vec3_make( hx,  hy, -hz);
    vec3 p111 = vec3_make( hx,  hy,  hz);

    tris[0]  = (triangle){ p000, p011, p010};
    tris[1]  = (triangle){ p000, p001, p011 };

    tris[2]  = (triangle){ p100, p110, p111 };
    tris[3]  = (triangle){ p100, p111, p101 };

    tris[4]  = (triangle){ p000, p100, p101 };
    tris[5]  = (triangle){ p000, p101, p001 };

    tris[6]  = (triangle){ p010, p011, p111 };
    tris[7]  = (triangle){ p010, p111, p110 };

    tris[8]  = (triangle){ p000, p010, p110 };
    tris[9]  = (triangle){ p000, p110, p100 };

    tris[10] = (triangle){ p001, p101, p111 };
    tris[11] = (triangle){ p001, p111, p011 };

    mesh_geom mesh = mesh_geom_make(12, tris, r, g, b, visible);
    mesh.type = SHAPE_BOX;
    mesh.hx = hx;
    mesh.hy = hy;
    mesh.hz = hz;

    return mesh;
}

mesh_geom mesh_geom_init_plane(float hx, float hz,
                               float r, float g, float b, int visible) {
    triangle *tris = mesh_alloc_triangles(2);

    vec3 p0 = vec3_make(-hx, 0.0f, -hz);
    vec3 p1 = vec3_make( hx, 0.0f, -hz);
    vec3 p2 = vec3_make( hx, 0.0f,  hz);
    vec3 p3 = vec3_make(-hx, 0.0f,  hz);

    tris[0] = (triangle){ p0, p1, p2 };
    tris[1] = (triangle){ p0, p2, p3 };

    return mesh_geom_make(2, tris, r, g, b, visible);
}

mesh_geom mesh_geom_init_tetrahedron(float radius, float r, float g, float b, int visible) {
    triangle *tris = mesh_alloc_triangles(4);

    vec3 v0 = vec3_scale(vec3_normalize(vec3_make( 1.0f,  1.0f,  1.0f)), radius);
    vec3 v1 = vec3_scale(vec3_normalize(vec3_make(-1.0f, -1.0f,  1.0f)), radius);
    vec3 v2 = vec3_scale(vec3_normalize(vec3_make(-1.0f,  1.0f, -1.0f)), radius);
    vec3 v3 = vec3_scale(vec3_normalize(vec3_make( 1.0f, -1.0f, -1.0f)), radius);

    tris[0] = (triangle){ v0, v1, v2 };
    tris[1] = (triangle){ v0, v3, v1 };
    tris[2] = (triangle){ v0, v2, v3 };
    tris[3] = (triangle){ v1, v3, v2 };

    return mesh_geom_make(4, tris, r, g, b, visible);
}

mesh_geom mesh_geom_init_octahedron(float radius,
                                    float r, float g, float b, int visible) {
    triangle *tris = mesh_alloc_triangles(8);

    vec3 v0 = vec3_make( 0,  0,  radius);
    vec3 v1 = vec3_make( 0,  0, -radius);
    vec3 v2 = vec3_make( radius,  0, 0);
    vec3 v3 = vec3_make(-radius,  0, 0);
    vec3 v4 = vec3_make( 0,  radius, 0);
    vec3 v5 = vec3_make( 0, -radius, 0);

    // top
    tris[0] = (triangle){ v0, v2, v4 };
    tris[1] = (triangle){ v0, v4, v3 };
    tris[2] = (triangle){ v0, v3, v5 };
    tris[3] = (triangle){ v0, v5, v2 };

    // bottom
    tris[4] = (triangle){ v1, v4, v2 };
    tris[5] = (triangle){ v1, v3, v4 };
    tris[6] = (triangle){ v1, v5, v3 };
    tris[7] = (triangle){ v1, v2, v5 };

    return mesh_geom_make(8, tris, r, g, b, visible);
}

mesh_geom mesh_geom_init_dodecahedron(float radius,
                                      float r, float g, float b, int visible) {
    const float phi = (1.0f + sqrtf(5.0f)) * 0.5f;
    const float inv_phi = 1.0f / phi;

    vec3 v[20] = {
        vec3_make(-1, -1, -1), vec3_make(-1, -1,  1),
        vec3_make(-1,  1, -1), vec3_make(-1,  1,  1),
        vec3_make( 1, -1, -1), vec3_make( 1, -1,  1),
        vec3_make( 1,  1, -1), vec3_make( 1,  1,  1),

        vec3_make( 0, -inv_phi, -phi), vec3_make( 0, -inv_phi,  phi),
        vec3_make( 0,  inv_phi, -phi), vec3_make( 0,  inv_phi,  phi),

        vec3_make(-inv_phi, -phi, 0), vec3_make(-inv_phi,  phi, 0),
        vec3_make( inv_phi, -phi, 0), vec3_make( inv_phi,  phi, 0),

        vec3_make(-phi, 0, -inv_phi), vec3_make( phi, 0, -inv_phi),
        vec3_make(-phi, 0,  inv_phi), vec3_make( phi, 0,  inv_phi),
    };

    // norm to sphere
    for (int i = 0; i < 20; i++)
        v[i] = vec3_scale(vec3_normalize(v[i]), radius);

    // 12 pentagonal faces
    static const int faces[12][5] = {
        {0, 8,10, 2,16},
        {0,16,18, 1,12},
        {0,12,14, 4, 8},
        {8, 4,17, 6,10},
        {10,6,15,13, 2},
        {2,13,3,18,16},
        {1,9,11,3,13},
        {1,18,3,11,9},
        {4,14,5,19,17},
        {6,17,19,7,15},
        {3,11,7,19,18},
        {5,14,12,1,9}
    };

    triangle *tris = mesh_alloc_triangles(12 * 3); // 36 triangles
    int ti = 0;

    for (int i = 0; i < 12; i++) {
        int a = faces[i][0];
        for (int k = 1; k < 4; k++) {
            // fanning
            tris[ti++] = (triangle){
                v[a],
                v[faces[i][k + 1]],
                v[faces[i][k]]
            };
        }
    }

    mesh_geom mesh = mesh_geom_make(ti, tris, r, g, b, visible);

// #if GOURAUD_SHADING
//     mesh_geom_compute_smooth_normals(&mesh);
// #endif
    return mesh;
}

mesh_geom mesh_geom_init_icosahedron(float radius, float r, float g, float b, int visible) {
    const float phi = (1.0f + sqrtf(5.0f)) * 0.5f;

    vec3 verts[12] = {
        vec3_make(-1.0f,  phi,   0.0f),
        vec3_make( 1.0f,  phi,   0.0f),
        vec3_make(-1.0f, -phi,   0.0f),
        vec3_make( 1.0f, -phi,   0.0f),
        vec3_make( 0.0f, -1.0f,  phi),
        vec3_make( 0.0f,  1.0f,  phi),
        vec3_make( 0.0f, -1.0f, -phi),
        vec3_make( 0.0f,  1.0f, -phi),
        vec3_make( phi,   0.0f, -1.0f),
        vec3_make( phi,   0.0f,  1.0f),
        vec3_make(-phi,   0.0f, -1.0f),
        vec3_make(-phi,   0.0f,  1.0f),
    };

    for (int i = 0; i < 12; i++)
        verts[i] = vec3_scale(vec3_normalize(verts[i]), radius);

    static const int idx[20][3] = {
        {0,11,5}, {0,5,1}, {0,1,7}, {0,7,10}, {0,10,11},
        {1,5,9}, {5,11,4}, {11,10,2}, {10,7,6}, {7,1,8},
        {3,9,4}, {3,4,2}, {3,2,6}, {3,6,8}, {3,8,9},
        {4,9,5}, {2,4,11}, {6,2,10}, {8,6,7}, {9,8,1}
    };

    triangle *tris = mesh_alloc_triangles(20);
    for (int i = 0; i < 20; i++) {
        tris[i] = (triangle){
            verts[idx[i][0]],
            verts[idx[i][1]],
            verts[idx[i][2]]
        };
    }

    mesh_geom mesh = mesh_geom_make(20, tris, r, g, b, visible);
#if GOURAUD_SHADING
    mesh_geom_compute_smooth_normals(&mesh);
#endif
    return mesh;
}

static inline vec3 mesh_midpoint_on_sphere(vec3 a, vec3 b, float radius) {
    vec3 mid = vec3_scale(vec3_add(a, b), 0.5f);
    return vec3_scale(vec3_normalize(mid), radius);
}

mesh_geom mesh_geom_init_icosphere(float radius, int subdivisions, 
                                float r, float g, float b, int visible) {
    if (subdivisions < 0)
        subdivisions = 0;

    mesh_geom base = mesh_geom_init_icosahedron(radius, r, g, b, visible);
    triangle *curr = base.triangles;
    int curr_count = base.triangle_count;

    for (int s = 0; s < subdivisions; s++) {
        int next_count = curr_count * 4;
        triangle *next = mesh_alloc_triangles(next_count);

        for (int i = 0; i < curr_count; i++) {
            vec3 a = curr[i].v0;
            vec3 b = curr[i].v1;
            vec3 c = curr[i].v2;

            vec3 ab = mesh_midpoint_on_sphere(a, b, radius);
            vec3 bc = mesh_midpoint_on_sphere(b, c, radius);
            vec3 ca = mesh_midpoint_on_sphere(c, a, radius);

            next[4 * i + 0] = (triangle){ a,  ab, ca };
            next[4 * i + 1] = (triangle){ b,  bc, ab };
            next[4 * i + 2] = (triangle){ c,  ca, bc };
            next[4 * i + 3] = (triangle){ ab, bc, ca };
        }

        curr = next;
        curr_count = next_count;
    }

    mesh_geom mesh = mesh_geom_make(curr_count, curr, r, g, b, visible);
    mesh.type = SHAPE_SPHERE;
    mesh.radius = radius;
#if GOURAUD_SHADING
    mesh_geom_compute_smooth_normals(&mesh);
#endif
    return mesh;
}


static int obj_is_space(char c) {
    return c == ' ' || c == '\t' || c == '\r';
}

static int obj_is_digit(char c) {
    return c >= '0' && c <= '9';
}

static void obj_skip_spaces(const char **p) {
    while (**p && obj_is_space(**p))
        (*p)++;
}

static void obj_skip_to_eol(const char **p) {
    while (**p && **p != '\n')
        (*p)++;
    if (**p == '\n')
        (*p)++;
}

static int obj_parse_int(const char **p, int *out) {
    int sign = 1;
    int val = 0;
    int seen = 0;

    obj_skip_spaces(p);

    if (**p == '-') {
        sign = -1;
        (*p)++;
    } else if (**p == '+') {
        (*p)++;
    }

    while (obj_is_digit(**p)) {
        val = val * 10 + (**p - '0');
        (*p)++;
        seen = 1;
    }

    if (!seen)
        return 0;

    *out = sign * val;
    return 1;
}

static float obj_parse_float_raw(const char **p) {
    float sign = 1.0f;
    float val = 0.0f;
    float frac_scale = 0.1f;
    int exp_sign = 1;
    int exp_val = 0;

    if (**p == '-') {
        sign = -1.0f;
        (*p)++;
    } else if (**p == '+') {
        (*p)++;
    }

    while (obj_is_digit(**p)) {
        val = val * 10.0f + (float)(**p - '0');
        (*p)++;
    }

    if (**p == '.') {
        (*p)++;
        while (obj_is_digit(**p)) {
            val += (float)(**p - '0') * frac_scale;
            frac_scale *= 0.1f;
            (*p)++;
        }
    }

    if (**p == 'e' || **p == 'E') {
        (*p)++;
        if (**p == '-') {
            exp_sign = -1;
            (*p)++;
        } else if (**p == '+') {
            (*p)++;
        }

        while (obj_is_digit(**p)) {
            exp_val = exp_val * 10 + (**p - '0');
            (*p)++;
        }

        float pow10 = 1.0f;
        for (int i = 0; i < exp_val; i++)
            pow10 *= 10.0f;

        if (exp_sign > 0)
            val *= pow10;
        else
            val /= pow10;
    }

    return sign * val;
}

static int obj_parse_float(const char **p, float *out) {
    const char *start = *p;
    obj_skip_spaces(p);

    if (!(obj_is_digit(**p) || **p == '-' || **p == '+' || **p == '.')) {
        *p = start;
        return 0;
    }

    *out = obj_parse_float_raw(p);
    return 1;
}

/* Parse one OBJ face vertex token.
 * Examples:
 *   12
 *   12/7
 *   12/7/3
 *   12//3
 * We only care about the first integer (vertex index).
 */
static int obj_parse_face_index(const char **p, int *v_idx) {
    if (!obj_parse_int(p, v_idx))
        return 0;

    while (**p && !obj_is_space(**p) && **p != '\n') {
        (*p)++;
    }

    return 1;
}

static int obj_count_obj(const char *obj_text, int *vertex_count, int *triangle_count) {
    const char *p = obj_text;
    int vc = 0;
    int tc = 0;

    while (*p) {
        if (*p == '#') {
            obj_skip_to_eol(&p);
            continue;
        }

        if (*p == 'v' && obj_is_space(*(p + 1))) {
            vc++;
            obj_skip_to_eol(&p);
            continue;
        }

        if (*p == 'f' && obj_is_space(*(p + 1))) {
            int face_vertices = 0;
            const char *q = p + 1;   // skip 'f'

            while (*q && *q != '\n') {
                int idx;
                obj_skip_spaces(&q);

                if (!*q || *q == '\n')
                    break;

                if (!obj_parse_face_index(&q, &idx))
                    break;

                face_vertices++;
            }

            if (face_vertices >= 3)
                tc += (face_vertices - 2);

            obj_skip_to_eol(&p);
            continue;
        }

        obj_skip_to_eol(&p);
    }

    *vertex_count = vc;
    *triangle_count = tc;
    return 1;
}

mesh_geom mesh_geom_init_obj_from_memory(const char *obj_text,
                                        float r, float g, float b, int visible) {
    int vertex_count = 0;
    int triangle_count = 0;

    if (!obj_text) {
        printk("mesh_geom_init_obj_from_memory: null obj_text\n");
        return mesh_geom_make(0, 0, r, g, b, visible);
    }

    obj_count_obj(obj_text, &vertex_count, &triangle_count);

    if (vertex_count <= 0 || triangle_count <= 0) {
        printk("mesh_geom_init_obj_from_memory: no verts/tris found\n");
        return mesh_geom_make(0, 0, r, g, b, visible);
    }

    vec3 *verts = (vec3 *)kmalloc(sizeof(vec3) * vertex_count);
    triangle *tris = mesh_alloc_triangles(triangle_count);

    if (!verts || !tris) {
        printk("mesh_geom_init_obj_from_memory: allocation failed\n");
        return mesh_geom_make(0, 0, r, g, b, visible);
    }

    const char *p = obj_text;
    int vi = 0;
    int ti = 0;

    while (*p) {
        if (*p == '#') {
            obj_skip_to_eol(&p);
            continue;
        }

        if (*p == 'v' && obj_is_space(*(p + 1))) {
            float x, y, z;
            p++;  // skip 'v'

            if (!obj_parse_float(&p, &x) ||
                !obj_parse_float(&p, &y) ||
                !obj_parse_float(&p, &z)) {
                printk("OBJ parse error: malformed vertex\n");
                obj_skip_to_eol(&p);
                continue;
            }

            if (vi < vertex_count)
                verts[vi++] = vec3_make(x, y, z);

            obj_skip_to_eol(&p);
            continue;
        }

        if (*p == 'f' && obj_is_space(*(p + 1))) {
            int face_idx[32];
            int nf = 0;

            p++;  // skip 'f'

            while (*p && *p != '\n') {
                int idx;
                obj_skip_spaces(&p);

                if (!*p || *p == '\n')
                    break;

                if (!obj_parse_face_index(&p, &idx)) {
                    printk("OBJ parse error: malformed face token\n");
                    break;
                }

                if (nf < (int)(sizeof(face_idx) / sizeof(face_idx[0])))
                    face_idx[nf++] = idx;
                else {
                    printk("OBJ warning: face too large, truncating\n");
                    /* keep consuming rest of line but ignore extra vertices */
                }
            }

            if (nf >= 3) {
                /* validate all indices first */
                int ok = 1;
                for (int k = 0; k < nf; k++) {
                    int idx = face_idx[k];

                    /* OBJ is 1-based. Negative relative indices unsupported here. */
                    if (idx <= 0 || idx > vi) {
                        printk("OBJ parse error: face index out of range\n");
                        ok = 0;
                        break;
                    }
                }

                if (ok) {
                    /* fan triangulation:
                     * (0,1,2), (0,2,3), (0,3,4), ...
                     */
                    for (int k = 1; k < nf - 1; k++) {
                        if (ti < triangle_count) {
                            tris[ti++] = (triangle){
                                .v0 = verts[face_idx[0] - 1],
                                .v1 = verts[face_idx[k] - 1],
                                .v2 = verts[face_idx[k + 1] - 1]
                            };
                        }
                    }
                }
            }

            obj_skip_to_eol(&p);
            continue;
        }

        obj_skip_to_eol(&p);
    }

    mesh_geom mesh = mesh_geom_make(ti, tris, r, g, b, visible);

#if GOURAUD_SHADING
    mesh_geom_compute_smooth_normals(&mesh);
#endif

    return mesh;
}