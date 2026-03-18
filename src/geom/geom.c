// geom.c
#include "geom.h"

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
        float r, float g, float b) {
    mesh_geom mesh;
    mesh.id = g_next_mesh_geom_id++;
    mesh.triangle_count = triangle_count;
    mesh.triangles = triangles;

    mesh.r = r;
    mesh.g = g;
    mesh.b = b;
    return mesh;
}

mesh_geom mesh_geom_init_custom_take(int triangle_count, triangle *triangles, float r, float g, float b) {
    return mesh_geom_make(triangle_count, triangles, r, g, b);
}

mesh_geom mesh_geom_init_custom_copy(int triangle_count, const triangle *triangles, float r, float g, float b) {
    triangle *dst = mesh_alloc_triangles(triangle_count);
    for (int i = 0; i < triangle_count; i++)
        dst[i] = triangles[i];

    return mesh_geom_make(triangle_count, dst, r, g, b);
}

mesh_geom mesh_geom_init_triangle(triangle tri, float r, float g, float b) {
    triangle *tris = mesh_alloc_triangles(1);
    tris[0] = tri;
    return mesh_geom_make(1, tris, r, g, b);
}

mesh_geom mesh_geom_init_box(float hx, float hy, float hz,
                             float r, float g, float b) {
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

    return mesh_geom_make(12, tris, r, g, b);
}

mesh_geom mesh_geom_init_plane(float hx, float hz,
                               float r, float g, float b) {
    triangle *tris = mesh_alloc_triangles(2);

    vec3 p0 = vec3_make(-hx, 0.0f, -hz);
    vec3 p1 = vec3_make( hx, 0.0f, -hz);
    vec3 p2 = vec3_make( hx, 0.0f,  hz);
    vec3 p3 = vec3_make(-hx, 0.0f,  hz);

    tris[0] = (triangle){ p0, p1, p2 };
    tris[1] = (triangle){ p0, p2, p3 };

    return mesh_geom_make(2, tris, r, g, b);
}

mesh_geom mesh_geom_init_tetrahedron(float radius, float r, float g, float b) {
    triangle *tris = mesh_alloc_triangles(4);

    vec3 v0 = vec3_scale(vec3_normalize(vec3_make( 1.0f,  1.0f,  1.0f)), radius);
    vec3 v1 = vec3_scale(vec3_normalize(vec3_make(-1.0f, -1.0f,  1.0f)), radius);
    vec3 v2 = vec3_scale(vec3_normalize(vec3_make(-1.0f,  1.0f, -1.0f)), radius);
    vec3 v3 = vec3_scale(vec3_normalize(vec3_make( 1.0f, -1.0f, -1.0f)), radius);

    tris[0] = (triangle){ v0, v1, v2 };
    tris[1] = (triangle){ v0, v3, v1 };
    tris[2] = (triangle){ v0, v2, v3 };
    tris[3] = (triangle){ v1, v3, v2 };

    return mesh_geom_make(4, tris, r, g, b);
}

mesh_geom mesh_geom_init_icosahedron(float radius, float r, float g, float b) {
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

    return mesh_geom_make(20, tris, r, g, b);
}

static inline vec3 mesh_midpoint_on_sphere(vec3 a, vec3 b, float radius) {
    vec3 mid = vec3_scale(vec3_add(a, b), 0.5f);
    return vec3_scale(vec3_normalize(mid), radius);
}

mesh_geom mesh_geom_init_icosphere(float radius, int subdivisions, 
                                float r, float g, float b) {
    if (subdivisions < 0)
        subdivisions = 0;

    mesh_geom base = mesh_geom_init_icosahedron(radius, r, g, b);
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

    return mesh_geom_make(curr_count, curr, r, g, b);
}
