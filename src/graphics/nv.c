#include "nv.h"
#include "float-helpers.h"

static int scene_vertex_count = 0;
// coords are center-relative screen coords - (0,0) is the center of the screen (viewport offset applied in cl_init_binning)
nv_vertex_nch_nps_t make_nv_vertex_nch_nps(float xs, float ys, float zs, float inv_wc, float r, float g, float b) {
    return (nv_vertex_nch_nps_t) {
        .xs_ys = pack_xs_ys_fixed12_4(float_to_fixed12_4(xs), float_to_fixed12_4(ys)),
        .zs = zs,
        .inv_wc = inv_wc,
        .varyings = { r, g, b }
    };
}

void put_triangle_to_nv(triangle tri, int idx_in_mesh, uint16_t *vert_index_list, nv_vertex_nch_nps_t *shaded_vertex_data_addr, float r, float g, float b) {
    nv_vertex_nch_nps_t vertex = make_nv_vertex_nch_nps(tri.v0.x, tri.v0.y, tri.v0.z, INV_WC_DEF, r, g, b);
    shaded_vertex_data_addr[idx_in_mesh] = vertex;
    vert_index_list[idx_in_mesh] = idx_in_mesh;
    vertex = make_nv_vertex_nch_nps(tri.v1.x, tri.v1.y, tri.v1.z, INV_WC_DEF, r, g, b);
    shaded_vertex_data_addr[idx_in_mesh + 1] = vertex;
    vert_index_list[idx_in_mesh + 1] = idx_in_mesh + 1;
    vertex = make_nv_vertex_nch_nps(tri.v2.x, tri.v2.y, tri.v2.z, INV_WC_DEF, r, g, b);
    shaded_vertex_data_addr[idx_in_mesh + 2] = vertex;
    vert_index_list[idx_in_mesh + 2] = idx_in_mesh + 2;
}

void put_mesh_geom_to_nv(mesh_geom mesh, uint16_t *vert_index_list, nv_vertex_nch_nps_t *shaded_vertex_data_addr) {
    for (int i = 0; i < mesh.triangle_count; i++) {
        put_triangle_to_nv(mesh.triangles[i], i * 3, vert_index_list, shaded_vertex_data_addr, mesh.r, mesh.g, mesh.b);
    }
}

void put_proj_tri_to_nv(float xs[3], float ys[3], float zs[3],
    vec3 c0, vec3 c1, vec3 c2,
    int base_idx,
    uint16_t *vert_index_list,
    nv_vertex_nch_nps_t *shaded_vertex_data_addr) {
    vec3 cs[3] = { c0, c1, c2 };
    for (int i = 0; i < 3; i++) {
        shaded_vertex_data_addr[base_idx + i] = make_nv_vertex_nch_nps(xs[i], ys[i], zs[i], INV_WC_DEF, cs[i].r, cs[i].g, cs[i].b);
        vert_index_list[base_idx + i] = base_idx + i;
    }
}
