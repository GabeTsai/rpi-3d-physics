#ifndef RENDER_H
#define RENDER_H

#include "rpi.h"
#include "rpi-math.h"
#include "physics.h"
#include "nv.h"

#define MAX_RIGID_BODIES 1000

typedef struct {
    int num_rbds;
    rigid_body *rbds;
} scene;

static inline void scene_init(scene *s, rigid_body *rbds_storage) {
    s->num_rbds = 0;
    s->rbds = rbds_storage;
}

void scene_clear(scene *s);
int scene_add_rigid_body(scene *s, const rigid_body *rb);
int render_scene(const scene *s,
                 const camera *cam,
                 float zs,
                 vec3 light_dir,
                 uint16_t *vert_index_list,
                 nv_vertex_nch_nps_t *shaded_vertex_data_addr);


bool proj_tri_to_screen(triangle tri, vec3 *world_points, 
                        float *proj_xs, float *proj_ys, float *proj_zs,
                        const camera *cam, quat orientation, vec3 position);

int render_rigid_body(const rigid_body *rb, const camera *cam,
                      vec3 light_dir,
                      float r, float g, float b,
                      int vert_offset,
                      uint16_t *vert_index_list,
                      nv_vertex_nch_nps_t *shaded_vertex_data_addr);
#endif