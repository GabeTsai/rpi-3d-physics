#ifndef RENDER_H
#define RENDER_H

#include "rpi.h"
#include "rpi-math.h"
#include "nv.h"

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