#pragma once

// #include "../physics/physics.h"
#include "physics.h"
#include "vec3.h"
#include "quat.h"
#include "rpi.h"
typedef struct {
    vec3 position;
    quat rotation;

    float fx;
    float fy;

    float cx;
    float cy;
} camera;

/* initialize camera */
static inline void camera_init(camera *c, vec3 position, quat rotation, float fx, float fy, float cx, float cy) {
    c->position = position;
    c->rotation = rotation;

    c->fx = fx;
    c->fy = fy;
    c->cx = cx;
    c->cy = cy;
}

static inline vec3 camera_world_to_camera(const camera *cam, vec3 p_world) {
    vec3 rel = vec3_sub(p_world, cam->position);
    quat inv = quat_conjugate(cam->rotation);
    return quat_rotate_vec3(inv, rel);
}

static inline int camera_project_point(const camera *cam,
                                       vec3 world_point,
                                       float *px, float *py)
{
    vec3 rel = vec3_sub(world_point, cam->position);
    vec3 p_cam = quat_rotate_vec3(quat_conjugate(cam->rotation), rel);

    if(p_cam.z <= 0.0f)
        return 0;
    // printk("%f, %f, %f\n", world_point.x, world_point.y, world_point.z);
    // printk("%f, %f, %f\n", p_cam.x, p_cam.y, p_cam.z);
    // printk("%f, %f, %f, %f\n", cam->fx, cam->fy, cam->cx, cam->cy);

    *px = cam->fx * (p_cam.x / p_cam.z) + cam->cx;
    *py = cam->cy - cam->fy * (p_cam.y / p_cam.z);
    // printk("%f, %f\n", *px, *py);

    return 1;
}

typedef struct {
    vec3 p_cam;
    float xc, yc, zc, wc;
    float xs, ys, zs;
    float inv_wc;
} camera_clip_proj;

static inline int camera_project_clip(const camera *cam,
                                      vec3 world_point,
                                      float znear,
                                      float zfar,
                                      camera_clip_proj *out)
{
    vec3 rel = vec3_sub(world_point, cam->position);
    vec3 p_cam = quat_rotate_vec3(quat_conjugate(cam->rotation), rel);

    float x = p_cam.x;
    float y = p_cam.y;
    float z = p_cam.z;

    if (z <= znear || z >= zfar)
        return 0;

    out->p_cam = p_cam;

    // true projective header
    out->wc = z;
    out->xc = x;
    out->yc = -y;

    // choose a depth term that gives a sane post-divide zs in [0,1]
    out->zc = (z - znear) / (zfar - znear) * z;

    // raster coords in pixels relative to viewport centre
    out->xs = cam->fx * (out->xc / out->wc);
    out->ys = cam->fy * (out->yc / out->wc);
    out->zs = out->zc / out->wc;
    out->inv_wc = 1.0f / out->wc;

    return 1;
}

static inline void camera_translate(camera *c, vec3 delta) {
    c->position = vec3_add(c->position, delta);
}

static inline void camera_rotate(camera *c, quat q) {
    c->rotation = quat_mul(q, c->rotation);
}

