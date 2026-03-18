#pragma once

#include "rpi.h"
#include "rpi-math.h"
#include "vec3.h"
#include "quat.h"

typedef struct {
    vec3 v0, v1, v2;
} triangle;

typedef struct {
    triangle *triangles;
    int triangle_count;
} mesh_geom;