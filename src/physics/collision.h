#pragma once

#include "physics.h"

typedef struct {
    vec3 min;
    vec3 max;
} aabb3;

typedef struct {
    vec3 p;      // Minkowski support point = supp_a - supp_b
    vec3 supp_a; // world-space support on A
    vec3 supp_b; // world-space support on B
} support_point;

typedef struct {
    support_point pts[4];
    int count;
} simplex3;

typedef struct {
    int hit;              // 1 if collision
    simplex3 simplex;     // final simplex for EPA warm-start
    vec3 search_dir;      // last search dir if no hit
} gjk_result;

typedef struct {
    int hit;              // 1 if penetration result valid
    vec3 normal;          // world-space collision normal, from A toward B
    float depth;          // penetration depth
    vec3 contact_a;       // world-space contact point on A
    vec3 contact_b;       // world-space contact point on B
    vec3 contact; 
} epa_result;

typedef struct {
    int hit;              // broad-phase + narrow-phase hit
    gjk_result gjk;
    epa_result epa;
} collision_result;

aabb3 phys_body_compute_aabb(const rigid_body *b);
int phys_aabb_overlap(aabb3 a, aabb3 b);

gjk_result phys_gjk_intersect(const rigid_body *a, const rigid_body *b);
epa_result phys_epa_from_gjk(const rigid_body *a, const rigid_body *b,
                             const simplex3 *gjk_simplex);

collision_result phys_collide_convex(const rigid_body *a, const rigid_body *b);