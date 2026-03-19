#include "rpi.h"
#include "physics.h"
#include "dynamics.h"
#include "collision.h"

static void print_vec3(const char *name, vec3 v) {
    printk("%s = (%f, %f, %f)\n", name, v.x, v.y, v.z);
}

void notmain(void) {
    kmalloc_init(10);
    printk("physics box collision test\n");

    rigid_body_geom g1, g2;
    rigid_body b1, b2;

    int ok = 0;

    // Box 1: centered at (0,0,0), size 2x2x2
    // ok = phys_geom_init_box(&g1, 1.0f, 1.0f, 1.0f, 1.0f);
    mesh_geom m1 = mesh_geom_init_box(1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f);
    phys_geom_init(&g1, m1, 1.0f);
    if (ok != 0) {
        printk("phys_geom_init_box g1 failed\n");
        clean_reboot();
    }

    // Box 2: also size 2x2x2
    // ok = phys_geom_init_box(&g2, 1.0f, 1.0f, 1.0f, 1.0f);
    mesh_geom m2 = mesh_geom_init_box(1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f);
    phys_geom_init(&g2, m2, 1.0f);
    if (ok != 0) {
        printk("phys_geom_init_box g2 failed\n");
        clean_reboot();
    }

    quat q_identity = {1.0f, 0.0f, 0.0f, 0.0f};

    vec3 p1 = {.x = 0.0f, .y = 0.0f, .z = 0.0f};
    vec3 p2 = {.x = 0.8f, .y = 0.0f, .z = 0.0f};   // overlapping along x since each box extends 1.0

    ok = phys_body_init(&b1, &g1, p1, quat_from_euler(0.01f, 0.0f, 0.0f));
    if (ok != 0) {
        printk("phys_body_init b1 failed\n");
        clean_reboot();
    }

    ok = phys_body_init(&b2, &g2, p2, q_identity);
    if (ok != 0) {
        printk("phys_body_init b2 failed\n");
        clean_reboot();
    }

    collision_result c = phys_collide_convex(&b1, &b2);

    printk("collision hit = %d\n", c.hit);

    if (c.hit) {
        printk("epa hit = %d\n", c.epa.hit);
        printk("penetration depth = %f\n", c.epa.depth);
        print_vec3("collision normal", c.epa.normal);
    } else {
        printk("no collision detected\n");
    }

    clean_reboot();
}