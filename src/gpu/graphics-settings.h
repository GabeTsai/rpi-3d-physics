#ifndef GRAPHICS_SETTINGS_H
#define GRAPHICS_SETTINGS_H

typedef enum { 
    DEPTH_TEST_NEVER = 0,
    DEPTH_TEST_LT,
    DEPTH_TEST_EQ, 
    DEPTH_TEST_LEQ,
    DEPTH_TEST_GT,
    DEPTH_TEST_NE,
    DEPTH_TEST_GE,
    DEPTH_TEST_ALWAYS

} DepthTestFunc;

#include <stdint.h>
#include <stdbool.h>
#include "rpi.h"

#define AMBIENT_INTENSITY_DEF 0.5f
#define DEFAULT_FB_CLEAR_COLOR 0xFF18191AFF18191AULL

#endif