# PROGS += test-float.c 
# PROGS += test-cl-simple-triangle-rotating.c
# PROGS += test-cl-simple-triangle.c
# PROGS += test-cl-simple-triangle-tile.c
# PROGS += test-cl-simple-triangle-rotate-light.c
# PROGS += test-cl-cube.c
# PROGS += test-cl-icosphere.c
# PROGS += test-cl-simple-triangle-rotating-cam.c
# untested above
# PROGS += test-cl-physics.c # works
PROGS += test-cl-balls.c # works

COMMON_SRC := ../gpu/mailbox.c ../gpu/mailbox-interface.c
COMMON_SRC += ../gpu/cl-interface.c
COMMON_SRC += ../gpu/cl.c
COMMON_SRC += ../gpu/v3d.c
COMMON_SRC += ../gpu/simple-frag-shader.c
COMMON_SRC += ../gpu/frag-shader-fixed-light.c
COMMON_SRC += ../matrix/matrix.c
COMMON_SRC += ../physics/physics.c
# COMMON_SRC += ../graphics/camera.h
COMMON_SRC += ../graphics/nv.c
COMMON_SRC += ../geom/geom.c
COMMON_SRC += ../graphics/render.c
COMMON_SRC += ../physics/collision.c
COMMON_SRC += ../physics/dynamics.c

CFLAGS += -I../gpu
CFLAGS += -I../matrix
CFLAGS += -I../physics
CFLAGS += -I../geom
CFLAGS += -I../graphics
CFLAGS += $(LIBM_INC)
LIBS += $(LIBM)

# uncomment if you want it to automatically run.
RUN=1

BOOTLOADER = my-install

L = $(CS140E_2026_FINAL_PROJ_PATH)/libpi/staff-objs
STAFF_OBJS += $(L)/kmalloc.o

# include $(CS140E_2026_FINAL_PROJ_PATH)/libpi/defs.mk
include $(CS140E_2026_FINAL_PROJ_PATH)/libpi/mk/Makefile.robust-v2 