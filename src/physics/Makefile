PROGS := test.c
COMMON_SRC := ../physics/physics.c ../physics/collision.c ../geom/geom.c
CFLAGS += -fno-omit-frame-pointer
CFLAGS += -fno-optimize-sibling-calls
CFLAGS += -fno-strict-aliasing
CFLAGS += -fstack-usage
CFLAGS += -Wstack-usage=1024
CFLAGS += -fno-inline
CFLAGS += -O0
CFLAGS += -I../gpu
CFLAGS += -I../geom
CFLAGS += -I../matrix
CFLAGS += -I../graphics
CFLAGS += $(LIBM_INC)
LIBS += $(LIBM)

# uncomment if you want it to automatically run.
RUN=1

BOOTLOADER = my-install

L = $(CS140E_2026_FINAL_PROJ_PATH)/libpi/staff-objs
# STAFF_OBJS += $(L)/staff-kmalloc.o

include $(CS140E_2026_FINAL_PROJ_PATH)/libpi/mk/Makefile.robust-v2