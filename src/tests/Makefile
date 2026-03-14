# PROGS += test-float.c 
PROGS += test-cl-simple-triangle-rotating.c

COMMON_SRC := ../gpu/mailbox.c ../gpu/mailbox-interface.c
COMMON_SRC += ../gpu/cl.c
COMMON_SRC += ../gpu/v3d.c
COMMON_SRC += ../gpu/simple-frag-shader.c
COMMON_SRC += ../matrix/matrix.c
COMMON_SRC += ../physics/physics.c
CFLAGS += -I../gpu
CFLAGS += -I../matrix
CFLAGS += -I../physics
CFLAGS += $(LIBM_INC)
LIBS += $(LIBM)

# uncomment if you want it to automatically run.
RUN=1

BOOTLOADER = my-install

L = $(CS140E_2026_FINAL_PROJ_PATH)/libpi/staff-objs
STAFF_OBJS += $(L)/kmalloc.o

# include $(CS140E_2026_FINAL_PROJ_PATH)/libpi/defs.mk
include $(CS140E_2026_FINAL_PROJ_PATH)/libpi/mk/Makefile.robust-v2 