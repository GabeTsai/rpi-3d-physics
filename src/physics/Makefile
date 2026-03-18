PROGS := physics.c
COMMON_SRC := ../gpu/mailbox.c ../gpu/mailbox-interface.c
CFLAGS += -I../gpu
CFLAGS += $(LIBM_INC)
LIBS += $(LIBM)

# uncomment if you want it to automatically run.
RUN=1

BOOTLOADER = my-install

L = $(CS140E_2026_FINAL_PROJ_PATH)/libpi/staff-objs
STAFF_OBJS += $(L)/staff-kmalloc.o

include $(CS140E_2026_FINAL_PROJ_PATH)/libpi/mk/Makefile.robust-v2