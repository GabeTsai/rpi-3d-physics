PROGS := keyboard-test.c
COMMON_SRC += keyboard.c ps2_keys.c sw-uart.c
COMMON_SRC += handler.c

CFLAGS += -Wall -Werror -Wno-unused-function -Wno-unused-variable
CFLAGS += -I.
CFLAGS += -I$(CS140E_2026_FINAL_PROJ_PATH)/libpi/include
CFLAGS += -I$(CS140E_2026_FINAL_PROJ_PATH)/libpi/staff-src
CFLAGS += -I$(CS140E_2026_FINAL_PROJ_PATH)/libpi/src

RUN = 1
BOOTLOADER = my-install

include $(CS140E_2026_FINAL_PROJ_PATH)/libpi/mk/Makefile.robust-v2