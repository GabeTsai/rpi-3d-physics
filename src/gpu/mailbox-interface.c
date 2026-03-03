#include "mailbox-interface.h"
#include "rpi.h"

bool mbox_get_property(uint32_t* msg) {
    uint32_t addr = (uint32_t) msg;
    if ((addr & 0xF) == 0) { // check 16-aligned
        mbox_write(MB_TAGS_ARM_TO_VC, addr);
        while (mbox_read(MB_TAGS_ARM_TO_VC) != addr);
        return msg[1] == MBOX_REQUEST_SUCCESS;
    }
    return 0;
}

uint32_t RPI_get_serialnum(void) {
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_BOARD_SERIAL,
        8, // 8 byte response
        MBOX_REQUEST,
        0,
        0,
        0
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get serial number\n");
    } 
    return msg[5];
}

uint32_t RPI_get_model(void) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_BOARD_MODEL,
        8, // 8 byte response
        MBOX_REQUEST,
        0,
        0,
        0
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get model number\n");
    } 
    return msg[5];
}

uint32_t RPI_get_memsize(void) {
   uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_ARM_MEMORY,
        8, // 8 byte response
        MBOX_REQUEST,
        0,
        0,
        0
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get memory size \n");
    } 
    // msg[5] is base addr, msg[6] is size
    return msg[6];
}

uint32_t RPI_get_revision(void) {
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_BOARD_REVISION,
        8, // 8 byte response
        MBOX_REQUEST,
        0,
        0,
        0
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get board revision \n");
    } 
    return msg[5];
}

uint32_t RPI_get_temp(void) {
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_TEMP,
        8, // 8 byte response
        MBOX_REQUEST,
        0,
        0,
        0
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get temperature \n");
    } 
    // msg[5] is temp id, msg[6] is value
    return msg[6];
}
