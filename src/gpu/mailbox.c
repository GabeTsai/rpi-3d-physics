#include "mailbox.h"
#include "mailbox-interface.h"
#include "rpi.h"

void mbox_write(MboxChannel channel, uint32_t data) {
    while (GET32(MBOX_STATUS) & MBOX_FULL_BIT);
    PUT32(MBOX_WRITE, (data & ~0xF) | (channel & 0xF));
}

uint32_t mbox_read(MboxChannel channel) {
    uint32_t res;
    while (1) {
        // poll until there is a response
        while (GET32(MBOX_STATUS) & MBOX_EMPTY_BIT);

        dev_barrier();
        uint32_t res = GET32(MBOX_READ);
        dev_barrier();

        if ((res & 0xF) == channel) {
            return res & ~0xF;
        }
    }
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