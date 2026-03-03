#include "mailbox.h"
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
