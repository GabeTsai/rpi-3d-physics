#include "mailbox-interface.h"

bool mbox_get_property(uint32_t* msg) {
    uint32_t addr = (uint32_t) msg;
    if ((addr & 0xF) == 0) { // check 16-aligned
        mbox_write(MB_TAGS_ARM_TO_VC, addr);
        while (mbox_read(MB_TAGS_ARM_TO_VC) != addr);
        return msg[1] == MBOX_REQUEST_SUCCESS;
    }
    return 0;
}
