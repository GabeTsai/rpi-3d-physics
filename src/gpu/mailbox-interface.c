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

static void check_response(uint32_t* msg) {
    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get property\n");
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

    check_response(msg);
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

    check_response(msg);
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

    check_response(msg);
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

    check_response(msg);
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

    check_response(msg);
    // msg[5] is temp id, msg[6] is value
    return msg[6];
}

uint32_t RPI_clock_get_curhz(MailboxClock clock) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_CLOCK_RATE,
        8, // 8 byte response
        MBOX_REQUEST,
        clock,
        0,
        0
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get clock rate \n");
    } 
    return msg[6];
}

uint32_t RPI_clock_get_realhz(MailboxClock clock) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_CLOCK_RATE_MEASURED,
        8, // 8 byte response
        MBOX_REQUEST,
        clock,
        0,
        0
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get clock rate \n");
    } 
    return msg[6];
}

uint32_t RPI_set_clock_hz(MailboxClock clock, uint32_t hz) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_SET_CLOCK_RATE,
        2 * sizeof(uint32_t), // clock_id, rate, skip_turbo
        MBOX_REQUEST,  // request code
        clock,
        hz,
        0,  
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to set clock rate \n");
    } 
    return msg[6];
}

uint32_t RPI_clock_get_maxhz(MailboxClock clock) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_MAX_CLOCK_RATE,
        8, // 8 byte response
        MBOX_REQUEST,
        clock,
        0,
        0
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get max clock rate \n");
    } 
    return msg[6];
}

uint32_t RPI_clock_get_minhz(MailboxClock clock) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_MIN_CLOCK_RATE,
        8, // 8 byte response
        MBOX_REQUEST,
        clock,
        0,
        0
    };

    dev_barrier();
    int res = mbox_get_property(msg);
    dev_barrier();

    if (!res) {
        panic("failed to get max clock rate \n");
    } 
    return msg[6];
}

/*
Allocate buffer

    Tag: 0x00040001
    Request:
        Length: 4
        Value:
            u32: alignment in bytes
    Response:
        Length: 8
        Value:
            u32: frame buffer base address in bytes
            u32: frame buffer size in bytes
*/
mbox_response_t RPI_fb_allocate(uint32_t alignment) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_ALLOCATE_FRAMEBUFFER,
        8, // 8-byte response
        MBOX_REQUEST,  // request code
        alignment, // alignment in bytes (overwritten with base addr on response)
        0, // frame buffer base address in bytes (filled on response)
        0, // frame buffer size in bytes (filled on response)
    };

    check_response(msg);
    mbox_response_t response = { msg[5], msg[6] };
    return response;
}

/*
Blank screen

    Tag: 0x00040002
    Request:
        Length: 4
        Value:
            u32: state
        State:
        Bit 0: 0=off, 1=on
        Bits 1-31: reserved for future use (set to 0)
    Response:
        Length: 4
        Value:
            u32: state
        State:
        Bit 0: 0=off, 1=on
        Bits 1-31: reserved for future use

*/

mbox_response_t RPI_fb_blank_screen(uint32_t state) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_BLANK_SCREEN,
        4, // 4-byte response
        MBOX_REQUEST,
        state,
        0
    };

    check_response(msg);
    mbox_response_t response = { msg[5]};
    return response;    
}

mbox_response_t RPI_fb_get_physical_width_height(void) {
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_PHYSICAL_WIDTH_HEIGHT,
        8, // 8-byte response
        MBOX_REQUEST,
        0, // width in pixels
        0, // height in pixels
    };
    check_response(msg);
    mbox_response_t response = { msg[5], msg[6] };
    return response;
}

mbox_response_t RPI_fb_set_physical_width_height(uint32_t width, uint32_t height) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_SET_PHYSICAL_WIDTH_HEIGHT,
        8, // 8-byte response
        MBOX_REQUEST,
        width,
        height
    };
    check_response(msg);
    mbox_response_t response = { msg[5], msg[6] };
    return response;
}

mbox_response_t RPI_fb_get_virtual_width_height(void) {
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_VIRTUAL_WIDTH_HEIGHT,
        8, // 8-byte response
        MBOX_REQUEST,
        0, // width in pixels
        0, // height in pixels
    };
    check_response(msg);
    mbox_response_t response = { msg[5], msg[6] };
    return response;
}

mbox_response_t RPI_fb_set_virtual_width_height(uint32_t width, uint32_t height) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_SET_VIRTUAL_WIDTH_HEIGHT,
        8, // 8-byte response
        MBOX_REQUEST,
        width,
        height  
    };
    check_response(msg);
    mbox_response_t response = { msg[5], msg[6] };
    return response;
}

mbox_response_t RPI_fb_get_depth(void) {
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_DEPTH,
        4, // 4-byte response
        MBOX_REQUEST,
        0, // depth
    };
    check_response(msg);
    mbox_response_t response = { msg[5] };
    return response;
}

mbox_response_t RPI_fb_set_depth(uint32_t depth) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_SET_DEPTH,
        4, // 4-byte response
        MBOX_REQUEST,
        depth
    };
    check_response(msg);
    mbox_response_t response = { msg[5] };
    return response;
}

mbox_response_t RPI_fb_get_pitch(void) {
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_GET_PITCH,
        4, // 4-byte response
        MBOX_REQUEST,
        0, // pitch
    };
    check_response(msg);
    mbox_response_t response = { msg[5] };
    return response;
}

/*
 * from https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface: 
 * "If an allocate buffer tag is omitted when setting parameters, 
 * then no change occurs unless it can be accommodated without changing the buffer base or size."
 * Thus, allocate tag should be sent along with other tags to actually change framebuffer size.
 */
fb_info_t RPI_fb_init(uint32_t p_width, uint32_t p_height, 
                    uint32_t v_width, uint32_t v_height, uint32_t depth) {

    uint32_t msg[FB_MSG_NWORDS] __attribute__((aligned(16))) = {
        FB_MSG_NWORDS * sizeof(uint32_t),
        MBOX_REQUEST,

        MBOX_TAG_SET_PHYSICAL_WIDTH_HEIGHT, // [2]
        8,                                  // [3] value buffer size
        MBOX_REQUEST,                       // [4]
        p_width,                              // [5]
        p_height,                             // [6]

        MBOX_TAG_SET_VIRTUAL_WIDTH_HEIGHT,  // [7]
        8,                                  // [8]
        MBOX_REQUEST,                       // [9]
        v_width,                              // [10]
        v_height,                             // [11]

        MBOX_TAG_SET_DEPTH,                 // [12]
        4,                                  // [13]
        MBOX_REQUEST,                       // [14]
        depth,                              // [15]

        MBOX_TAG_ALLOCATE_FRAMEBUFFER,      // [16]
        8,                                  // [17] 8 bytes: response is [base, size]
        MBOX_REQUEST,                       // [18]
        16,                                 // [19] alignment (in) -> base addr (out)
        0,                                  // [20] -> size (out)

        0,                                  // [21] end tag
    };

    check_response(msg);

    fb_info_t fb = {
        .base_addr = msg[19],
        .size      = msg[20],
        .p_width   = msg[5],
        .p_height  = msg[6],
        .v_width   = msg[10],
        .v_height  = msg[11],
        .depth     = msg[15],
    };

    return fb;
}

mbox_response_t RPI_qpu_enable(uint32_t enable) { 
    uint32_t msg[MSG_SIZE] __attribute__((aligned(16))) = {
        MSG_SIZE * sizeof(uint32_t),
        MBOX_REQUEST,
        MBOX_TAG_SET_ENABLE_QPU,
        4, // 4-byte response
        MBOX_REQUEST,
        enable
    };
    check_response(msg);
    mbox_response_t response = { msg[5] };
    return response; 
}