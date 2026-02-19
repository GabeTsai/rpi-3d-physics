#ifndef MAILBOX_INTERFACE_H
#define MAILBOX_INTERFACE_H

#include "mailbox.h"
#include <stdbool.h>

enum {
    MBOX_REQUEST = 0,
    MBOX_REQUEST_SUCCESS = 0x80000000,
    MBOX_REQUEST_FAIL,
};

typedef enum MailboxClock_t {
    EMMC_CLK = 1,
    UART_CLK,
    ARM_CLK,
    CORE_CLK,
    V3D_CLK,
    H264_CLK,
    ISP_CLK,
    SDRAM_CLK,
    PIXEL_CLK,
    PWM_CLK,
    HEVC_CLK,
    EMMC2_CLK,
    M2MC_CLK,
    PIXEL_BVB_CLK,
    NUM_CLOCKS
} MailboxClock; 

typedef enum {
    /* Videocore info */
    MBOX_TAG_GET_FIRMWARE_REVISION      = 0x00000001,

    /* Hardware info */
    MBOX_TAG_GET_BOARD_MODEL            = 0x00010001,
    MBOX_TAG_GET_BOARD_REVISION         = 0x00010002,
    MBOX_TAG_GET_BOARD_MAC_ADDR         = 0x00010003,
    MBOX_TAG_GET_BOARD_SERIAL           = 0x00010004,
    MBOX_TAG_GET_ARM_MEMORY             = 0x00010005,
    MBOX_TAG_GET_VC_MEMORY              = 0x00010006,
    MBOX_TAG_GET_CLOCKS                 = 0x00010007,

    /* Config */
    MBOX_TAG_GET_COMMAND_LINE           = 0x00050001,

    /* Shared resource */
    MXBO_TAG_GET_DMA_CHANNELS           = 0x00060001,

    /* Power */
    MBOX_TAG_GET_POWER_STATE            = 0x00020001,
    MBOX_TAG_GET_TIMING                 = 0x00020002,
    MBOX_TAG_SET_POWER_STATE            = 0x00028001,

    /* Clocks */
    MBOX_TAG_GET_CLOCK_STATE            = 0x00030001,
    MBOX_TAG_SET_CLOCK_STATE            = 0x00038001,
    MBOX_TAG_GET_CLOCK_RATE             = 0x00030002,
    MBOX_TAG_GET_CLOCK_RATE_MEASURED    = 0x00030047,
    MBOX_TAG_SET_CLOCK_RATE             = 0x00038002,
    MBOX_TAG_GET_MAX_CLOCK_RATE         = 0x00030004,
    MBOX_TAG_GET_MIN_CLOCK_RATE         = 0x00030007,
    MBOX_TAG_GET_TURBO                  = 0x00030009,
    MBOX_TAG_SET_TURBO                  = 0x00038009,

    /* Onboard LED */
    MBOX_TAG_GET_ONBOARD_LED_STATUS     = 0x00030041,
    MBOX_TAG_TEST_ONBOARD_LED_STATUS    = 0x00034041,
    MBOX_TAG_SET_ONBOARD_LED_STATUS     = 0x00038041,

    /* Voltage */
    MBOX_TAG_GET_VOLTAGE                = 0x00030003,
    MBOX_TAG_SET_VOLTAGE                = 0x00038003,
    MBOX_TAG_GET_MAX_VOLTAGE            = 0x00030005,
    MBOX_TAG_GET_MIN_VOLTAGE            = 0x00030008,

    /* Temperature */
    MBOX_TAG_GET_TEMP                   = 0x00030006,
    MBOX_TAG_GET_MAX_TEMP               = 0x0003000A,

    /* Memory */
    MBOX_TAG_ALLOCATE_MEMORY            = 0x0003000C,
    MBOX_TAG_LOCK_MEMORY                = 0x0003000D,
    MBOX_TAG_UNLOCK_MEMORY              = 0x0003000E,
    MBOX_TAG_RELEASE_MEMORY             = 0x0003000F,

    /* Execute code */
    MBOX_TAG_EXECUTE_CODE               = 0x00030010,

    /* Displaymax */
    MBOX_TAG_GET_DISPMANX_HANDLE        = 0x00030014,
    MBOX_TAG_GET_EDID_BLOCK             = 0x00030020,

    /* Framebuffer */
    MBOX_TAG_ALLOCATE_FRAMEBUFFER       = 0x00040001,
    MBOX_TAG_RELEASE_FRAMEBUFFER        = 0x00048001,
    MBOX_TAG_BLANK_SCREEN               = 0x00040002,

    MBOX_TAG_GET_PHYSICAL_WIDTH_HEIGHT  = 0x00040003,
    MBOX_TAG_GET_VIRTUAL_WIDTH_HEIGHT   = 0x00040004,
    MBOX_TAG_GET_DEPTH                  = 0x00040005,
    MBOX_TAG_GET_PIXEL_ORDER            = 0x00040006,
    MBOX_TAG_GET_ALPHA_MODE             = 0x00040007,
    MBOX_TAG_GET_PITCH                  = 0x00040008,
    MBOX_TAG_GET_VIRTUAL_OFFSET         = 0x00040009,
    MBOX_TAG_GET_OVERSCAN               = 0x0004000A,
    MBOX_TAG_GET_PALETTE                = 0x0004000B,

    MBOX_TAG_TEST_PHYSICAL_WIDTH_HEIGHT = 0x00044003,
    MBOX_TAG_TEST_VIRTUAL_WIDTH_HEIGHT  = 0x00044004,
    MBOX_TAG_TEST_DEPTH                 = 0x00044005,
    MBOX_TAG_TEST_PIXEL_ORDER           = 0x00044006,
    MBOX_TAG_TEST_ALPHA_MODE            = 0x00044007,
    MBOX_TAG_TEST_VIRTUAL_OFFSET        = 0x00044009,
    MBOX_TAG_TEST_OVERSCAN              = 0x0004400A,
    MBOX_TAG_TEST_PALETTE               = 0x0004400B,

    MBOX_TAG_SET_PHYSICAL_WIDTH_HEIGHT  = 0x00048003,
    MBOX_TAG_SET_VIRTUAL_WIDTH_HEIGHT   = 0x00048004,
    MBOX_TAG_SET_DEPTH                  = 0x00048005,
    MBOX_TAG_SET_PIXEL_ORDER            = 0x00048006,
    MBOX_TAG_SET_ALPHA_MODE             = 0x00048007,
    MBOX_TAG_SET_VIRTUAL_OFFSET         = 0x00048009,
    MBOX_TAG_SET_OVERSCAN               = 0x0004800A,
    MBOX_TAG_SET_PALETTE                = 0x0004800B,

    /* Cursor */
    MBOX_TAG_SET_CURSOR_INFO            = 0x00008010,
    MBOX_TAG_SET_CURSOR_STATE           = 0x00008011,

    /* QPU */
    MBOX_TAG_EXECUTE_QPU                = 0x00030011,
    MBOX_TAG_SET_ENABLE_QPU             = 0x00030012,

    /* SD Card */
    /* VHCIQ */
} MboxTags;

bool mbox_get_property(uint32_t* msg);

#endif