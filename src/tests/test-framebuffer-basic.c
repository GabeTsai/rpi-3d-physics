#include "rpi.h"   
#include "mailbox-interface.h"

void notmain(void) {
    fb_info_t fb = RPI_fb_init(256, 256, 512, 512,32);
    output("base addr:  %x\n", fb.base_addr);
    output("size:       %d bytes\n", fb.size);
    output("physical width:      %d\n", fb.p_width);
    output("physical height:     %d\n", fb.p_height);
    output("virtual width:      %d\n", fb.v_width);
    output("virtual height:     %d\n", fb.v_height);
    output("depth:      %d\n", fb.depth);

    uint32_t pitch = RPI_fb_get_pitch().val1;
    output("pitch:      %d\n", pitch);
    uint32_t *pix = (uint32_t*)fb.base_addr;
    uint32_t stride = pitch / BYTES_PER_PIXEL;

    for (uint32_t y = 0; y < fb.p_height; y++) {
        for (uint32_t x = 0; x < fb.p_width; x++) {
            pix[y * stride + x] = 0x00FFFFFF;
        }
    }

    delay_ms(5000);
    RPI_fb_blank_screen(1);

    output("done");
    // RPI_fb_blank_screen(0);
    // delay_ms(2000);
}