#include "gpio.h"
#include "gpio_extra.h"
#include "uart.h"

const int CLK = GPIO_PIN3;
const int DATA = GPIO_PIN4;

static void wait_for_falling_clock_edge(void) {
    while (gpio_read(CLK) == 0) { }
    while (gpio_read(CLK) == 1) { }
}

static int ps2_read_bit(void) {
    wait_for_falling_clock_edge();
    return gpio_read(DATA);
}

static unsigned char ps2_read_scancode(void) {
    int start, parity, stop;
    unsigned char code = 0;

    start = ps2_read_bit();
    if (start != 0) {
        return 0xFF;   // invalid frame
    }

    for (int i = 0; i < 8; i++) {
        int bit = ps2_read_bit();
        code |= (bit << i);   // LSB first
    }

    parity = ps2_read_bit();
    stop   = ps2_read_bit();

    if (stop != 1) {
        return 0xFF;   // invalid frame
    }

    // optional: verify odd parity
    int ones = 0;
    for (int i = 0; i < 8; i++)
        ones += (code >> i) & 1;
    ones += parity;

    if ((ones & 1) == 0) {
        return 0xFF;   // parity error
    }

    return code;
}

void main(void) {
    gpio_init();
    uart_init();

    gpio_set_input(CLK);
    gpio_set_pullup(CLK);

    gpio_set_input(DATA);
    gpio_set_pullup(DATA);

    while (1) {
        unsigned char sc = ps2_read_scancode();
        uart_puthex(sc);
        uart_putchar('\n');
    }
}