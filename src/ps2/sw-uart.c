#include "rpi.h"
#include "sw-uart.h"
#include "gpio.h"
#include "cycle-count.h"
#include "cycle-util.h"

#include <stdarg.h>

static inline void wait_until_cycle(uint32_t target) {
    while ((int32_t)(cycle_cnt_read() - target) < 0)
        ;
}

void sw_uart_put8(sw_uart_t *uart, uint8_t b) {
    unsigned tx = uart->tx;
    uint32_t n = uart->cycle_per_bit;
    uint32_t t = cycle_cnt_read();

    gpio_write(tx, 0);          // start bit
    t += n;
    wait_until_cycle(t);

    for (unsigned i = 0; i < 8; i++) {
        gpio_write(tx, (b >> i) & 1);
        t += n;
        wait_until_cycle(t);
    }

    gpio_write(tx, 1);          // stop bit
    t += n;
    wait_until_cycle(t);
}

int sw_uart_get8_timeout(sw_uart_t *uart, uint32_t timeout_usec) {
    unsigned rx = uart->rx;

    if (!wait_until_usec(rx, 0, timeout_usec))
        return -1;

    uint32_t n = uart->cycle_per_bit;
    uint32_t t = cycle_cnt_read();

    // middle of first data bit
    t += n + n / 2;
    wait_until_cycle(t);

    uint8_t b = 0;
    for (unsigned i = 0; i < 8; i++) {
        if (gpio_read(rx))
            b |= 1u << i;

        t += n;
        wait_until_cycle(t);
    }

    // stop bit should be high
    if (!gpio_read(rx))
        return -1;

    return b;
}

sw_uart_t sw_uart_init_helper(unsigned tx, unsigned rx,
        unsigned baud,
        unsigned cyc_per_bit,
        unsigned usec_per_bit)
{
    assert(tx && tx < 31);
    assert(rx && rx < 31);
    assert(cyc_per_bit && cyc_per_bit > usec_per_bit);
    assert(usec_per_bit);

    unsigned mhz = 700 * 1000 * 1000;
    unsigned derived = cyc_per_bit * baud;
    if (!((mhz - baud) <= derived && derived <= (mhz + baud)))
        panic("too much diff: cyc_per_bit = %d * baud = %d\n",
            cyc_per_bit, cyc_per_bit * baud);

    gpio_set_output(tx);
    gpio_write(tx, 1);      // idle high

    gpio_set_input(rx);

    return (sw_uart_t) {
        .tx = tx,
        .rx = rx,
        .baud = baud,
        .cycle_per_bit = cyc_per_bit,
        .usec_per_bit = usec_per_bit
    };
}