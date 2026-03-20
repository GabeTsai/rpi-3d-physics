#include "rpi.h"
#include "sw-uart.h"
#include "keyboard.h"
#include "rpi-interrupts.h"

static sw_uart_t uart;

static void print_key(unsigned char k) {
    if (k >= 0x20 && k <= 0x7e) {
        sw_uart_put8(&uart, k);
    } else if (k == '\n') {
        sw_uart_put8(&uart, '\n');
    } else if (k == '\b') {
        sw_uart_putk(&uart, "<BS>");
    } else {
        sw_uart_putk(&uart, "<0x");
        sw_uart_printk(&uart, "%x", k);
        sw_uart_putk(&uart, ">");
    }
}
void notmain(void) {
    uart = sw_uart_default();

    sw_uart_putk(&uart, "keyboard test starting\n");
    sw_uart_putk(&uart, "type on PS/2 keyboard:\n");

    interrupt_init();
    keyboard_init(KEYBOARD_CLOCK, KEYBOARD_DATA);
    keyboard_use_interrupts();
    enable_interrupts();

    while (1) {
        unsigned char k = keyboard_read_next();
        print_key(k);
    }
}