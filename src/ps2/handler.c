#include "rpi.h"
#include "keyboard.h"
#include "gpio.h"

void interrupt_vector(unsigned pc) {
    (void)pc;
        // printk("IRQ!\n");

    if (gpio_has_interrupt()) {
        // printk("IRQ!\n");
        keyboard_check_and_handle_interrupt();
        return;
    }

    panic("unexpected interrupt\n");
}