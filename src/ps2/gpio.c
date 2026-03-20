#include "gpio.h"
#include "rpi.h"

/*
 * BCM2835 / Raspberry Pi GPIO
 *
 * Function select:
 *   GPFSEL0..5   base + 0x00
 *
 * Set / clear:
 *   GPSET0..1    base + 0x1c
 *   GPCLR0..1    base + 0x28
 *
 * Level:
 *   GPLEV0..1    base + 0x34
 *
 * Pull-up/down:
 *   GPPUD        base + 0x94
 *   GPPUDCLK0..1 base + 0x98
 *
 * Event detect:
 *   GPREN0..1    base + 0x4c   rising edge
 *   GPFEN0..1    base + 0x58   falling edge
 *   GPHEN0..1    base + 0x64   high detect
 *   GPLEN0..1    base + 0x70   low detect
 *   GPAREN0..1   base + 0x7c   async rising
 *   GPAFEN0..1   base + 0x88   async falling
 *   GPEDS0..1    base + 0x40   event detect status
 *
 * IRQ basic for GPIO_INT0..3:
 *   system peripheral interrupts 49..52
 */

enum {
    GPIO_BASE_ADDR = 0x20200000,

    GPFSEL0   = GPIO_BASE_ADDR + 0x00,
    GPSET0    = GPIO_BASE_ADDR + 0x1c,
    GPCLR0    = GPIO_BASE_ADDR + 0x28,
    GPLEV0    = GPIO_BASE_ADDR + 0x34,
    GPEDS0    = GPIO_BASE_ADDR + 0x40,
    GPREN0    = GPIO_BASE_ADDR + 0x4c,
    GPFEN0    = GPIO_BASE_ADDR + 0x58,
    GPHEN0    = GPIO_BASE_ADDR + 0x64,
    GPLEN0    = GPIO_BASE_ADDR + 0x70,
    GPAREN0   = GPIO_BASE_ADDR + 0x7c,
    GPAFEN0   = GPIO_BASE_ADDR + 0x88,
    GPPUD     = GPIO_BASE_ADDR + 0x94,
    GPPUDCLK0 = GPIO_BASE_ADDR + 0x98,

    IRQ_ENABLE2 = 0x2000b214,
};

static void gpio_check_pin(unsigned pin) {
    if (pin >= 54)
        panic("illegal pin=%d\n", pin);
}

static inline unsigned reg_for_pin(unsigned base, unsigned pin) {
    return base + 4 * (pin / 32);
}

static inline unsigned bit_for_pin(unsigned pin) {
    return 1u << (pin % 32);
}

static inline void short_delay(void) {
    for (volatile int i = 0; i < 150; i++) { }
}

static void gpio_enable_irq_line_for_pin(unsigned pin) {
    /*
     * GPIO interrupts 0..3 correspond to IRQs 49..52.
     * Pins 0..31 are in bank 0 => GPIO_INT0
     * Pins 32..53 are in bank 1 => GPIO_INT1
     *
     * (GPIO_INT2/3 are not normally needed for external GPIO pins here.)
     */
    unsigned irq = (pin < 32) ? GPIO_INT0 : GPIO_INT1;
    dev_barrier();
    PUT32(IRQ_ENABLE2, 1u << (irq - 32));
    dev_barrier();
}

void gpio_set_function(unsigned pin, gpio_func_t function) {
    gpio_check_pin(pin);

    unsigned reg = GPFSEL0 + 4 * (pin / 10);
    unsigned shift = (pin % 10) * 3;

    unsigned v = GET32(reg);
    v &= ~(7u << shift);
    v |= ((unsigned)function << shift);
    PUT32(reg, v);
}

void gpio_set_input(unsigned pin) {
    gpio_set_function(pin, GPIO_FUNC_INPUT);
}

void gpio_set_output(unsigned pin) {
    gpio_set_function(pin, GPIO_FUNC_OUTPUT);
}

void gpio_write(unsigned pin, unsigned val) {
    gpio_check_pin(pin);

    unsigned reg = val ? reg_for_pin(GPSET0, pin) : reg_for_pin(GPCLR0, pin);
    PUT32(reg, bit_for_pin(pin));
}

int gpio_read(unsigned pin) {
    gpio_check_pin(pin);

    unsigned reg = reg_for_pin(GPLEV0, pin);
    return (GET32(reg) & bit_for_pin(pin)) != 0;
}

void gpio_set_on(unsigned pin) {
    gpio_write(pin, 1);
}

void gpio_set_off(unsigned pin) {
    gpio_write(pin, 0);
}

void gpio_set_pullup(unsigned pin) {
    gpio_check_pin(pin);

    PUT32(GPPUD, 0b10);
    short_delay();
    PUT32(reg_for_pin(GPPUDCLK0, pin), bit_for_pin(pin));
    short_delay();
    PUT32(GPPUD, 0);
    PUT32(reg_for_pin(GPPUDCLK0, pin), 0);
}

void gpio_set_pulldown(unsigned pin) {
    gpio_check_pin(pin);

    PUT32(GPPUD, 0b01);
    short_delay();
    PUT32(reg_for_pin(GPPUDCLK0, pin), bit_for_pin(pin));
    short_delay();
    PUT32(GPPUD, 0);
    PUT32(reg_for_pin(GPPUDCLK0, pin), 0);
}

void gpio_pud_off(unsigned pin) {
    gpio_check_pin(pin);

    PUT32(GPPUD, 0b00);
    short_delay();
    PUT32(reg_for_pin(GPPUDCLK0, pin), bit_for_pin(pin));
    short_delay();
    PUT32(GPPUD, 0);
    PUT32(reg_for_pin(GPPUDCLK0, pin), 0);
}

int gpio_get_pud(unsigned pin) {
    /*
     * BCM2835 doesn't expose readable per-pin PUD state in a useful way here.
     * If your staff code expects this, either track in software or return -1.
     */
    gpio_check_pin(pin);
    return -1;
}

int gpio_has_interrupt(void) {
    /*
     * Header says this should report if there is currently a GPIO_INT0 interrupt.
     * IRQ 49 is in IRQ pending 2 bit (49-32)=17.
     */
    unsigned pending2 = GET32(0x2000b208);
    return (pending2 & (1u << (GPIO_INT0 - 32))) != 0;
}

void gpio_int_rising_edge(unsigned pin) {
    gpio_check_pin(pin);
    dev_barrier();
    OR32(reg_for_pin(GPREN0, pin), bit_for_pin(pin));
    dev_barrier();
    gpio_enable_irq_line_for_pin(pin);
}

void gpio_int_falling_edge(unsigned pin) {
    gpio_check_pin(pin);
    dev_barrier();
    OR32(reg_for_pin(GPFEN0, pin), bit_for_pin(pin));
    dev_barrier();
    gpio_enable_irq_line_for_pin(pin);
}

void gpio_enable_hi_int(unsigned pin) {
    gpio_check_pin(pin);
    dev_barrier();
    OR32(reg_for_pin(GPHEN0, pin), bit_for_pin(pin));
    dev_barrier();
    gpio_enable_irq_line_for_pin(pin);
}

void gpio_int_low(unsigned pin) {
    gpio_check_pin(pin);
    dev_barrier();
    OR32(reg_for_pin(GPLEN0, pin), bit_for_pin(pin));
    dev_barrier();
    gpio_enable_irq_line_for_pin(pin);
}

int gpio_event_detected(unsigned pin) {
    gpio_check_pin(pin);
    dev_barrier();
    unsigned v = GET32(reg_for_pin(GPEDS0, pin));
    dev_barrier();
    return (v & bit_for_pin(pin)) != 0;
}

void gpio_event_clear(unsigned pin) {
    gpio_check_pin(pin);
    dev_barrier();
    PUT32(reg_for_pin(GPEDS0, pin), bit_for_pin(pin));
    dev_barrier();
}

void gpio_int_async_falling_edge(unsigned pin) {
    gpio_check_pin(pin);
    dev_barrier();
    OR32(reg_for_pin(GPAFEN0, pin), bit_for_pin(pin));
    dev_barrier();
    gpio_enable_irq_line_for_pin(pin);
}

void gpio_int_async_rising_edge(unsigned pin) {
    gpio_check_pin(pin);
    dev_barrier();
    OR32(reg_for_pin(GPAREN0, pin), bit_for_pin(pin));
    dev_barrier();
    gpio_enable_irq_line_for_pin(pin);
}

void gpio_fiq_async_falling_edge(unsigned pin) {
    gpio_int_async_falling_edge(pin);
}

void gpio_fiq_async_rising_edge(unsigned pin) {
    gpio_int_async_rising_edge(pin);
}

void gpio_fiq_falling_edge(unsigned pin) {
    gpio_int_falling_edge(pin);
}

void gpio_fiq_rising_edge(unsigned pin) {
    gpio_int_rising_edge(pin);
}