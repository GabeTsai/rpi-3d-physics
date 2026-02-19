# include "gpio-custom.h"
# include "rpi.h"
# include "rpi-interrupts.h"
# include <stdbool.h>

const char *gpio_panic_msg = "illegal pin=%d\n";

void GPIO_set_mode(PinT *pin, PinModeT mode) { 
    uint32_t pin_number = pin->pin_number;
    if (pin_number >= NUM_GPIO_PINS) { 
        gpio_panic(gpio_panic_msg, pin_number);
    }
    if((mode & 0b111) != mode){
        gpio_panic("illegal func=%x\n", mode);
    }
            
    uint32_t base_addr = GPIO_SELECT_BASE;
    uint32_t offset = (pin_number / 10) * REG_SIZE_BYTES;
    uint32_t shift = (pin_number % 10) * 3;
    uint32_t reg = GET32(base_addr + offset);
    reg &= ~(0x7 << shift); 
    reg |= (mode << shift); 
    PUT32(base_addr + offset, reg);
}

PinOutputTypeT GPIO_read(PinT *pin) { 
    uint32_t pin_number = pin->pin_number;
    if (pin_number >= 32) { 
        gpio_panic(gpio_panic_msg, pin_number);
    }
    uint32_t value;
    if (pin_number < REG_SIZE_BITS) { 
        value = (GET32(GPIO_LEV_BASE) >> pin_number) & 1;
    } else { 
        value = (GET32(GPIO_LEV_BASE + 4) >> (pin_number - REG_SIZE_BITS)) & 1;
    }
    return value;
}

void GPIO_set(PinT *pin, PinOutputTypeT value) { 
    if (value == HIGH) { 
        GPIO_set_high(pin);
    } else { 
        GPIO_set_low(pin);
    }
} 

void GPIO_set_high(PinT *pin) { 
    uint32_t pin_number = pin->pin_number;
    if (pin_number >= 32) { 
        gpio_panic(gpio_panic_msg, pin_number);
    }
    if (pin_number < REG_SIZE_BITS) { 
        PUT32(GPIO_SET_OUTPUT_BASE, 1 << pin_number);
    } else { 
        PUT32(GPIO_SET_OUTPUT_BASE + 4, 1 << (pin_number - REG_SIZE_BITS));
    }
}

void GPIO_set_low(PinT *pin) { 
    uint32_t pin_number = pin->pin_number;
    if (pin_number >= 32) { 
        gpio_panic(gpio_panic_msg, pin_number);
    }
    if (pin_number < REG_SIZE_BITS) { 
        PUT32(GPIO_CLR_OUTPUT_BASE, 1 << pin_number);
    } else { 
        PUT32(GPIO_CLR_OUTPUT_BASE + 4, 1 << (pin_number - REG_SIZE_BITS));
    }
}

void GPIO_set_output(PinT *pin) { 
    GPIO_set_mode(pin, OUTPUT);
}
void GPIO_set_input(PinT *pin) { 
    GPIO_set_mode(pin, INPUT);
}

void GPIO_enable_rising_edge_int(PinT *pin) { 
    uint32_t pin_number = pin->pin_number;
    if (pin_number >= 32) { 
        gpio_panic(gpio_panic_msg, pin_number);
    }
    uint32_t r = GPIO_RISING_ENABLE_0 + (pin_number / REG_SIZE_BITS) * REG_SIZE_BYTES;
    OR32(r, 1 << (pin_number % REG_SIZE_BITS));
}

void GPIO_enable_falling_edge_int(PinT *pin) { 
    uint32_t pin_number = pin->pin_number;
    if (pin_number >= 32) { 
        gpio_panic(gpio_panic_msg, pin_number);
    }
    uint32_t r = GPIO_FALLING_ENABLE_0 + (pin_number / REG_SIZE_BITS) * REG_SIZE_BYTES;
    OR32(r, 1 << (pin_number % REG_SIZE_BITS));
}

bool GPIO_is_int_detected(PinT *pin) { 
    uint32_t pin_number = pin->pin_number;
    if (pin_number >= 32) { 
        gpio_panic(gpio_panic_msg, pin_number);
    }
    uint32_t r = GPIO_EVENT_DETECT_0 + (pin_number / REG_SIZE_BITS) * REG_SIZE_BYTES;
    dev_barrier();
    uint32_t b = GET32(r);
    dev_barrier();
    return b & (1 << (pin_number % REG_SIZE_BITS));
}

void GPIO_clear_int(PinT *pin) { 
    uint32_t pin_number = pin->pin_number;
    if (pin_number >= 32) { 
        gpio_panic(gpio_panic_msg, pin_number);
    }
    uint32_t r = GPIO_EVENT_DETECT_0 + (pin_number / REG_SIZE_BITS) * REG_SIZE_BYTES;
    dev_barrier();
    PUT32(r, 1 << (pin_number % REG_SIZE_BITS));
    dev_barrier();
}

void GPIO_enable_general_int(PinT *pin) { 
    uint32_t pin_number = pin->pin_number;
    if (pin_number >= 32) { 
        gpio_panic(gpio_panic_msg, pin_number);
    }
    const uint32_t m = 1 << ((GPIO_IRQ_BASE + pin_number / REG_SIZE_BITS) % REG_SIZE_BITS);
    OR32(IRQ_Enable_2, m);
}

bool GPIO_is_int_enabled() { 
    const uint32_t m = 1 << (GPIO_IRQ_BASE);
    uint32_t enabled = GET32(IRQ_Enable_2) & m;
    return enabled != 0;
}
