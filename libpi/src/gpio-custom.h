#pragma once
#include <stdint.h>
#include <stdbool.h>

#define REG_SIZE_BYTES 4
#define REG_SIZE_BITS 32
#define NUM_GPIO_PINS 54

#define GPIO_IRQ_BASE 49 - 32 // Offset for GPIO interrupts in IRQ_ENABLE_2
typedef enum GPIOBaseAddrT { 
    GPIO_SELECT_BASE = 0x20200000,
    GPIO_SET_OUTPUT_BASE = GPIO_SELECT_BASE + 0x1C,
    GPIO_CLR_OUTPUT_BASE = GPIO_SELECT_BASE + 0x28,
    GPIO_LEV_BASE = GPIO_SELECT_BASE + 0x34,
    GPIO_EVENT_DETECT_0 = GPIO_SELECT_BASE + 0x40,
    GPIO_EVENT_DETECT_1 = GPIO_SELECT_BASE + 0x44,
    GPIO_RISING_ENABLE_0 = GPIO_SELECT_BASE + 0x4C,
    GPIO_RISING_ENABLE_1 = GPIO_SELECT_BASE + 0x50,
    GPIO_FALLING_ENABLE_0 = GPIO_SELECT_BASE + 0x58,
    GPIO_FALLING_ENABLE_1 = GPIO_SELECT_BASE + 0x5C,
} GPIOBaseAddrT;

typedef enum PinOutputTypeT { 
    LOW = 0x00,
    HIGH
} PinOutputTypeT;

// define mode for the GPIO pin
typedef enum PinModeT { 
    INPUT = 0x00,
    OUTPUT, 
    ALT5, 
    ALT4, 
    ALT0, 
    ALT1, 
    ALT2, 
    ALT3
} PinModeT;

// struct for GPIO pin
typedef struct PinT { 
    uint32_t pin_number; 
} PinT; 

void GPIO_set_mode(PinT *pin, PinModeT mode);
void GPIO_set_output(PinT *pin);
void GPIO_set_input(PinT *pin);
void GPIO_set(PinT *pin, PinOutputTypeT value);
PinOutputTypeT GPIO_read(PinT *pin);
void GPIO_set_high(PinT *pin);
void GPIO_set_low(PinT *pin);

// GPIO interrupts
void GPIO_enable_rising_edge_int(PinT *pin);
bool GPIO_is_int_enabled();
void GPIO_enable_falling_edge_int(PinT *pin);
bool GPIO_is_int_detected(PinT *pin);
void GPIO_clear_int(PinT *pin);
void GPIO_enable_general_int(PinT *pin);

