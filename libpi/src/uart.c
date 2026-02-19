#include "rpi.h"
#include "uart.h"

// initialize [XXX: we should take a baud rate?]
void uart_init(void) { 
    dev_barrier();

    // GPIO pins should be set up before enabling UART
    GPIO_set_mode(&uart_tx_pin, ALT5);
    GPIO_set_mode(&uart_rx_pin, ALT5);

    // Set both GPIO pins to high 
    // GPIO_set_high(&uart_tx_pin);
    // GPIO_set_high(&uart_rx_pin);
    
    // we're writing to a different device now so we need barrier
    dev_barrier();
    uart_device_enable();
    dev_barrier();
    
    PUT32(AUX_MU_CNTL_REG, 0x00);   // Disable TX/RX 
    PUT32(AUX_MU_IER_REG, 0x00); // Disable interrupts
    PUT32(AUX_MU_IIR_REG, 0b110); // clear FIFOs
    PUT32(AUX_MU_LCR_REG, 0b11);    // Set data size to 8 bits
    PUT32(AUX_MU_MCR_REG, 0x00);
    PUT32(AUX_MU_BAUD_REG, BAUD_RATE_REG);  // Set baud rate
    uart_enable();
    dev_barrier();
}

void uart_enable(void) { 
    PUT32(AUX_MU_CNTL_REG, 0b11); // enable TX and RX
}

void uart_device_enable(void) { 
    uint32_t enables = GET32(AUX_ENABLES);
    enables |= 0b1; // set bit 0 to enable mini UART
    PUT32(AUX_ENABLES, enables);
}

void uart_disable(void) { 
    dev_barrier();
    uart_flush_tx();
    PUT32(AUX_MU_CNTL_REG, 0x00); // disable mini UART
    // rmw to aux enables
    uint32_t enables = GET32(AUX_ENABLES);
    enables &= !0b1; // clear bit 0 to disable mini UART
    PUT32(AUX_ENABLES, enables);
    dev_barrier();
}

int uart_get8(void) { 
    dev_barrier();
    // check bit 5 of LSR
    while ((GET32(AUX_MU_LSR_REG) & 1) == 0) {
        // wait for data to be available
    }
    int res = (int)(GET32(AUX_MU_IO_REG) & 0xFF);
    dev_barrier();
    return res;
}

int uart_put8(uint8_t c) { 
    dev_barrier();
    while ((GET32(AUX_MU_LSR_REG) & (0b100000)) == 0) {
        // wait for data to be available
    }
    PUT32(AUX_MU_IO_REG, c);
    dev_barrier();
    return 1;
}

// returns -1 if no byte, the value otherwise.
int uart_get8_async(void) { 
    dev_barrier();
    if(!uart_has_data())
        return -1;
    return uart_get8();
}

// 0 = no data, 1 = at least one byte
// bit 0 is set if the receive FIFO contains at least 1 symbol
int uart_has_data(void) { 
    dev_barrier();
    int res = (GET32(AUX_MU_LSR_REG) & 0x01) != 0;  
    dev_barrier();
    return res;
}

int uart_tx_is_empty(void) { 
    dev_barrier();
    // bit 6 (7th bit) of LSR is set if TX FIFO is empty and idle
    int res = (GET32(AUX_MU_LSR_REG) & (0b1000000)) != 0;
    dev_barrier();
    return res;
}

// flush out the tx fifo
void uart_flush_tx(void) { 
    while(!uart_tx_is_empty()){}
}

void uart_send_int(uint32_t num) {
    if (num == 0) {
        uart_put8('0');
    }
    char output[11] = "\0\0\0\0\0\0\0\0\0\0";
    uint8_t i = 0;
    while (num > 0 && i < sizeof(output)) {
        output[i] = '0' + num % 10;
        num = num / 10;
        i++;
    }
    
    for (char *c = output + i - 1; c >= output; c--) {
        if (*c) {
            uart_put8(*c);
        }
    }
}

void uart_send_string(const char *msg) {
    while (*msg) {
        uart_put8(*msg);
        msg++;
    }
}

void uart_set_baud_rate(uint32_t baud_reg) {
    dev_barrier();
    PUT32(AUX_MU_CNTL_REG, 0); // disable TX/RX while changing baud rate
    PUT32(AUX_MU_BAUD_REG, baud_reg);
    PUT32(AUX_MU_CNTL_REG, 0b11); // re-enable 
    dev_barrier();
}