#include <stdint.h>
#include "rpi.h"
#include "gpio-custom.h"

// We assume the system clock rate is 250MHz
#define BAUD_RATE 115200
#define BAUD_RATE_REG 270
#define BASE_AUX 0x20215000
enum {
    AUX_ENABLES = BASE_AUX + 0x04
};

enum { 
    AUX_MU_IO_REG       = BASE_AUX + 0x40,
    AUX_MU_IER_REG       = BASE_AUX + 0x44,
    AUX_MU_IIR_REG       = BASE_AUX + 0x48,
    AUX_MU_LCR_REG       = BASE_AUX + 0x4C,   // data size is a 2 bit field. write 0b11 for 8bit
    AUX_MU_MCR_REG       = BASE_AUX + 0x50,   
    AUX_MU_LSR_REG       = BASE_AUX + 0x54,   
    AUX_MU_MSR_REG       = BASE_AUX + 0x58,
    AUX_MU_SCRATCH       = BASE_AUX + 0x5C,
    AUX_MU_CNTL_REG      = BASE_AUX + 0x60,
    AUX_MU_STAT_REG      = BASE_AUX + 0x64,
    AUX_MU_BAUD_REG      = BASE_AUX + 0x68
}; 

enum { 
    UART_TX_PIN = 14,
    UART_RX_PIN = 15
};

PinT uart_tx_pin = {UART_TX_PIN};
PinT uart_rx_pin = {UART_RX_PIN};

// initialize [XXX: we should take a baud rate?]
void uart_init(void);
// disable
void uart_device_enable(void);
void uart_enable(void);
void uart_disable(void);

// put one byte on the uart:
// returns < 0 on error.
int uart_put8(uint8_t c);

// returns -1 if no byte, the value otherwise.
int uart_get8_async(void);

// 0 = no data, 1 = at least one byte
int uart_has_data(void);

int uart_tx_is_empty(void);
// flush out the tx fifo
void uart_flush_tx(void);

void uart_send_int(uint32_t num);
void uart_send_string(const char *msg);
void uart_set_baud_rate(uint32_t baud_reg);