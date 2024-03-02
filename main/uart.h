#ifndef UART_H
#define UART_H

#include <stdio.h>
#include "driver/uart.h"
#define BUFF_SIZE 256

typedef struct
{
    uint8_t data[BUFF_SIZE];
    uint8_t lenght;
} uart_data_t;

void uart_begin(uart_port_t port, int baud_rate, uint8_t tx_pin, uint8_t rx_pin, uart_parity_t parity);
void uart_read(uart_port_t port, uart_data_t *uart, uint16_t timeout_ms);
void uart_write(uart_port_t port, uint8_t *data, uint8_t size);
#endif