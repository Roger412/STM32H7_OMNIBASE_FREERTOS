// uart_rx.h
#ifndef UART_RX_H
#define UART_RX_H

#include <stdbool.h>
#include <stdint.h>

#define RX_BUF_SIZE 256

static uint8_t rx_buf[RX_BUF_SIZE];   // Ring buffer storage
static volatile size_t rx_head;
static volatile size_t rx_tail;
static uint8_t rx_char;               // Temporary char buffer

#endif
