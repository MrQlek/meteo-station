#ifndef __IO_PINS_H__
#define __IO_PINS_H__

#include "stm32l4xx.h"

#define LED_PORT GPIOA
#define LED_PIN 5

#define DEBUG_UART_TX_PORT GPIOA
#define DEBUG_UART_TX_PIN 2

#define DEBUG_UART_RX_PORT GPIOA
#define DEBUG_UART_RX_PIN 3

#endif // !__IO_PINS_H__