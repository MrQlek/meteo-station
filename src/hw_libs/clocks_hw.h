#ifndef __CLOCKS_HW_H__
#define __CLOCKS_HW_H__

#include "stm32l4xx.h"

typedef enum {
    CLOCK_GPIOA,
    CLOCK_USART2,
    clock_t_LIMIT
} clock_t;

typedef enum {
    CLOCK_DISABLED = 0x00,
    CLOCK_ENABLED = 0x01,
    clock_state_t_LIMIT
} clock_state_t;

void clocks_init();
extern void clocks_switch_peripheral_clock(clock_t clock, clock_state_t target_state);

#endif // !__CLOCKS_HW_H__