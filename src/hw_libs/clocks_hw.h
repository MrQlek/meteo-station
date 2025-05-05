#ifndef __CLOCKS_HW_H__
#define __CLOCKS_HW_H__

#include "stm32l4xx.h"

typedef enum {
    CLOCK_GPIOA,
    CLOCK_GPIOB,
    CLOCK_GPIOC,
    CLOCK_USART2,
    CLOCK_SPI1,
    CLOCK_I2C1,
    CLOCK_ADC,
    clock_t_LIMIT
} clock_t;

typedef enum {
    CLOCK_DISABLED = 0x00,
    CLOCK_ENABLED = 0x01,
    clock_state_t_LIMIT
} clock_state_t;

extern void clocks_init();
extern void clocks_switch_peripheral_clock(clock_t clock, clock_state_t target_state);
extern void clocks_switch_adc_clock_to_system_clock();

#endif // !__CLOCKS_HW_H__