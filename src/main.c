#include <stdint.h>
#include "stm32l4xx.h"

#define _RCC_BASE (0x40021000)
#define _RCC_AHB2ENR_OFFSET (0x4C)
#define _RCC_AHB2ENR (*((volatile uint32_t*)(_RCC_BASE + _RCC_AHB2ENR_OFFSET)))
#define _RCC_AHB2ENR_GPIOAEN_POS (0)

#define _GPIO_MODER_OFFSET (0x00)
#define _GPIO_ODR_OFFSET (0x14)

#define _GPIO_MODE5_POS (10)
#define _GPIO_MODE5_MASK (0b11 << _GPIO_MODE5_POS)

#define _GPIOA_BASE (0x48000000)
#define _GPIOA_MODER (*((volatile uint32_t*)(_GPIOA_BASE + _GPIO_MODER_OFFSET)))
#define _GPIOA_ODR (*((volatile uint32_t*)(_GPIOA_BASE + _GPIO_ODR_OFFSET)))


#define LED_PIN 5
#define LED_PIN_ODR_MASK (0x01 << LED_PIN)

int main(void) {
    // ENABLE PERIPHERAL CLOCKS
    _RCC_AHB2ENR |= (1 << _RCC_AHB2ENR_GPIOAEN_POS);

    // SET GPIO MODE
    _GPIOA_MODER = ((_GPIOA_MODER & (~_GPIO_MODE5_MASK))
        | (0b01 << _GPIO_MODE5_POS)); 

    // SET GPIO STATE
    while(1) {
        _GPIOA_ODR ^= LED_PIN_ODR_MASK;

        for (uint32_t i = 0; i < 1000000; i++);
    }
}

