#include <stdint.h>
#include "stm32l4xx.h"

#define _RCC_BASE (0x40021000)
#define _RCC_AHB2ENR_OFFSET (0x4C)
#define _RCC_AHB2ENR (*((volatile uint32_t*)(_RCC_BASE + _RCC_AHB2ENR_OFFSET)))
#define _RCC_AHB2ENR_GPIOAEN_POS (0)
#define _RCC_AHB2ENR_GPIOCEN_POS (2)

#define _GPIO_MODER_OFFSET (0x00)
#define _GPIO_ODR_OFFSET (0x14)
#define _GPIO_IDR_OFFSET (0x10)

#define _GPIO_MODE5_POS (10)
#define _GPIO_MODE5_MASK (0b11 << _GPIO_MODE5_POS)

#define _GPIO_MODE13_POS (26)
#define _GPIO_MODE13_MASK (0b11 << _GPIO_MODE13_POS)

#define _GPIOA_BASE (0x48000000)
#define _GPIOA_MODER (*((volatile uint32_t*)(_GPIOA_BASE + _GPIO_MODER_OFFSET)))
#define _GPIOA_ODR (*((volatile uint32_t*)(_GPIOA_BASE + _GPIO_ODR_OFFSET)))

#define _GPIOC_BASE (0x48000800)
#define _GPIOC_MODER (*((volatile uint32_t*)(_GPIOC_BASE + _GPIO_MODER_OFFSET)))
#define _GPIOC_ODR (*((volatile uint32_t*)(_GPIOC_BASE + _GPIO_ODR_OFFSET)))
#define _GPIOC_IDR (*((const volatile uint32_t*)(_GPIOC_BASE + _GPIO_IDR_OFFSET)))


#define LED_PIN 5
#define LED_PIN_ODR_MASK (0x01 << LED_PIN)
#define BUTTON_PIN 13
#define BUTTON_PIN_IDR_MASK (0x01 << BUTTON_PIN)

int main_blink_reg() {
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

int main_button_reg() {
    _RCC_AHB2ENR |= (1 << _RCC_AHB2ENR_GPIOAEN_POS);
    _RCC_AHB2ENR |= (1 << _RCC_AHB2ENR_GPIOCEN_POS);

    _GPIOA_MODER = ((_GPIOA_MODER & (~_GPIO_MODE5_MASK))
        | (0x01 << _GPIO_MODE5_POS));
    _GPIOC_MODER = ((_GPIOC_MODER & (~_GPIO_MODE13_MASK))
        | (0x00 << _GPIO_MODE13_POS));

    while (1) {
        uint8_t button_state = (_GPIOC_IDR >> BUTTON_PIN) & 0x01;

        _GPIOA_ODR = ((_GPIOA_ODR & (~LED_PIN_ODR_MASK))
            | (button_state << LED_PIN));

    }
}

int main_blink() {
    RCC->AHB2ENR |= (1 << RCC_AHB2ENR_GPIOAEN_Pos);

    GPIOA->MODER = ((GPIOA->MODER & (~GPIO_MODER_MODE5_Msk))                  
        | (0x01 << GPIO_MODER_MODE5_Pos));

    while(1) {                                                                
        GPIOA->ODR ^= LED_PIN_ODR_MASK;
        for (uint32_t i = 0; i < 1000000; i++);                             
    }    
}

int main(void) {
    RCC->AHB2ENR |= (1 << RCC_AHB2ENR_GPIOAEN_Pos);                           
    RCC->AHB2ENR |= (1 << RCC_AHB2ENR_GPIOCEN_Pos);

    GPIOA->MODER = ((GPIOA->MODER & (~GPIO_MODER_MODE5_Msk)) 
        | (0x01 << GPIO_MODER_MODE5_Pos));
    GPIOC->MODER = ((GPIOC->MODER & (~GPIO_MODER_MODE13_Msk)) 
        | (0x00 << GPIO_MODER_MODE13_Pos));

    while(1) {
        uint8_t button_state = (GPIOC->IDR & BUTTON_PIN_IDR_MASK) >> BUTTON_PIN;
        GPIOA->ODR = ((GPIOA->ODR & (~LED_PIN_ODR_MASK))
            | ((button_state) << LED_PIN));
    }
}

