#include "stm32l4xx.h"

#define LED_PIN 5
#define LED_PIN_ODR_MASK (0x01 << LED_PIN)

int main(void) {
    RCC->AHB2ENR |= (1 << RCC_AHB2ENR_GPIOAEN_Pos);

    GPIOA->MODER = ((GPIOA->MODER & (~GPIO_MODER_MODE5_Msk))                  
        | (0x01 << GPIO_MODER_MODE5_Pos));

    while(1) {                                                                
        GPIOA->ODR ^= LED_PIN_ODR_MASK;
        for (uint32_t i = 0; i < 1000000; i++);                             
    }    
}

