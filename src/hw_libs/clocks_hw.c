#include "asserts.h"
#include "lang_utils.h"

#include "clocks_hw.h"

void clocks_init() {
    // Set default clock MSI to 8 MHz
    RCC->CR = ((RCC->CR & (~RCC_CR_MSIRANGE_Msk))
        | RCC_CR_MSIRANGE_7);
    RCC->CR = RCC->CR | RCC_CR_MSIRGSEL;

    // Set SysTick to 1 ms
    SysTick_Config(8000); //SYSTEM CLOCK IS MSI WITH 8 MHz. Interrupt every 1 ms
}

void clocks_switch_peripheral_clock(clock_t clock, clock_state_t target_state) {
    _assert(VERIFY_ENUM(clock, clock_t));
    _assert(VERIFY_ENUM(target_state, clock_state_t));

    switch (clock) {
        case CLOCK_GPIOA:
            RCC->AHB2ENR = (RCC->AHB2ENR & (~RCC_AHB2ENR_GPIOAEN_Msk))
                | target_state << RCC_AHB2ENR_GPIOAEN_Pos;
            break;
        case CLOCK_USART2:
            RCC->APB1ENR1 = (RCC->APB1ENR1 & (~RCC_APB1ENR1_USART2EN_Msk))
                | target_state << RCC_APB1ENR1_USART2EN_Pos;
            break;

        default:
            _assert(0);
            break;
    }
}