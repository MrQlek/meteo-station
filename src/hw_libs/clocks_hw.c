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
        case CLOCK_GPIOB:
            RCC->AHB2ENR = (RCC->AHB2ENR & (~RCC_AHB2ENR_GPIOBEN_Msk))
                | target_state << RCC_AHB2ENR_GPIOBEN_Pos;
            break;
        case CLOCK_GPIOC:
            RCC->AHB2ENR = (RCC->AHB2ENR & (~RCC_AHB2ENR_GPIOCEN_Msk))
                | target_state << RCC_AHB2ENR_GPIOCEN_Pos;
            break;
        case CLOCK_USART2:
            RCC->APB1ENR1 = (RCC->APB1ENR1 & (~RCC_APB1ENR1_USART2EN_Msk))
                | target_state << RCC_APB1ENR1_USART2EN_Pos;
            break;
        case CLOCK_SPI1:
            RCC->APB2ENR = (RCC->APB2ENR & (~RCC_APB2ENR_SPI1EN_Msk))
                | target_state << RCC_APB2ENR_SPI1EN_Pos;
            break;        
        case CLOCK_I2C1:
            RCC->APB1ENR1 = (RCC->APB1ENR1 & (~RCC_APB1ENR1_I2C1EN_Msk))
                | target_state << RCC_APB1ENR1_I2C1EN_Pos;
            break;
        case CLOCK_ADC:
            RCC->AHB2ENR = (RCC->AHB2ENR & (~RCC_AHB2ENR_ADCEN_Msk))
                | target_state << RCC_AHB2ENR_ADCEN_Pos;

        default:
            _assert(0);
            break;
    }
}

void clocks_switch_adc_clock_to_system_clock() {
    RCC->CCIPR |= 0b11 << RCC_CCIPR_ADCSEL_Pos;
}