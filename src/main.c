#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx.h"
#include "gpio_hw.h"
#include "io_pins.h"

int main() {
    char fw_version[] = FW_VERSION;
    char datetime[] = DATETIME;
    (void)fw_version;
    (void)datetime;

    RCC->AHB2ENR |= (1 << RCC_AHB2ENR_GPIOAEN_Pos);
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN_Msk;

    gpio_set_mode(LED_PORT, LED_PIN, GPIO_MODE_OUTPUT);

    // INIT GPIO FOR UART
    gpio_set_mode(DEBUG_UART_TX_PORT, DEBUG_UART_TX_PIN,
        GPIO_MODE_ALTERNATE_FUNCTION);
    gpio_set_mode(DEBUG_UART_RX_PORT, DEBUG_UART_RX_PIN,
        GPIO_MODE_ALTERNATE_FUNCTION);

    gpio_set_alternate_function(DEBUG_UART_TX_PORT, DEBUG_UART_TX_PIN, 
        GPIO_ALTERNATE_FUNCTION_A2_USART2_TX);
    gpio_set_alternate_function(DEBUG_UART_RX_PORT, DEBUG_UART_RX_PIN,
        GPIO_ALTERNATE_FUNCTION_A3_USART2_RX);    

    // INIT UART
    USART2->BRR = 416; // 4 MHz / 9600
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;    

    gpio_state_t led_state = GPIO_STATE_LOW;

    while(1) {
        led_state = (led_state == GPIO_STATE_LOW) ? GPIO_STATE_HIGH : GPIO_STATE_LOW;
        gpio_set_state(LED_PORT, LED_PIN, led_state);

        char text[] = "hello world \r\n";
        for(int i = 0; text[i] != 0; i++) {
            USART2->TDR = text[i];
            while(!(USART2->ISR & USART_ISR_TC));
        }

        for(volatile int i = 0; i < 1000000; i++);
    }
    return 0;
}