#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx.h"

#define _assert(x) ({x;})
#define assert_pointer(x)

#define CAT(a, b) a##b
#define VERIFY_ENUM(x, _enum) (x < CAT(_enum,_LIMIT))

static inline bool is_gpio_port_correct(const GPIO_TypeDef * const port) {
    assert_pointer(port);

    if(port == GPIOA) {
        return true;
    }
    if(port == GPIOB) {
        return true;
    }
    if(port == GPIOC) {
        return true;
    }
    if(port == GPIOD) {
        return true;
    }
    if(port == GPIOE) {
        return true;
    }
    if(port == GPIOF) {
        return true;
    }
    if(port == GPIOG) {
        return true;
    }
    if(port == GPIOH) {
        return true;
    }
    return false;
}

static inline bool is_gpio_pin_correct(const GPIO_TypeDef * const port,
    const uint8_t pin) {

    assert_pointer(port);

    if(port == GPIOH) {
        return (pin <= 2);
    }

    return (pin <= 15);

}

typedef enum {
    GPIO_MODE_INPUT = 0x00, 
    GPIO_MODE_OUTPUT = 0x01,
    GPIO_MODE_ALTERNATE_FUNCTION = 0x02,
    GPIO_MODE_ANALOG = 0x03,
    gpio_mode_t_LIMIT
} gpio_mode_t;

void gpio_set_mode(GPIO_TypeDef * port, uint8_t pin, gpio_mode_t mode) {
    _assert(is_gpio_port_correct(port));
    _assert(is_gpio_pin_correct(port, pin));
    _assert(VERIFY_ENUM(mode, gpio_mode_t));

    const uint32_t moder_possition = pin * 2;
    const uint32_t moder_mask = 0b11 << moder_possition;

    port->MODER = ((port->MODER & (~moder_mask))
        | (mode << moder_possition));    
} 

typedef enum {
    GPIO_STATE_LOW = 0x00,
    GPIO_STATE_HIGH = 0x01,
    gpio_state_t_LIMIT
} gpio_state_t;

void gpio_set_state(GPIO_TypeDef * port, uint8_t pin, gpio_state_t state) {
    _assert(is_gpio_port_correct(port));
    _assert(is_gpio_pin_correct(port, pin));
    _assert(VERIFY_ENUM(state, gpio_state_t));

    uint32_t pin_mask = 0x01 << pin;
    port->ODR = ((port->ODR & (~pin_mask))
        | state << pin);
}

gpio_state_t gpio_read_state(GPIO_TypeDef * port, uint8_t pin) {
    _assert(is_gpio_port_correct(port));
    _assert(is_gpio_pin_correct(port, pin));

    if((port->IDR >> pin) & 0x01) {
        return GPIO_STATE_HIGH;
    }
    return GPIO_STATE_LOW;
}

typedef enum {
    GPIO_ALTERNATE_FUNCTION_A2_USART2_TX = 0x07,
    GPIO_ALTERNATE_FUNCTION_A3_USART2_RX = 0x07,
} gpio_alternate_function_t;

void gpio_set_alternate_function(GPIO_TypeDef * port, uint8_t pin, 
    gpio_alternate_function_t alternate_function) {

    _assert(is_gpio_port_correct(port));
    _assert(is_gpio_pin_correct(port, pin));


    if(pin < 8) {
        const uint8_t alternate_function_possition = pin * 4;
        const uint32_t alternate_function_mask = (uint32_t)(0b1111) << alternate_function_possition;

        port->AFR[0] = 
            ((port->AFR[0] & (~alternate_function_mask))
                | (alternate_function << alternate_function_possition));
    } else {
        const uint8_t alternate_function_possition = (uint8_t)((pin - 8) * 4); 
        const uint32_t alternate_function_mask = (uint32_t)(0b1111) << alternate_function_possition;
    
        port->AFR[1] = 
            ((port->AFR[1] & (~alternate_function_mask))
                | (alternate_function << alternate_function_possition));
    }
}

typedef enum {
    GPIO_PULL_NONE = 0b00,
    GPIO_PULL_UP = 0b01,
    GPIO_PULL_DOWN = 0b10,
    gpio_pull_t_LIMIT
} gpio_pull_t;

void gpio_set_pull_type(GPIO_TypeDef * port, uint8_t pin, 
    const gpio_pull_t pull) {
    
    _assert(is_gpio_port_correct(port));
    _assert(is_gpio_pin_correct(port, pin));
    _assert(VERIFY_ENUM(pull, gpio_pull_t));


    const uint32_t pull_possition = pin * 2;
    const uint32_t pull_mask = 0b11 << pull_possition;

    port->PUPDR = ((port->PUPDR & (~pull_mask))
        | (pull << pull_possition));
}

#define LED_PORT GPIOA
#define LED_PIN 5

#define DEBUG_UART_TX_PORT GPIOA
#define DEBUG_UART_TX_PIN 2

#define DEBUG_UART_RX_PORT GPIOA
#define DEBUG_UART_RX_PIN 3

int main() {
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