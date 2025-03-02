#ifndef __GPIO_HW_H__
#define __GPIO_HW_H__
#include "stm32l4xx.h"

#include "asserts.h"
#include "lang_utils.h"


typedef enum {
    GPIO_MODE_INPUT = 0x00,
    GPIO_MODE_OUTPUT = 0x01,
    GPIO_MODE_ALTERNATE_FUNCTION = 0x02,
    GPIO_MODE_ANALOG = 0x03,
    gpio_mode_t_LIMIT
} gpio_mode_t;

typedef enum {
    GPIO_STATE_LOW = 0x00,
    GPIO_STATE_HIGH = 0x01,
    gpio_state_t_LIMIT
} gpio_state_t;


typedef enum {
    GPIO_ALTERNATE_FUNCTION_A2_USART2_TX = 0x07,
    GPIO_ALTERNATE_FUNCTION_A3_USART2_RX = 0x07,
} gpio_alternate_function_t;

typedef enum {
    GPIO_PULL_NONE = 0b00,
    GPIO_PULL_UP = 0b01,
    GPIO_PULL_DOWN = 0b10,
    gpio_pull_t_LIMIT
} gpio_pull_t;

extern void gpio_set_mode(GPIO_TypeDef * port, uint8_t pin, gpio_mode_t mode);

extern void gpio_set_state(GPIO_TypeDef * port, uint8_t pin, gpio_state_t state);
extern gpio_state_t gpio_read_state(GPIO_TypeDef * port, uint8_t pin);

extern void gpio_set_alternate_function(GPIO_TypeDef * port, uint8_t pin, 
    gpio_alternate_function_t alternate_function);
extern void gpio_set_pull_type(GPIO_TypeDef * port, uint8_t pin, 
    const gpio_pull_t pull);

static inline bool is_gpio_port_correct(const GPIO_TypeDef * const port) {
    assert_pointer(port)

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

    assert_pointer(port)
    if(port == GPIOH) {
        return (pin <= 2);
    }

    return (pin <= 15);
}

#endif // !__GPIO_HW_H__