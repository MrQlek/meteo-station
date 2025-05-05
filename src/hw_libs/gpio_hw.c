#include "lang_utils.h"

#include "gpio_hw.h"

void gpio_set_mode(GPIO_TypeDef * port, uint8_t pin, gpio_mode_t mode) {
    _assert(is_gpio_port_correct(port));
    _assert(is_gpio_pin_correct(port, pin));
    _assert(VERIFY_ENUM(mode, gpio_mode_t));

    const uint32_t moder_possition = pin * 2;
    const uint32_t moder_mask = 0b11 << moder_possition;

    port->MODER = ((port->MODER & (~moder_mask))
        | (mode << moder_possition));
}

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

void gpio_set_alternate_function(GPIO_TypeDef * port, uint8_t pin, 
    gpio_alternate_function_t alternate_function) {
    
    _assert(is_gpio_port_correct(port));
    _assert(is_gpio_pin_correct(port, pin));

    if(pin < 8) {
        const uint8_t alternate_function_possition = pin * 4;
        port->AFR[0] = 
            ((port->AFR[0]
                & (~((uint32_t)(0b1111) << alternate_function_possition)))
            | (alternate_function << alternate_function_possition));
    } else {
        const uint8_t alternate_function_possition = (uint8_t)((pin - 8) * 4);
        port->AFR[1] = 
            ((port->AFR[1] 
                & (~((uint32_t)(0b1111) << alternate_function_possition)))
            | (alternate_function << alternate_function_possition));
    }
}

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

void gpio_set_adc_connection(GPIO_TypeDef * port, uint8_t pin, 
    const gpio_adc_connection_t connection) {

    _assert(is_gpio_port_correct(port));
    _assert(is_gpio_pin_correct(port, pin));
    _assert(VERIFY_ENUM(connection, gpio_adc_connection_t));

    const uint32_t connection_mask = 0b1 << pin;
    port->ASCR = ((port->ASCR & (~connection_mask))
        | (connection << pin));
}

void gpio_set_output_type(GPIO_TypeDef * port, uint8_t pin, gpio_output_type_t type) {
    _assert(is_gpio_port_correct(port));
    _assert(is_gpio_pin_correct(port, pin));
    _assert(VERIFY_ENUM(type, gpio_output_type_t));

    uint32_t pin_mask = 0x01 << pin;
    port->OTYPER = ((port->OTYPER & (~pin_mask))
        | (type << pin));
}