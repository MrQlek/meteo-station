#include "stm32l4xx.h"

#include "io_pins.h"

#include "ring_buffer.h"
#include "gpio_hw.h"
#include "clocks_hw.h"

#include "lang_utils.h"
#include "asserts.h"
#include "arm_utils.h"

#include "clocks.h"

#include "console_hw.h"

static uint8_t console_ring_buffer_storage[128] = ARRAY_INIT_ALL_ZEROS;
static ring_buffer_t console_ring_buffer = RING_BUFFER(
    console_ring_buffer_storage);
static bool is_uart_transfer_in_progress = false;

int console_write(char *ptr, int len) {
    assert_pointer(ptr);

    if(len > ring_buffer_left_space(&console_ring_buffer)) {
        return 0;
    }

    for(int i = 0; i < len; i++) {
        ring_buffer_push(&console_ring_buffer, ptr[i]);
    }

    ARM_CRITICAL_SECTION({
        if(!is_uart_transfer_in_progress) {
            byte_result_t next_byte_result = ring_buffer_pop(&console_ring_buffer);
            if(next_byte_result.result == RESULT_OK) {
                USART2->TDR = next_byte_result.value;
                is_uart_transfer_in_progress = true;
            }
        }
    });

    return len;
}

int console_write_blocking(char *ptr, int len) {
    assert_pointer(ptr);
    int return_len = console_write(ptr, len);

    time_ms_t start_time_ms = clock_ms();
    const time_ms_t TIMEOUT_MS = MILLISECONDS_PER_SECOND;
    while(is_uart_transfer_in_progress
        && clock_ms() - start_time_ms < TIMEOUT_MS);

    _assert(is_uart_transfer_in_progress == false);

    return return_len;
}

int _write(int file UNUSED_PARAM, char *ptr, int len) {
    assert_pointer(ptr);
    return console_write_blocking(ptr, len);
}

static void console_init_gpio() {
    clocks_switch_peripheral_clock(CLOCK_GPIOA, CLOCK_ENABLED);

    gpio_set_mode(DEBUG_UART_TX_PORT, DEBUG_UART_TX_PIN,
        GPIO_MODE_ALTERNATE_FUNCTION);
    gpio_set_mode(DEBUG_UART_RX_PORT, DEBUG_UART_RX_PIN,
        GPIO_MODE_ALTERNATE_FUNCTION);

    gpio_set_alternate_function(DEBUG_UART_TX_PORT, DEBUG_UART_TX_PIN, 
        GPIO_ALTERNATE_FUNCTION_A2_USART2_TX);
    gpio_set_alternate_function(DEBUG_UART_RX_PORT, DEBUG_UART_RX_PIN,
        GPIO_ALTERNATE_FUNCTION_A3_USART2_RX);
}

static uint16_t calculate_brr_register_value(const uint32_t baudrate, 
    const uint32_t uart_clock_frequency_hz) {

    uint32_t brr_value = uart_clock_frequency_hz/baudrate;

    _assert(brr_value < UINT16_MAX);

    return (uint16_t)brr_value;
}

static void console_init_hw() {
    console_init_gpio();

    clocks_switch_peripheral_clock(CLOCK_USART2, CLOCK_ENABLED);

    USART2->BRR = calculate_brr_register_value(115200, 8000000);
    USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE;
    
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 8);
    NVIC_EnableIRQ(USART2_IRQn);

}

void console_init() {
    console_init_hw();
}

void usart2_irq_handler() {
    uint32_t flags = USART2->ISR;

    if(flags & USART_ISR_TC) {
        USART2->ICR |= USART_ICR_TCCF;

        byte_result_t next_byte_result = ring_buffer_pop(&console_ring_buffer);

        if(next_byte_result.result == RESULT_OK) {
            USART2->TDR = next_byte_result.value;
        } else {
            is_uart_transfer_in_progress = false;
        }
    }
}
