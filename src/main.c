#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "stm32l4xx.h"
#include "gpio_hw.h"
#include "io_pins.h"

typedef uint32_t time_ms_t;

static time_ms_t uptime_ms = 0;

void SysTick_Handler() {
    uptime_ms++;
}

time_ms_t clock_ms() {
    return uptime_ms;
}

#define MILLISECONDS_PER_SECOND 1000

void delay_ms(time_ms_t ms) {
    time_ms_t start_ms = clock_ms();
    while(clock_ms() - start_ms < ms);
}

void clocks_init() {
    // Set default clock MSI to 8 MHz
    RCC->CR = ((RCC->CR & (~RCC_CR_MSIRANGE_Msk))
        | RCC_CR_MSIRANGE_7);
    RCC->CR = RCC->CR | RCC_CR_MSIRGSEL;

    // Set SysTick to 1 ms
    SysTick_Config(8000); //SYSTEM CLOCK IS MSI WITH 8 MHz. Interrupt every 1 ms
}

typedef enum {
    CLOCK_GPIOA,
    CLOCK_USART2,
    clock_t_LIMIT
} clock_t;

typedef enum {
    CLOCK_DISABLED = 0x00,
    CLOCK_ENABLED = 0x01,
    clock_state_t_LIMIT
} clock_state_t;

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

typedef struct {
    uint8_t * const storage;
    uint16_t head;
    uint16_t tail;
    const uint16_t maxlen;
} ring_buffer_t;

#define NELEMS(a) ((int)(sizeof(a)/sizeof((a)[0])))
#define RING_BUFFER(_storage) (ring_buffer_t) { \
    .storage = _storage, \
    .head = 0, \
    .tail = 0, \
    .maxlen = NELEMS(_storage) \
}

typedef enum {
    RESULT_OK = 0,
    RESULT_GENERIC_ERROR = 1,
} result_t;

typedef struct {
    uint8_t value;
    result_t result;
} byte_result_t;

#define BYTE_RESULT(_val, _result) (byte_result_t) { \
    .value = _val, \
    .result = _result, \
}

#define MUT
result_t ring_buffer_push(MUT ring_buffer_t * const buffer, uint8_t data) {
    assert_pointer(buffer);

    uint16_t next = buffer->head + 1;

    if(next >= buffer->maxlen) {
        next = 0;
    }

    if(next == buffer->tail) {
        return RESULT_GENERIC_ERROR;
    }

    buffer->storage[buffer->head] = data;
    buffer->head = next;

    return RESULT_OK;
}

byte_result_t ring_buffer_pop(MUT ring_buffer_t * const buffer) {
    assert_pointer(buffer);

    if(buffer->head == buffer->tail) {
        return BYTE_RESULT(0, RESULT_GENERIC_ERROR); 
    }

    uint16_t next = buffer->tail + 1;
    if(next >= buffer->maxlen) {
        next = 0;
    }

    uint8_t data = buffer->storage[buffer->tail];
    buffer->tail = next;
    return BYTE_RESULT(data, RESULT_OK);
}

uint16_t ring_buffer_left_space(const ring_buffer_t * const buffer) {
    if(buffer->tail > buffer->head) {
        return buffer->tail - buffer->head;
    }

    //TODO check if this is correct
    return (uint16_t)(
        ((uint16_t)(buffer->maxlen - buffer->head) + buffer->tail) - 1);
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

static void console_init() {
    console_init_hw();
}

#define ARRAY_INIT_ALL_ZEROS {}
static uint8_t console_ring_buffer_storage[128] = ARRAY_INIT_ALL_ZEROS;
static ring_buffer_t console_ring_buffer = RING_BUFFER(
    console_ring_buffer_storage);
static bool is_uart_transfer_in_progress = false;

#define ARM_CRITICAL_SECTION(action) { \
    uint32_t irq_state = __get_PRIMASK(); \
    __disable_irq(); \
    action; \
    if(irq_state == 0) { \
        __enable_irq(); \
    } \
}

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

#define UNUSED_PARAM            __attribute__((unused))

int _write(int file UNUSED_PARAM, char *ptr, int len) {
    assert_pointer(ptr);
    return console_write_blocking(ptr, len);
}

#define NEWLINE "\r\n"
#define OUT(msg, ...) printf(msg NEWLINE, ##__VA_ARGS__)
#define INFO(msg, ...) OUT("INFO: "msg, ##__VA_ARGS__)
#define DUMPI(x) OUT(# x " = %d", (int)x)
#define DUMPX(x) OUT(# x " = 0x%02x", (unsigned int)x)

int main() {
    clocks_init();

    clocks_switch_peripheral_clock(CLOCK_GPIOA, CLOCK_ENABLED);
    gpio_set_mode(LED_PORT, LED_PIN, GPIO_MODE_OUTPUT);

    console_init();

    gpio_state_t led_state = GPIO_STATE_LOW;
    int test_var = 256;

    INFO("FW VERSION %s, build at %s", FW_VERSION, DATETIME);

    while(1) {
        led_state = (led_state == GPIO_STATE_LOW) ? GPIO_STATE_HIGH : GPIO_STATE_LOW;
        gpio_set_state(LED_PORT, LED_PIN, led_state);

        OUT("hello world");
        DUMPI(test_var);
        DUMPX(test_var);

        delay_ms(1000);
    }
    return 0;
}