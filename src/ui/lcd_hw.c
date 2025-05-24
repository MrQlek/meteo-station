#include "lcd_hw.h"
#include "io_pins.h"
#include "gpio_hw.h"
#include "clocks_hw.h"
#include "clocks.h"
#include "asserts.h"

#define LCD_I2C (I2C1)


static void i2c_gpio_init() {
    clocks_switch_peripheral_clock(CLOCK_GPIOB, CLOCK_ENABLED);

    gpio_set_output_type(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN,
        GPIO_OUTPUT_TYPE_OPEN_DRAIN);

    gpio_set_mode(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN,
        GPIO_MODE_ALTERNATE_FUNCTION);
    gpio_set_mode(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN,
        GPIO_MODE_ALTERNATE_FUNCTION);

    gpio_set_alternate_function(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN, 
        GPIO_ALTERNATE_FUNCTION_B9_I2C1_SDA);
    gpio_set_alternate_function(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN, 
        GPIO_ALTERNATE_FUNCTION_B8_I2C1_SCL);
}

static void i2c_init() {
    clocks_switch_peripheral_clock(CLOCK_I2C1, CLOCK_ENABLED);

    // Configure filters (disable analog and enable digital)
    // Disable analog filter
    LCD_I2C->CR1 = (LCD_I2C->CR1 & (~I2C_CR1_ANFOFF_Msk))
        | 0x01 << I2C_CR1_ANFOFF_Pos;

    // Enable digital filter
    LCD_I2C->CR1 = (LCD_I2C->CR1 & (~I2C_CR1_DNF_Msk))
        | 0x01 << I2C_CR1_DNF_Pos;

    // Set I2C timing
    // I2C clock 8 MHz
    // I2C clock after prescaling 1 MHz (period 1 us)
    // Data setup time (SCLDEL) and data hold time (SDADEL) set to 1 us
    // I2C CLK signal target frequency 10 kHz (100 us) with 50% duty cycle
    // SCL high period (100 us / 2) = 50 us -> 49 + 1
    // SCL low period (100 us / 2) = 50 us -> 49 + 1
    LCD_I2C->TIMINGR = (uint32_t)(0x07 << I2C_TIMINGR_PRESC_Pos
        | 0x01 << I2C_TIMINGR_SCLDEL_Pos
        | 0x01 << I2C_TIMINGR_SDADEL_Pos
        | 49 << I2C_TIMINGR_SCLH_Pos
        | 49 << I2C_TIMINGR_SCLL_Pos);

    // Configure NOSTRETCH
    // Need to be cleared for master mode
    LCD_I2C->CR1 = (LCD_I2C->CR1 & (~I2C_CR1_NOSTRETCH_Msk))
        | 0x00 << I2C_CR1_NOSTRETCH_Pos;

    // Enable I2C
    LCD_I2C->CR1 = (LCD_I2C->CR1 & (~I2C_CR1_PE_Msk))
        | 0x01 << I2C_CR1_PE_Pos;
}

static result_t i2c_stop_transmission() {
    LCD_I2C->CR2 |= 0x01 << I2C_CR2_STOP_Pos;
    const time_ms_t timeout_ms = 100;
    const time_ms_t start_ms = clock_ms();

    while(clock_ms() - start_ms < timeout_ms) {
        if(LCD_I2C->ISR & I2C_ISR_STOPF) {
            LCD_I2C->ICR = 0x01 << I2C_ICR_STOPCF_Pos;
            return RESULT_OK;
        }
    }
    return RESULT_TIMEOUT_ERROR;

}

static void i2c_setup_transmission_assert_i2c_is_idle() {
    const time_ms_t timeout_ms = 100;
    const time_ms_t start_ms = clock_ms();

    while((gpio_read_state(LCD_I2C_SCL_PORT, LCD_I2C_SCL_PIN) == GPIO_STATE_LOW
            || gpio_read_state(LCD_I2C_SDA_PORT, LCD_I2C_SDA_PIN) == GPIO_STATE_LOW)
        && clock_ms() - start_ms < timeout_ms);

    _assert(clock_ms() - start_ms < timeout_ms);
}

static result_t i2c_setup_transmission_wait_for_ack(const i2c_direction_t direction) {
    const time_ms_t timeout_ms = 100;
    const time_ms_t start_ms = clock_ms();

    while(clock_ms() - start_ms < timeout_ms) {
        if(LCD_I2C->ISR & I2C_ISR_NACKF) {
            LCD_I2C->ICR = 1 << I2C_ICR_NACKCF_Pos;

            i2c_stop_transmission();
            return RESULT_GENERIC_ERROR;
        }

        switch (direction) {
            case I2C_DIRECTION_WRITE:
                if(LCD_I2C->ISR & I2C_ISR_TXIS) {
                    return RESULT_OK;
                }
                break;
            default:
                _assert(0);
        }
    }
    return RESULT_TIMEOUT_ERROR;
}

static result_t i2c_setup_transmission(const uint8_t address,
    const i2c_direction_t direction, const uint8_t data_len) {
    // Set addressing mode to 7-bit
    LCD_I2C->CR2 = (LCD_I2C->CR2 & (~I2C_CR2_ADD10_Msk))
        | 0x00 << I2C_CR2_ADD10_Pos;

    // Set slave address
    LCD_I2C->CR2 = (LCD_I2C->CR2 & (~I2C_CR2_SADD_Msk))
        | address << (I2C_CR2_SADD_Pos + 1);

    // Set transfer direction
    LCD_I2C->CR2 = (LCD_I2C->CR2 & (~I2C_CR2_RD_WRN_Msk))
        | direction << I2C_CR2_RD_WRN_Pos;

    // Set number of bytes to transfer
    LCD_I2C->CR2 = (LCD_I2C->CR2 & (~I2C_CR2_NBYTES_Msk))
        | data_len << I2C_CR2_NBYTES_Pos;

    // Disable autoend
    LCD_I2C->CR2 = (LCD_I2C->CR2 & (~I2C_CR2_AUTOEND_Msk))
        | 0x00 << I2C_CR2_AUTOEND_Pos;

    // CLEAR ALL INTERRUPT FLAGS BEFOR TRANSMISSION
    LCD_I2C->ICR = I2C_ALL_INTERUPT_FLAGS; 

    // CHECK IF I2C BUS IS IDLE
    i2c_setup_transmission_assert_i2c_is_idle();

    // START TRANSMISSION
    LCD_I2C->CR2 |= 0x01 << I2C_CR2_START_Pos;

    // WAIT FOR ACK
    return i2c_setup_transmission_wait_for_ack(direction);
}

static result_t i2c_transmit_bytes(const uint8_t * const data, const uint8_t data_len) {
    assert_pointer(data);

    const time_ms_t timeout_ms = 100;
    for(int i = 0; i < data_len; i++) {
        const time_ms_t start_ms = clock_ms();

        while (true) {
            if(LCD_I2C->ISR & I2C_ISR_TXIS) {
                LCD_I2C->TXDR = data[i];
                break;
            }

            if(clock_ms() - start_ms >= timeout_ms) {
                return RESULT_TIMEOUT_ERROR;
            }
        }
    }

    const time_ms_t start_ms = clock_ms();
    while(clock_ms() - start_ms < timeout_ms) {
        if(LCD_I2C->ISR & I2C_ISR_TC) {
            return RESULT_OK;
        }
    }
    return RESULT_TIMEOUT_ERROR;
}

static result_t i2c_write_data(const uint8_t * const data, const uint8_t data_len, 
    const uint8_t address) {

    assert_pointer(data);

    PROPAGATE_ERROR(i2c_setup_transmission(address, I2C_DIRECTION_WRITE, data_len));

    result_t transmit_result = i2c_transmit_bytes(data, data_len);
    result_t stop_transmission_result = i2c_stop_transmission();
    
    PROPAGATE_ERROR_AS(transmit_result | stop_transmission_result,
        RESULT_GENERIC_ERROR);

    return RESULT_OK;
}

void lcd_init_hw() {
    i2c_gpio_init();
    i2c_init();
}

result_t lcd_write_data(const uint8_t * const data, const uint8_t data_len) {
    i2c_write_data(data, data_len, LCD_I2C_ADDRESS);

    return RESULT_OK;
}