#include "asserts.h"
#include "lang_utils.h"

#include "io_pins.h"
#include "gpio_hw.h"
#include "clocks_hw.h"

#include "bme280_types.h"

#include "weather_sensor_hw.h"

#define WEATHER_SENSOR_SPI (SPI1)

static void bme280_gpio_init() {
    clocks_switch_peripheral_clock(CLOCK_GPIOA, CLOCK_ENABLED);
    clocks_switch_peripheral_clock(CLOCK_GPIOB, CLOCK_ENABLED);

    gpio_set_mode(WEATHER_SENSOR_SPI_CS_PORT, WEATHER_SENSOR_SPI_CS_PIN,
        GPIO_MODE_OUTPUT);
    gpio_set_mode(WEATHER_SENSOR_SPI_SCK_PORT, WEATHER_SENSOR_SPI_SCK_PIN,
        GPIO_MODE_ALTERNATE_FUNCTION);
    gpio_set_mode(WEATHER_SENSOR_SPI_MISO_PORT, WEATHER_SENSOR_SPI_MISO_PIN,
        GPIO_MODE_ALTERNATE_FUNCTION);
    gpio_set_mode(WEATHER_SENSOR_SPI_MOSI_PORT, WEATHER_SENSOR_SPI_MOSI_PIN,
        GPIO_MODE_ALTERNATE_FUNCTION);


    gpio_set_state(WEATHER_SENSOR_SPI_CS_PORT, WEATHER_SENSOR_SPI_CS_PIN, 
        GPIO_STATE_HIGH);
    gpio_set_alternate_function(WEATHER_SENSOR_SPI_SCK_PORT,
        WEATHER_SENSOR_SPI_SCK_PIN, 
        GPIO_ALTERNATE_FUNCTION_A5_SPI1_SCK);
    gpio_set_alternate_function(WEATHER_SENSOR_SPI_MISO_PORT,
        WEATHER_SENSOR_SPI_MISO_PIN, 
        GPIO_ALTERNATE_FUNCTION_A6_SPI1_MISO);
    gpio_set_alternate_function(WEATHER_SENSOR_SPI_MOSI_PORT,
        WEATHER_SENSOR_SPI_MOSI_PIN, 
        GPIO_ALTERNATE_FUNCTION_A7_SPI1_MOSI);
}

static void bme280_spi_init() {
    clocks_switch_peripheral_clock(CLOCK_SPI1, CLOCK_ENABLED);

    //SET MASTER MODE
    WEATHER_SENSOR_SPI->CR1 = (WEATHER_SENSOR_SPI->CR1 & (~SPI_CR1_MSTR_Msk))
        | 0x01 << SPI_CR1_MSTR_Pos;

    //SET SINGLE MASTER MODE
    WEATHER_SENSOR_SPI->CR2 = (WEATHER_SENSOR_SPI->CR2 & (~SPI_CR2_SSOE_Msk))
        | 0x01 << SPI_CR2_SSOE_Pos;

    //SET DATA SIZE
    WEATHER_SENSOR_SPI->CR2 = (WEATHER_SENSOR_SPI->CR2 & (~SPI_CR2_DS_Msk))
        | SPI_DATA_SIZE_8BITS << SPI_CR2_DS_Pos;

    //SET FIFO TRIGER
    WEATHER_SENSOR_SPI->CR2 = (WEATHER_SENSOR_SPI->CR2 & (~SPI_CR2_FRXTH_Msk))
        | SPI_FIFO_TRIGGER_8BITS << SPI_CR2_FRXTH_Pos;

    //ENABLE SPI
    WEATHER_SENSOR_SPI->CR1 = (WEATHER_SENSOR_SPI->CR1 & (~SPI_CR1_SPE_Msk))
        | 0x01 << SPI_CR1_SPE_Pos;
}

void weather_sensor_init_hw() {
    bme280_gpio_init();
    bme280_spi_init();
}

static void spi_clear_rx_buffor() {
    uint16_t data = 0;
    data = *((uint8_t*)&WEATHER_SENSOR_SPI->DR);
    UNUSED(data);
}

static byte_result_t spi_write_and_read_byte(const uint8_t byte_to_write) {
    spi_clear_rx_buffor();
    while(WEATHER_SENSOR_SPI->SR & SPI_SR_BSY);
    *((uint8_t*)&WEATHER_SENSOR_SPI->DR) = byte_to_write;
    while(WEATHER_SENSOR_SPI->SR & SPI_SR_BSY);

    uint8_t read_byte = 0;
    uint8_t read_bytes_num = 0;
    while(WEATHER_SENSOR_SPI->SR & SPI_SR_RXNE) {
        read_byte = *((uint8_t*)&WEATHER_SENSOR_SPI->DR);
        read_bytes_num++;
    }

    return (byte_result_t) {
        .value = read_byte,
        .result = read_bytes_num == 1 ? RESULT_OK : RESULT_GENERIC_ERROR,
    };
}


byte_result_t bme280_read_single(const bme280_register_address_t address) {
    gpio_set_state(WEATHER_SENSOR_SPI_CS_PORT, WEATHER_SENSOR_SPI_CS_PIN, 
        GPIO_STATE_LOW);

    spi_write_and_read_byte(address | 0x80);
    byte_result_t result =  spi_write_and_read_byte(0x00);

    gpio_set_state(WEATHER_SENSOR_SPI_CS_PORT, WEATHER_SENSOR_SPI_CS_PIN, 
        GPIO_STATE_HIGH);

    return result;
}

result_t bme280_write_single(const bme280_register_address_t address,
    const uint8_t value_to_write) {
    gpio_set_state(WEATHER_SENSOR_SPI_CS_PORT, WEATHER_SENSOR_SPI_CS_PIN, 
        GPIO_STATE_LOW);

    spi_write_and_read_byte((uint8_t)(address & (~0x80)));
    byte_result_t result =  spi_write_and_read_byte(value_to_write);

    gpio_set_state(WEATHER_SENSOR_SPI_CS_PORT, WEATHER_SENSOR_SPI_CS_PIN, 
        GPIO_STATE_HIGH);

    return result.result;
}

result_t bme280_read_multiple(
    const bme280_register_address_t address,
    OUTPUT uint8_t * const output,
    const uint8_t registers_to_read) {

    assert_pointer(output);

    gpio_set_state(WEATHER_SENSOR_SPI_CS_PORT, WEATHER_SENSOR_SPI_CS_PIN, 
        GPIO_STATE_LOW);

    spi_write_and_read_byte(address | 0x80);
    result_t result = RESULT_OK;
    for(int i = 0; i < registers_to_read; i++) {
        byte_result_t byte_result =  spi_write_and_read_byte(0x00);
        if(byte_result.result != RESULT_OK) {
            result = byte_result.result;
            break;
        }
        output[i] = byte_result.value;
    }
    gpio_set_state(WEATHER_SENSOR_SPI_CS_PORT, WEATHER_SENSOR_SPI_CS_PIN, 
        GPIO_STATE_HIGH);

    return result;
}