#include "lang_utils.h"
#include "io_utils.h"

#include "gpio_hw.h"
#include "clocks_hw.h"
#include "io_pins.h"

#include "console_hw.h"
#include "clocks.h"

#define WEATHER_SENSOR_SPI_SCK_PORT GPIOA
#define WEATHER_SENSOR_SPI_SCK_PIN 5

#define WEATHER_SENSOR_SPI_MISO_PORT GPIOA
#define WEATHER_SENSOR_SPI_MISO_PIN 6

#define WEATHER_SENSOR_SPI_MOSI_PORT GPIOA
#define WEATHER_SENSOR_SPI_MOSI_PIN 7

#define WEATHER_SENSOR_SPI_CS_PORT GPIOB
#define WEATHER_SENSOR_SPI_CS_PIN 6

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

#define WEATHER_SENSOR_SPI (SPI1)
#define SPI_DATA_SIZE_8BITS (0b0111)
#define SPI_FIFO_TRIGGER_8BITS (0x01)

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

#define UNUSED(variable) ((void) (variable))
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

typedef enum PACKED {
    BME280_REGISTER_ADDRESS_ID = 0x50,
    BME280_REGISTER_ADDRESS_CTRL_HUMIDITY = 0x72,
    BME280_REGISTER_ADDRESS_CTRL_MEASUREMENT = 0x74,
    BME280_REGISTER_ADDRESS_READOUT_ALL_MEASUREMENTS = 0x77,
    BME280_REGISTER_ADDRESS_READOUT_PRESSURE_MSB = 0x77,
    BME280_REGISTER_ADDRESS_READOUT_TEMPERATURE_MSB = 0x7A,
    BME280_REGISTER_ADDRESS_READOUT_HUMIDITY_MSB = 0x7D,
} bme280_register_address_t;
ASSERT(sizeof(bme280_register_address_t) == 1);

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

#define OUTPUT
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

#define BME280_ID (0x60)

#define BME280_CTRL_HUMIDITY_OVERSAMPLING_X1 (0b001)
#define BME280_CTRL_HUMIDITY_OVERSAMPLING_POS (0)

#define BME280_CTRL_MEASUREMENT_TEMPERATURE_OVERSAMPLING_X1 (0b001)
#define BME280_CTRL_MEASUREMENT_TEMPERATURE_OVERSAMPLING_POS (5)

#define BME280_CTRL_MEASUREMENT_PRESSURE_OVERSAMPLING_X1 (0b001)
#define BME280_CTRL_MEASUREMENT_PRESSURE_OVERSAMPLING_POS (2)

#define BME280_CTRL_MEASUREMENT_MODE_NORMAL (0b11)
#define BME280_CTRL_MEASUREMENT_MODE_POS (0)

#define PROPAGATE_ERROR(_val) { \
    result_t _res = _val; \
    if(_res != RESULT_OK) { \
        return _res; \
    } \
}

result_t weather_sensor_init() {
    weather_sensor_init_hw();

    byte_result_t result = bme280_read_single(BME280_REGISTER_ADDRESS_ID);
    if(result.result != RESULT_OK
        || result.value != BME280_ID) {
        
        return RESULT_GENERIC_ERROR;
    }

    PROPAGATE_ERROR(bme280_write_single(BME280_REGISTER_ADDRESS_CTRL_HUMIDITY, 
        BME280_CTRL_HUMIDITY_OVERSAMPLING_X1 << BME280_CTRL_HUMIDITY_OVERSAMPLING_POS));
    PROPAGATE_ERROR(bme280_write_single(BME280_REGISTER_ADDRESS_CTRL_MEASUREMENT, 
        (BME280_CTRL_MEASUREMENT_TEMPERATURE_OVERSAMPLING_X1 << BME280_CTRL_MEASUREMENT_TEMPERATURE_OVERSAMPLING_POS)
        | (BME280_CTRL_MEASUREMENT_PRESSURE_OVERSAMPLING_X1 << BME280_CTRL_MEASUREMENT_PRESSURE_OVERSAMPLING_POS)
        | (BME280_CTRL_MEASUREMENT_MODE_NORMAL << BME280_CTRL_MEASUREMENT_MODE_POS)));

    return RESULT_OK;
}

//data types based on datasheet 
// to match variable types in compenstaion functions (page 23)
typedef struct {
    int32_t temperature;
    int32_t pressure;
    int32_t humidity;
    result_t result;
} bme280_measurements_raw_t;

#define BME280_PRESSURE_READOUT_RAW_SIZE_BYTES (3)
#define BME280_TEMPERATURE_READOUT_RAW_SIZE_BYTES (3)
#define BME280_HUMIDITY_READOUT_RAW_SIZE_BYTES (2)

#define BME280_ALL_READOUT_RAW_SIZE_BYTES ( \
    BME280_PRESSURE_READOUT_RAW_SIZE_BYTES \
        + BME280_TEMPERATURE_READOUT_RAW_SIZE_BYTES \
        + BME280_HUMIDITY_READOUT_RAW_SIZE_BYTES)

static result_t bme280_pressure_raw_value_valid(int32_t temperature_raw) {
    //based on datasheet (page 27)
    if(temperature_raw == 0x80000) {
        return RESULT_GENERIC_ERROR;
    }
    return RESULT_OK;
}

static result_t bme280_temperature_raw_value_valid(int32_t temperature_raw) {
    //based on datasheet (page 27)
    if(temperature_raw == 0x80000) {
        return RESULT_GENERIC_ERROR;
    }
    return RESULT_OK;
}

static result_t bme280_humidity_raw_value_valid(int32_t temperature_raw) {
    //based on datasheet (page 26)
    if(temperature_raw == 0x8000) {
        return RESULT_GENERIC_ERROR;
    }
    return RESULT_OK;
}

static bme280_measurements_raw_t bme280_read_measurements_raw() {
    ASSERT(BME280_ALL_READOUT_RAW_SIZE_BYTES == 8);

    uint8_t all_measurements_readout[
        BME280_ALL_READOUT_RAW_SIZE_BYTES] = ARRAY_INIT_ALL_ZEROS;
    result_t readout_result = bme280_read_multiple(
        BME280_REGISTER_ADDRESS_READOUT_ALL_MEASUREMENTS,
        all_measurements_readout, NELEMS(all_measurements_readout));

    int32_t pressure_raw = (all_measurements_readout[0] << 12)
        | (all_measurements_readout[1] << 4)
        | ((all_measurements_readout[2] >> 4) & 0x0F);

    int32_t temperature_raw = (all_measurements_readout[3] << 12)
        | (all_measurements_readout[4] << 4)
        | ((all_measurements_readout[5] >> 4) & 0x0F);

    int32_t humidity_raw = (all_measurements_readout[6] << 8)
        | (all_measurements_readout[7] << 0);

    result_t final_result = RESULT_GENERIC_ERROR;

    if(readout_result == RESULT_OK
        && bme280_pressure_raw_value_valid(pressure_raw) == RESULT_OK
        && bme280_temperature_raw_value_valid(temperature_raw) == RESULT_OK
        && bme280_humidity_raw_value_valid(humidity_raw) == RESULT_OK) {

        final_result = RESULT_OK;
    }

    return (bme280_measurements_raw_t) {
        .temperature = temperature_raw,
        .pressure = pressure_raw,
        .humidity = humidity_raw,
        .result = final_result};
}

int main() {
    clocks_init();

    clocks_switch_peripheral_clock(CLOCK_GPIOA, CLOCK_ENABLED);

    console_init();
    weather_sensor_init();


    INFO("FW VERSION %s, build at %s", FW_VERSION, DATETIME);

    while(1) {
        bme280_measurements_raw_t raw_measurements = bme280_read_measurements_raw();
        DUMPI(raw_measurements.temperature);
        DUMPI(raw_measurements.pressure);
        DUMPI(raw_measurements.humidity);
        DUMPI(raw_measurements.result);

        OUT("hello world");

        delay_ms(1000);
    }
    return 0;
}