#ifndef __WEATHER_SENSOR_HW_H__
#define __WEATHER_SENSOR_HW_H__

#include "asserts.h"
#include "lang_utils.h"
#include "results.h"

#include "bme280_types.h"

#define SPI_DATA_SIZE_8BITS (0b0111)
#define SPI_FIFO_TRIGGER_8BITS (0x01)

extern result_t bme280_write_single(const bme280_register_address_t address,
    const uint8_t value_to_write);
extern byte_result_t bme280_read_single(const bme280_register_address_t address);
extern result_t bme280_read_multiple(
    const bme280_register_address_t address,
    OUTPUT uint8_t * const output,
    const uint8_t registers_to_read);

extern void weather_sensor_init_hw();

#endif // !__WEATHER_SENSOR_HW_H__