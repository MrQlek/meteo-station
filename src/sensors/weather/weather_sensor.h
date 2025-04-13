#ifndef __WEATHER_SENSOR_H__
#define __WEATHER_SENSOR_H__

#include "lang_utils.h"
#include "results.h"

#include "weather_sensor_types.h"

typedef struct {
    union {
        uint16_t raw[3];
        struct {
            uint16_t T1;
            int16_t T2;
            int16_t T3;
        } param;
    };
} bme280_temperature_calibration_parameters_t;

typedef struct {
    union {
        uint16_t raw[9];
        struct {
            uint16_t P1;
            int16_t P2;
            int16_t P3;
            int16_t P4;
            int16_t P5;
            int16_t P6;
            int16_t P7;
            int16_t P8;
            int16_t P9;
        } param;
    };
} bme280_pressure_calibration_parameters_t;

typedef struct {
    uint8_t raw[7];
} bme280_humidity_calibration_parameters_raw_H2_H6_t;

typedef struct {
    uint8_t H1;
    int16_t H2;
    uint8_t H3;
    int16_t H4;
    int16_t H5;
    int8_t H6;
} bme280_humidity_calibration_parameters_t;


typedef struct {
    bme280_temperature_calibration_parameters_t temperature_calibration;
    bme280_pressure_calibration_parameters_t pressure_calibration;
    bme280_humidity_calibration_parameters_t humidity_calibration;
} weather_sensor_context_t;

//data types based on datasheet 
// to match variable types in compenstaion functions (page 25)
typedef struct {
    int32_t temperature;
    int32_t pressure;
    int32_t humidity;
    result_t result;
} bme280_measurements_raw_t;

extern result_t weather_sensor_init(MUT weather_sensor_context_t * const context);
extern weather_sensor_measurements_t weather_sensor_read_measurements(
    const weather_sensor_context_t * const context);

#endif // !__WEATHER_SENSOR_H__