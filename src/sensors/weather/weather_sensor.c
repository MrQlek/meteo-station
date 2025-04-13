#include "asserts.h"

#include "bme280_types.h"
#include "weather_sensor_types.h"
#include "weather_sensor_hw.h"

#include "weather_sensor.h"

static result_t bme280_read_temperature_calibration(
    MUT bme280_temperature_calibration_parameters_t * const calibration_parameters) {

    assert_pointer(calibration_parameters);
    return bme280_read_multiple(
        BME208_REGISTER_ADDRESS_TEMPERATURE_CALIBRATION_PARAMETERS_T1,
        (uint8_t*)calibration_parameters->raw,
        sizeof(calibration_parameters->raw));
}

static result_t bme280_read_pressure_calibration(
    MUT bme280_pressure_calibration_parameters_t * const calibration_parameters) {

    assert_pointer(calibration_parameters);
    return bme280_read_multiple(
        BME208_REGISTER_ADDRESS_PRESSURE_CALIBRATION_PARAMETERS_P1,
        (uint8_t*)calibration_parameters->raw,
        sizeof(calibration_parameters->raw));
}

static result_t bme280_read_humidity_calibration(
    MUT bme280_humidity_calibration_parameters_t * const calibration_parameters) {

    assert_pointer(calibration_parameters);
    bme280_humidity_calibration_parameters_raw_H2_H6_t raw = STRUCT_INIT_ALL_ZEROS;
    PROPAGATE_ERROR(bme280_read_multiple(
        BME208_REGISTER_ADDRESS_HUMIDITY_CALIBRATION_PARAMETERS_H1,
        (uint8_t*)&calibration_parameters->H1,
        sizeof(calibration_parameters->H1)));
    PROPAGATE_ERROR(bme280_read_multiple(
        BME208_REGISTER_ADDRESS_HUMIDITY_CALIBRATION_PARAMETERS_H2,
        (uint8_t*)&raw,
        sizeof(raw)));

    //based on datasheet (page 24)
    calibration_parameters->H2 = *((int16_t*)(&raw.raw[0]));
    calibration_parameters->H3 = raw.raw[2];
    calibration_parameters->H4 = ((raw.raw[3] << 4) | (raw.raw[4] & 0x0F));
    calibration_parameters->H5 = ((raw.raw[5] << 4) | ((raw.raw[4] & 0xF0) >> 4));
    calibration_parameters->H6 = (int8_t)raw.raw[6];

    return RESULT_OK;
}

result_t weather_sensor_init(MUT weather_sensor_context_t * const context) {
    weather_sensor_init_hw();

    byte_result_t result = bme280_read_single(BME280_REGISTER_ADDRESS_ID);
    if(result.result != RESULT_OK
        || result.value != BME280_ID) {
        
        return RESULT_GENERIC_ERROR;
    }

    PROPAGATE_ERROR(bme280_read_temperature_calibration(&context->temperature_calibration));
    PROPAGATE_ERROR(bme280_read_pressure_calibration(&context->pressure_calibration));
    PROPAGATE_ERROR(bme280_read_humidity_calibration(&context->humidity_calibration));

    PROPAGATE_ERROR(bme280_write_single(BME280_REGISTER_ADDRESS_CTRL_HUMIDITY, 
        BME280_CTRL_HUMIDITY_OVERSAMPLING_X1 << BME280_CTRL_HUMIDITY_OVERSAMPLING_POS));
    PROPAGATE_ERROR(bme280_write_single(BME280_REGISTER_ADDRESS_CTRL_MEASUREMENT, 
        (BME280_CTRL_MEASUREMENT_TEMPERATURE_OVERSAMPLING_X1 << BME280_CTRL_MEASUREMENT_TEMPERATURE_OVERSAMPLING_POS)
        | (BME280_CTRL_MEASUREMENT_PRESSURE_OVERSAMPLING_X1 << BME280_CTRL_MEASUREMENT_PRESSURE_OVERSAMPLING_POS)
        | (BME280_CTRL_MEASUREMENT_MODE_NORMAL << BME280_CTRL_MEASUREMENT_MODE_POS)));

    return RESULT_OK;
}

static result_t bme280_pressure_raw_value_valid(int32_t temperature_raw) {
    //based on datasheet (page 29)
    if(temperature_raw == 0x80000) {
        return RESULT_GENERIC_ERROR;
    }
    return RESULT_OK;
}

static result_t bme280_temperature_raw_value_valid(int32_t temperature_raw) {
    //based on datasheet (page 29)
    if(temperature_raw == 0x80000) {
        return RESULT_GENERIC_ERROR;
    }
    return RESULT_OK;
}

static result_t bme280_humidity_raw_value_valid(int32_t temperature_raw) {
    //based on datasheet (page 28)
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

//based on code from datasheet (page 25)
static int32_t bme280_compensate_temperature_t_fine(
    const bme280_temperature_calibration_parameters_t * const calibration_params,
    int32_t temperature_raw) {

    assert_pointer(calibration_params);

    int32_t var1 =
        ((((temperature_raw>>3) - ((int32_t)calibration_params->param.T1<<1)))
            * ((int32_t)calibration_params->param.T2)) >> 11;
    int32_t var2 =
        (((((temperature_raw>>4) - ((int32_t)calibration_params->param.T1))
            * ((temperature_raw>>4) - ((int32_t)calibration_params->param.T1))) >> 12)
            * ((int32_t)calibration_params->param.T3)) >> 14;
    return var1 + var2;
}

//based on code from datasheet (page 25)
static int32_t bme280_temperature_centiC_from_t_fine(int32_t t_fine) {
    return (t_fine * 5 + 128) >> 8;
}

//based on code from datasheet (page 25)
static uint32_t bme280_compensate_pressure_Pa(
    const bme280_pressure_calibration_parameters_t * const calibration_params,
    int32_t temperature_fine,
    int32_t pressure_raw) {

    assert_pointer(calibration_params);

    int64_t var1 = ((int64_t)temperature_fine) - 128000;

    int64_t var2 = var1 * var1 * (int64_t)calibration_params->param.P6
        + ((var1*(int64_t)calibration_params->param.P5)<<17)
        + (((int64_t)calibration_params->param.P4)<<35);

    var1 = ((var1 * var1 * (int64_t)calibration_params->param.P3)>>8)
        + ((var1 * (int64_t)calibration_params->param.P2)<<12);

    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration_params->param.P1)>>33;

    //TODO find better solution
    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }

    int64_t p = 1048576-pressure_raw;
    p = (((p<<31)-var2)*3125)/var1;

    var1 = (((int64_t)calibration_params->param.P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)calibration_params->param.P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calibration_params->param.P7)<<4);
    return ((uint32_t)p) >> 8;
}

// based on code from datasheet (page 25)
static uint32_t bme280_compensate_humidity_permille(
    const bme280_humidity_calibration_parameters_t * const calibration_params,
    int32_t temperature_fine,
    int32_t humidity_raw) {

    assert_pointer(calibration_params);
    int32_t v_x1_u32r = (temperature_fine - ((int32_t)76800));
    v_x1_u32r = (
        ((((humidity_raw << 14)
                - (((int32_t)calibration_params->H4) << 20)
                - (((int32_t)calibration_params->H5) * v_x1_u32r))
            + ((int32_t)16384)) >> 15)
        * (((((
            ((v_x1_u32r * ((int32_t)calibration_params->H6)) >> 10)
                * (((v_x1_u32r * ((int32_t)calibration_params->H3)) >> 11)
                    + ((int32_t)32768))) >> 10)
                + ((int32_t)2097152))
            * ((int32_t)calibration_params->H2) + 8192) >> 14));


    v_x1_u32r =
        (v_x1_u32r - ((
            (((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
            * ((int32_t)calibration_params->H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    uint32_t humidity_Q22_10 = (uint32_t)(v_x1_u32r>>12);

    return (humidity_Q22_10 * 10 / 1024);
}

weather_sensor_measurements_t weather_sensor_read_measurements(
    const weather_sensor_context_t * const context) {

    assert_pointer(context);
    bme280_measurements_raw_t raw_measurements = bme280_read_measurements_raw();

    if(raw_measurements.result != RESULT_OK) {
        return (weather_sensor_measurements_t) {
            .result = raw_measurements.result
        };
    }

    int32_t temperature_fine = bme280_compensate_temperature_t_fine(
        &context->temperature_calibration,
        raw_measurements.temperature);

    return (weather_sensor_measurements_t) {
        .temperature_centiC = bme280_temperature_centiC_from_t_fine(temperature_fine),
        .pressure_Pa = bme280_compensate_pressure_Pa(&context->pressure_calibration,
            temperature_fine, raw_measurements.pressure),
        .humidity_permille = bme280_compensate_humidity_permille(
            &context->humidity_calibration,
            temperature_fine, raw_measurements.humidity),
        .result = RESULT_OK};
}