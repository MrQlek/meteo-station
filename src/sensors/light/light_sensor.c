#include "light_sensor_hw.h"
#include "light_sensor.h"

uint32_t light_sensor_read_lux() {
    const uint32_t adc_readout_mV = light_sensor_adc_read_mV();

    const uint32_t LOADING_RESISTOR_VALUE_OHM = 10000;

    const uint32_t PHOTO_CURRENT_uA = 
        (adc_readout_mV * 1000)/LOADING_RESISTOR_VALUE_OHM;

    return 2 * PHOTO_CURRENT_uA;
}

void light_sensor_init() {
    light_sensor_init_hw();
}
