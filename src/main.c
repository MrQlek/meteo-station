#include "lang_utils.h"
#include "io_utils.h"

#include "gpio_hw.h"
#include "clocks_hw.h"
#include "io_pins.h"

#include "console_hw.h"
#include "clocks.h"

#include "weather_sensor.h"

void light_sensor_init_hw() {
    clocks_switch_peripheral_clock(CLOCK_GPIOC, CLOCK_ENABLED);
    gpio_set_mode(LIGHT_SENSOR_PORT, LIGHT_SENSOR_PIN, GPIO_MODE_ANALOG); 
    gpio_set_adc_connection(LIGHT_SENSOR_PORT, LIGHT_SENSOR_PIN, 
        GPIO_ADC_CONNECTION_ENABLED);

    clocks_switch_adc_clock_to_system_clock();
    clocks_switch_peripheral_clock(CLOCK_ADC, CLOCK_ENABLED);

    ADC123_COMMON->CCR |= ADC_CCR_CKMODE_1;

    ADC1->CR &= ~ADC_CR_DEEPPWD_Msk;

    ADC1->CR &= ~ADC_CR_ADVREGEN;
    ADC1->CR |= ADC_CR_ADVREGEN;
    delay_ms(1);

    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL);

    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));

    const uint32_t ADC_INPUT_FOR_GPIOC_0 = 1;
    ADC1->SQR1 = ADC_INPUT_FOR_GPIOC_0 << ADC_SQR1_SQ1_Pos;

    const uint32_t ADC_SAMPLING_TIME_6_POINT_5_CLOCK_CYCLES = 0b001;
    ADC1->SMPR1 |= ADC_SAMPLING_TIME_6_POINT_5_CLOCK_CYCLES << ADC_SMPR1_SMP1_Pos;
}

uint32_t light_sensor_adc_read_mV() {
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_EOC));

    const uint32_t adc_raw = ADC1->DR;

    const uint32_t REFERENCE_VOLTAGE_mV = 3300;
    return ((REFERENCE_VOLTAGE_mV * adc_raw)/ 4096);
}

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

int main() {
    clocks_init();

    clocks_switch_peripheral_clock(CLOCK_GPIOA, CLOCK_ENABLED);

    console_init();
    weather_sensor_context_t weather_sensor_context = STRUCT_INIT_ALL_ZEROS;
    weather_sensor_init(&weather_sensor_context);

    light_sensor_init();


    INFO("FW VERSION %s, build at %s", FW_VERSION, DATETIME);

    while(1) {
        OUT("hello world");
        const weather_sensor_measurements_t weather_sensor_measurements =
            weather_sensor_read_measurements(&weather_sensor_context);

        DUMPI(weather_sensor_measurements.temperature_centiC);
        DUMPI(weather_sensor_measurements.pressure_Pa);
        DUMPI(weather_sensor_measurements.humidity_permille);
        DUMPI(weather_sensor_measurements.result);
        DUMPI(light_sensor_adc_read_mV());
        DUMPI(light_sensor_read_lux());

        delay_ms(1000);
    }
    return 0;
}