#include "lang_utils.h"
#include "io_utils.h"

#include "gpio_hw.h"
#include "clocks_hw.h"
#include "io_pins.h"

#include "console_hw.h"
#include "clocks.h"

#include "weather_sensor.h"

int main() {
    clocks_init();

    clocks_switch_peripheral_clock(CLOCK_GPIOA, CLOCK_ENABLED);

    console_init();
    weather_sensor_context_t weather_sensor_context = STRUCT_INIT_ALL_ZEROS;
    weather_sensor_init(&weather_sensor_context);


    INFO("FW VERSION %s, build at %s", FW_VERSION, DATETIME);

    while(1) {
        OUT("hello world");
        const weather_sensor_measurements_t weather_sensor_measurements =
            weather_sensor_read_measurements(&weather_sensor_context);

        DUMPI(weather_sensor_measurements.temperature_centiC);
        DUMPI(weather_sensor_measurements.pressure_Pa);
        DUMPI(weather_sensor_measurements.humidity_permille);
        DUMPI(weather_sensor_measurements.result);

        delay_ms(1000);
    }
    return 0;
}