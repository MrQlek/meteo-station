#include "lang_utils.h"
#include "io_utils.h"

#include "io_pins.h"

#include "console_hw.h"
#include "clocks_hw.h"
#include "clocks.h"

#include "weather_sensor.h"
#include "light_sensor.h"
#include "lcd.h"

int main() {
    clocks_init();

    console_init();
    weather_sensor_context_t weather_sensor_context = STRUCT_INIT_ALL_ZEROS;
    weather_sensor_init(&weather_sensor_context);

    light_sensor_init();

    lcd_init();

    INFO("FW VERSION %s, build at %s", FW_VERSION, DATETIME);


    while(1) {
        OUT("hello world");
        // const weather_sensor_measurements_t weather_sensor_measurements =
        //     weather_sensor_read_measurements(&weather_sensor_context);

        lcd_write_string(0, 0, "Hello");
        lcd_write_string(1, 0, "world");


        delay_ms(5000);
    }
    return 0;
}