#include "lang_utils.h"
#include "io_utils.h"

#include "gpio_hw.h"
#include "clocks_hw.h"
#include "io_pins.h"

#include "console_hw.h"
#include "clocks.h"

#include "weather_sensor.h"
#include "light_sensor.h"
#include "lcd_hw.h"

int main() {
    clocks_init();

    clocks_switch_peripheral_clock(CLOCK_GPIOA, CLOCK_ENABLED);

    console_init();
    weather_sensor_context_t weather_sensor_context = STRUCT_INIT_ALL_ZEROS;
    weather_sensor_init(&weather_sensor_context);

    light_sensor_init();

    lcd_init_hw();
    uint8_t lcd_data = 0;

    INFO("FW VERSION %s, build at %s", FW_VERSION, DATETIME);


    while(1) {
        OUT("hello world");
        // const weather_sensor_measurements_t weather_sensor_measurements =
        //     weather_sensor_read_measurements(&weather_sensor_context);

        lcd_write_data(&lcd_data, sizeof(lcd_data));

        if(lcd_data == 0) {
            lcd_data = 1 << 2 | 1 << 3;
        } else {
            lcd_data = 0;
        }

        delay_ms(5000);
    }
    return 0;
}