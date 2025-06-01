#include "lang_utils.h"
#include "io_utils.h"
#include "asserts.h"

#include "io_pins.h"

#include "console_hw.h"
#include "clocks_hw.h"
#include "clocks.h"

#include "weather_sensor.h"
#include "light_sensor.h"
#include "lcd.h"

#define UI_UPDATE_PERIOD_MS (5 * MILLISECONDS_PER_SECOND)

typedef struct {
    weather_sensor_measurements_t weather_sensor_measurement;
    uint32_t light_sensor_lux;
} ui_external_data_t;

typedef void (*ui_display_f)(const uint8_t, const ui_external_data_t * const);

void string_right_padding(MUT char * const buffor,
    const uint16_t buffor_size,
    const uint16_t msg_len,
    const char padding_char) {

    assert_pointer(buffor);

    for(int i = msg_len; i < buffor_size - 1; i++) {
        buffor[i] = padding_char;
    }
    buffor[buffor_size - 1] = 0;
}

static void ui_display_temperature(const uint8_t display_row,
    const ui_external_data_t * const data_to_display) {

    assert_pointer(data_to_display);

    char line_msg[LCD_LINE_LEN + 1] = ARRAY_INIT_ALL_ZEROS;

    const int32_t temperature_centiC =
        data_to_display->weather_sensor_measurement.temperature_centiC;

    int msg_len = snprintf(line_msg, NELEMS(line_msg), "TEMP: %02ld.%02ldC",
        temperature_centiC/100,
        temperature_centiC%100);
    
    _assert(msg_len >= 0);

    string_right_padding(line_msg, NELEMS(line_msg), (uint16_t)msg_len, ' ');
    lcd_write_string(display_row, 0, line_msg);
}

static void ui_display_pressure(const uint8_t display_row,
    const ui_external_data_t * const data_to_display) {

    assert_pointer(data_to_display);

    char line_msg[LCD_LINE_LEN + 1];

    int msg_len = snprintf(line_msg, NELEMS(line_msg), "PRES: %luhPa",
        data_to_display->weather_sensor_measurement.pressure_Pa/100);

    _assert(msg_len >= 0);

    string_right_padding(line_msg, NELEMS(line_msg), (uint16_t)msg_len, ' ');
    lcd_write_string(display_row, 0, line_msg);
}

static void ui_display_humidity(const uint8_t display_row,
    const ui_external_data_t * const data_to_display) {

    assert_pointer(data_to_display);

    char line_msg[LCD_LINE_LEN + 1];

    const uint32_t humidity_permille =
        data_to_display->weather_sensor_measurement.humidity_permille;

    int msg_len = snprintf(line_msg, NELEMS(line_msg), "HUM: %02lu.%01lu%%",
        humidity_permille/10,
        humidity_permille%10);

    _assert(msg_len >= 0);

    string_right_padding(line_msg, NELEMS(line_msg), (uint16_t)msg_len, ' ');
    lcd_write_string(display_row, 0, line_msg);
}

static void ui_display_light(const uint8_t display_row,
    const ui_external_data_t * const data_to_display) {

    assert_pointer(data_to_display);

    char line_msg[LCD_LINE_LEN + 1];

    const uint32_t light_sensor_lux =
        data_to_display->light_sensor_lux;

    int msg_len = snprintf(line_msg, NELEMS(line_msg), "LIGHT: %lu LUX",
        light_sensor_lux);

    _assert(msg_len >= 0);

    string_right_padding(line_msg, NELEMS(line_msg), (uint16_t)msg_len, ' ');
    lcd_write_string(display_row, 0, line_msg);
}

typedef struct {
    time_ms_t last_display_update_time_ms;
    uint8_t row0_display_function_index;
    bool do_force_update;
} ui_context_t;

void ui_init(MUT ui_context_t * const context) {
    assert_pointer(context);

    lcd_init();

    context->last_display_update_time_ms = 0;
    context->row0_display_function_index = 0;
    context->do_force_update = true;
}

void ui_update(MUT ui_context_t * const context,
    const time_ms_t now_ms,
    const ui_external_data_t * const data_to_display) {

    assert_pointer(context);
    assert_pointer(data_to_display);

    const ui_display_f display_functions_arr[] = {
        ui_display_temperature,
        ui_display_pressure,
        ui_display_humidity,
        ui_display_light
    };

    if(context->do_force_update ||
        now_ms - context->last_display_update_time_ms > UI_UPDATE_PERIOD_MS) {

        _assert(
            context->row0_display_function_index < NELEMS(display_functions_arr));

        display_functions_arr[context->row0_display_function_index](
            0, data_to_display);

        const uint8_t row1_display_function_index = (uint8_t)
            ((context->row0_display_function_index + 1)%NELEMS(display_functions_arr));

        display_functions_arr[row1_display_function_index](
            1, data_to_display);

        context->row0_display_function_index = row1_display_function_index;
        context->last_display_update_time_ms = now_ms;
        context->do_force_update = false;
    }
}

int main() {
    clocks_init();

    console_init();
    weather_sensor_context_t weather_sensor_context = STRUCT_INIT_ALL_ZEROS;
    weather_sensor_init(&weather_sensor_context);

    light_sensor_init();

    ui_context_t ui_context = STRUCT_INIT_ALL_ZEROS;
    ui_init(&ui_context);

    INFO("FW VERSION %s, build at %s", FW_VERSION, DATETIME);


    while(1) {
        const time_ms_t now_ms = clock_ms();

        const weather_sensor_measurements_t weather_sensor_measurements =
            weather_sensor_read_measurements(&weather_sensor_context);

        const uint32_t light_sensor_lux = light_sensor_read_lux();

        const ui_external_data_t data_for_ui = {
            .weather_sensor_measurement = weather_sensor_measurements,
            .light_sensor_lux = light_sensor_lux,
        };

        ui_update(&ui_context, now_ms, &data_for_ui);
    }
    return 0;
}