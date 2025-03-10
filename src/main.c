#include "lang_utils.h"
#include "io_utils.h"

#include "gpio_hw.h"
#include "clocks_hw.h"
#include "io_pins.h"

#include "console_hw.h"
#include "clocks.h"

int main() {
    clocks_init();

    clocks_switch_peripheral_clock(CLOCK_GPIOA, CLOCK_ENABLED);
    gpio_set_mode(LED_PORT, LED_PIN, GPIO_MODE_OUTPUT);

    console_init();

    gpio_state_t led_state = GPIO_STATE_LOW;
    int test_var = 256;

    INFO("FW VERSION %s, build at %s", FW_VERSION, DATETIME);

    while(1) {
        led_state = (led_state == GPIO_STATE_LOW) ? GPIO_STATE_HIGH : GPIO_STATE_LOW;
        gpio_set_state(LED_PORT, LED_PIN, led_state);

        OUT("hello world");
        DUMPI(test_var);
        DUMPX(test_var);

        delay_ms(1000);
    }
    return 0;
}