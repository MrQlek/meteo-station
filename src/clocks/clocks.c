#include "clocks.h"

static time_ms_t uptime_ms = 0;

void SysTick_Handler() {
    uptime_ms++;
}

time_ms_t clock_ms() {
    return uptime_ms;
}

void delay_ms(time_ms_t ms) {
    time_ms_t start_ms = clock_ms();
    while(clock_ms() - start_ms < ms);
}
