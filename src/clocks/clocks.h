#ifndef __CLOCKS_H__
#define __CLOCKS_H__

#include "lang_utils.h"

typedef uint32_t time_ms_t;

extern time_ms_t clock_ms();
extern void delay_ms(time_ms_t ms);

#define MILLISECONDS_PER_SECOND 1000

#endif // !__CLOCKS_H__