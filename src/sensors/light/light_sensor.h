#ifndef __LIGHT_SENSOR_H__
#define __LIGHT_SENSOR_H__

#include "lang_utils.h"

extern void light_sensor_init();
extern uint32_t light_sensor_read_lux();

#endif // !__LIGHT_SENSOR_H__