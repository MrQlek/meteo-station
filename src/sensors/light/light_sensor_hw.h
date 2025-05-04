#ifndef __LIGHT_SENSOR_HW_H__
#define __LIGHT_SENSOR_HW_H__

#include "lang_utils.h"

extern void light_sensor_init_hw();
extern uint32_t light_sensor_adc_read_mV();

#endif // !__LIGHT_SENSOR_HW_H__