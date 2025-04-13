#ifndef __WEATHER_SENSOR_TYPES_H__
#define __WEATHER_SENSOR_TYPES_H__

#include "results.h"

typedef struct {
    int32_t temperature_centiC;
    uint32_t pressure_Pa;
    uint32_t humidity_permille;
    result_t result;
} weather_sensor_measurements_t;

#endif // !__WEATHER_SENSOR_TYPES_H__