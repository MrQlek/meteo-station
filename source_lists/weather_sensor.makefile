INCLUDES += \
    -Isrc/sensors/weather

WEATHER_SENSOR_SOURCE = \
    src/sensors/weather/weather_sensor_hw.c \
    src/sensors/weather/weather_sensor.c

C_SOURCE_FILES += $(WEATHER_SENSOR_SOURCE)