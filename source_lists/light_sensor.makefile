INCLUDES += \
    -Isrc/sensors/light

LIGHT_SENSOR_SOURCE = \
    src/sensors/light/light_sensor_hw.c \
    src/sensors/light/light_sensor.c

C_SOURCE_FILES += $(LIGHT_SENSOR_SOURCE)