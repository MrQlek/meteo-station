INCLUDES += \
    -Ivendor/CMSIS/CMSIS/Core/Include \
    -Ivendor/CMSIS/Device/ST/STM32L4/Include \
    -Isrc/

MAIN_SOURCE = \
    src/main.c \
    src/startup.c \
    src/syscalls.c \
    vendor/CMSIS/Device/ST/STM32L4/Source/Templates/system_stm32l4xx.c

C_SOURCE_FILES += $(MAIN_SOURCE)