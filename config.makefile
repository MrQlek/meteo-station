C_DEFS += \
    -DSTM32L476xx


C_FLAGS += \
 -mcpu=cortex-m4 \
 -mthumb \
 -std=c99 \
 --specs=nano.specs \
 -g

LD_FLAGS += \
 -mcpu=cortex-m4 \
 -mthumb \
    --specs=nano.specs \
    --specs=nosys.specs 
LD_FLAGS += -T"linker_script.ld"

COMPILATION_FLAGS += \
    -Wall \
    -Wextra \
	-Wformat-security \
	-Wformat-signedness \
	-Wformat-truncation \
	-Wnonnull \
	-Wnull-dereference \
	-Wuninitialized \
    -Werror

PROJECT_ONLY_FLAGS += -Wconversion
