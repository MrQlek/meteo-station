.SUFFIXES:				
.PHONY: all meteo 

all: meteo

PROJECTNAME = meteo

ifeq ($(VERBOSE),)
  ECHO = @
endif

BUILD_DIR = build

OUTPUT_DIR = ${BUILD_DIR}

# Version
ifdef VERSION
	FW_VERSION = \"${VERSION}\"
else
	FW_VERSION := \"$(shell ./tools/version.sh)\"
endif #VERSION

# Values that should be appended by the sub-makefiles
C_SOURCE_FILES   =

LIBS =

# Values that should be appended by config.makefile
C_DEFS   =

INCLUDES =


COMPILATION_FLAGS 	=
PROJECT_ONLY_FLAGS 	=

LD_FLAGS          	=


FLASH_ADDRESS = 0x08000000
UNIX_TIMESTAMP = \"time$(shell date +%s)\"
DATETIME = \"$(shell date --rfc-3339='seconds' |  sed 's/ /,/g')\"
OBJS =

####################################################################
# Definitions of toolchain.                                        #
####################################################################

CC      = "$(ARM_TOOLCHAIN_DIR)/arm-none-eabi-gcc"
OBJCOPY = "$(ARM_TOOLCHAIN_DIR)/arm-none-eabi-objcopy"
LD      = "$(ARM_TOOLCHAIN_DIR)/arm-none-eabi-gcc"

####################################################################
# Include sub-makefiles                                            #
####################################################################

-include config.makefile
-include source_lists/*

####################################################################
# Rules                                                            #
####################################################################

# -MMD : Don't generate dependencies on system header files.
# -MP  : Add phony targets, useful when a h-file is removed from a project.
# -MF  : Specify a file to write the dependencies to.
DEPFLAGS = -MMD -MP -MF $(@:.o=.d)

CSOURCES       = $(C_SOURCE_FILES)

COBJS       = $(addprefix $(OUTPUT_DIR)/,$(CSOURCES:.c=.o))
OBJS        += $(COBJS)

CDEPS       += $(addprefix $(OUTPUT_DIR)/,$(CSOURCES:.c=.d))

C_PATHS   = $(subst \,/,$(sort $(dir $(C_SOURCE_FILES))))
OBJ_DIRS  = $(sort $(dir $(OBJS)))

vpath %.c $(C_PATHS)

# Rule Definitions

meteo: CFLAGS = $(C_FLAGS) $(C_DEFS) $(INCLUDES) $(DEPFLAGS) \
    $(COMPILATION_FLAGS) \
	-DFW_VERSION=$(FW_VERSION) -DDATETIME=$(DATETIME)
meteo: $(OUTPUT_DIR)/$(PROJECTNAME).out


ifneq ($(wildcard /mnt/c/.*),) # Check if executed in WSL
# If WSL
flash: meteo
	~/tools/st-tool -c port=SWD mode=ur -w $(BUILD_DIR)/$(PROJECTNAME).bin $(FLASH_ADDRESS)
else # Not WSL
flash: meteo
	st-flash --connect-under-reset write $(BUILD_DIR)/$(PROJECTNAME).bin $(FLASH_ADDRESS)
endif 

# include auto-generated dependency files (explicit rules)
ifneq (clean,$(findstring clean, $(MAKECMDGOALS)))
-include $(CDEPS)
endif

$(OUTPUT_DIR)/$(PROJECTNAME).out: $(OBJS) $(LIB_FILES) | $(OBJ_DIRS)
	@echo 'Linking $(OUTPUT_DIR)/$(PROJECTNAME).out'
	$(ECHO)$(LD) $(LD_FLAGS) $(OBJS) $(LIBS) -o $(OUTPUT_DIR)/$(PROJECTNAME).out
	$(ECHO)$(OBJCOPY) $(OUTPUT_DIR)/$(PROJECTNAME).out -O binary $(OUTPUT_DIR)/$(PROJECTNAME).bin
	$(ECHO)$(OBJCOPY) $(OUTPUT_DIR)/$(PROJECTNAME).out -O ihex $(OUTPUT_DIR)/$(PROJECTNAME).hex
	$(ECHO)$(OBJCOPY) $(OUTPUT_DIR)/$(PROJECTNAME).out -O srec $(OUTPUT_DIR)/$(PROJECTNAME).s37
	@echo 'Done.'


$(OBJS): | $(OBJ_DIRS)

$(OBJ_DIRS):
	@mkdir -p $@

$(OUTPUT_DIR)/%.o: %.c
	@echo 'Building $<'
	$(ECHO)$(CC) $(CFLAGS) -c -o $@ $<

$(OUTPUT_DIR)/src/%.o: src/%.c
	@mkdir -p `dirname $@`
	@echo 'Building '$@
	$(ECHO)$(CC) $(CFLAGS) $(PROJECT_ONLY_FLAGS) -c -o $@ $<

clean:
	$(RM) -r $(BUILD_DIR)
