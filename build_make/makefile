###############################################################################
# Specify gcc path
#     > make -j GCC_PATH=/usr/bin
# for gcc in `/usr/bin/arm-none-eabi-gcc`
#
# Or use it with arm-none-eabi-gcc already in system PATH:
#     > make -j
#
# Build with debug information
#     > make -j DEBUG=1
# or like this
#     > make debug
# Build release version (no debug info)
#     > make release
#
# In order for a target to appear in `make help`, right after target line
# write help string starting with `##?`, eg.:
#     all:
#     ##? all - this says what all target does
#
# Set VERBOSE=1 to print all compilation commands
#     make VERBOSE=1, default is 0
###############################################################################


###############################################################################
# Scipt for coloring make output, defines "runt". "runt" is used to color the
# output of compilation command
# 
# Instead of:
#     $(BUILD_DIR)/%.o: %.c makefile | $(BUILD_DIR)
#         $(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
# run:
#     COMMAND = $(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
#     $(BUILD_DIR)/%.o: %.c makefile | $(BUILD_DIR)
#         @$(eval cmd=$(COMMAND))
#         @$(call runt,$(cmd),"BUILDING")
###############################################################################
###############################################################################
# Phony targets
###############################################################################
.PHONY: help all clean flash com debug release


###############################################################################
# Pretty colors for build output info
###############################################################################
include pretty.mk


###############################################################################
# Target
###############################################################################
# Name of the final executable
TARGET = LIP_app_1


###############################################################################
# Build variables
###############################################################################
# Debug - Set this flag to 1 to build binary with debug information
DEBUG ?= 1

# Optimization
OPT ?= -O0 # -O, -O0, -O1, -O2, -O3, -Ofast, -Og, -Os

# Print or don't print compilation commands
VERBOSE ?= 0

###############################################################################
# Paths
###############################################################################
# Build path - directory for build output and remporary files
BUILD_DIR = build


###############################################################################
# Source
###############################################################################
# C sources
C_SOURCES =  \
../Core/Src/main.c \
../Core/Src/gpio.c \
../Core/Src/adc.c \
../Core/Src/dma.c \
../Core/Src/i2c.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/sysmem.c \
../Core/Src/syscalls.c \
../Core/Src/system_stm32f4xx.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
../FreeRTOS/Source/croutine.c \
../FreeRTOS/Source/event_groups.c \
../FreeRTOS/Source/list.c \
../FreeRTOS/Source/queue.c \
../FreeRTOS/Source/stream_buffer.c \
../FreeRTOS/Source/tasks.c \
../FreeRTOS/Source/timers.c \
../FreeRTOS/Source/portable/MemMang/heap_4.c \
../FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
$(wildcard ../FreeRTOS-CLI/*.c) \
$(wildcard ../LIP/source/*.c) \
$(wildcard ../LIP/as5600_driver/src/*.c) \
$(wildcard ../LIP/as5600_driver/interface/*.c) \
$(wildcard ../LIP/as5600_driver/example/*.c)

# ASM sources for plain assembly files, these are fed to assembler without
# preprocessing. These files doesn't support C preprocessor directives like
# `#include`, `#define` or conditional compilation directives like `#ifdef`
ASM_SOURCES = \
startup_stm32f429xx.s

# ASM sources for asm files that support C preprocessor directives
ASMM_SOURCES = \


###############################################################################
# Binaries
###############################################################################
# Prefix for cross-compilation tool, eg. arm-none-eabi-gcc
PREFIX = arm-none-eabi-

# The gcc compiler bin path can be either defined in make command via
# GCC_PATH variable ($ make GCC_PATH=xxx) or it can be added to the PATH 
# environment variable.
ifdef GCC_PATH
	CC = $(GCC_PATH)/$(PREFIX)gcc
	AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
	CP = $(GCC_PATH)/$(PREFIX)objcopy
	SZ = $(GCC_PATH)/$(PREFIX)size
else
	CC = $(PREFIX)gcc
	AS = $(PREFIX)gcc -x assembler-with-cpp
	CP = $(PREFIX)objcopy
	SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S


###############################################################################
# CFLAGS
###############################################################################
# CPU, FPU, FLOAT-ABI
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard

# MCU from above specification
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Macros for gcc
# AS defines - Assembly specific definitions
AS_DEFS =

# C defines - C preprocessor definitions
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F429xx

# AS includes
AS_INCLUDES =

# C include paths
C_INCLUDES =  \
-I../Core/Inc \
-I../Drivers/STM32F4xx_HAL_Driver/Inc \
-I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-I../Drivers/CMSIS/Device/ST/STM32F4xx/Include \
-I../Drivers/CMSIS/Include \
-I../FreeRTOS/Source/include \
-I../FreeRTOS/Source/portable/GCC/ARM_CM4F \
-I../FreeRTOS-CLI/ \
-I../LIP/include/ \
-I../LIP/as5600_driver/src/ \
-I../LIP/as5600_driver/interface/ \
-I../LIP/as5600_driver/example/

# GCC compile flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) \
-Wall \
-fdata-sections \
-ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) \
-Wall \
-Wdouble-promotion \
-fdata-sections \
-ffunction-sections \
-fsingle-precision-constant \
-ffast-math \
-pedantic

# Add debug flags to CFLAGS if DEBUG mode enabled
ifeq ($(DEBUG), 1)
	CFLAGS += -g3 -gdwarf-2
endif

# Generate dependency information
# -----------------------------------------------------------------------------
# -MMD: Generate dependency information (.d file) for each source file
# -MP: Add phony targets for each dependency
# -MF: Dependency destination file
# (@:%.o=%.d): Change object file name into the same name with .d
# -----------------------------------------------------------------------------
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


###############################################################################
# LDFLAGS
###############################################################################
# linker script
# Libraries
# -----------------------------------------------------------------------------
# -lc: link C standard library aka libc.a
# -lm: link libm library
# -lnosys: link nosys library, minimal sys calls implementation 
#          stubs for system calls
# nano.specs: link newlib-nano, c std lib for embedded
# -T: linker script
# -Wl: tells gcc to pass following flags to the linker
#      -Map: map file specification
#      --cref: include cross reference table in the map file
#      --gc-sections: linker performs garbage collection on unused sections,
#                     removes any code and data sections which are not 
#                     referenced anywhere in the program
# -----------------------------------------------------------------------------
LDSCRIPT = STM32F429ZITx_FLASH.ld
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = \
$(MCU) \
-specs=nano.specs \
-T$(LDSCRIPT) \
$(LIBDIR) \
$(LIBS) \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
-Wl,--gc-sections


###############################################################################
# Building the application
###############################################################################
# -----------------------------------------------------------------------------
# List of objects from C files
# $(notdir $(C_SOURCES:.c=.o)): Converts each .c file from C_SOURCES to .o file 
#                               with the same name. Removes directory part.
#                               `notdir` is make built-in function
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))

# $(sort $(dir $(C_SOURCES))): Extract and sort directories paths from 
#                              C_SOURCES list of files, `sort` also remove 
#                              duplicates
# The whole line creates list of directories containing .c files from C_SOURCES
# `vpath %.c` tells make where to look for c source files
# -----------------------------------------------------------------------------
vpath %.c $(sort $(dir $(C_SOURCES)))

# List of objects from assembly files (.s and .S), analogous to the above
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

# Compile rules, C files into object files
# -----------------------------------------------------------------------------
# $(BUILD_DIR)/%.o: The target pattern for object files 
# %.c: corresponding source file
# makefile: dependency that ensures the rule runs when this makefile changes
# | $(BUILD_DIR): indicates that the BUILD_DIR must exist before the target is 
#                 build, but changes inside BUILD_DIR do not cause the target 
#                 to be rebuilt
# -Wa: passes flags to the asm
# -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)): 
#                 for asm to generate listing file
# -c: compile and assemble, but do not link
# <: denotes dependency form the rule
# -----------------------------------------------------------------------------
COMMAND1 = \
	$(CC) -c $(CFLAGS) \
	-Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) \
	$< -o $@
$(BUILD_DIR)/%.o: %.c makefile | $(BUILD_DIR)
	@if [ "$(VERBOSE)" -eq "1" ]; then \
		echo "$(COMMAND1)"; \
	fi
	@$(eval cmd=$(COMMAND1))
	@$(call runt,$(cmd),"BUILDING")

# For .s and .S assembly files
COMMAND2 = $(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.s makefile | $(BUILD_DIR)
	@$(eval cmd=$(COMMAND2))
	@$(call runt,$(cmd),"BUILDING")

COMMAND3 = $(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S makefile | $(BUILD_DIR)
	@$(eval cmd=$(COMMAND3))
	@$(call runt,$(cmd),"BUILDING")

# Linking objects into ELF 
# OBJECTS is a list of object files compiled and assembled from 
# .c, .s and .S files
COMMAND4 = $(CC) $(OBJECTS) $(LDFLAGS) -o $@
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) makefile
	@$(eval cmd=$(COMMAND4))
	@$(call runt,$(cmd),"LINKING")
	$(SZ) $@

# objcopy from elf into ihex
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

# objcopy from elf into bin
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

# Rule for creating BUILD_DIR
$(BUILD_DIR):
	@mkdir $@


###############################################################################
# Phony targets
###############################################################################
help:
##? help: This help message
	@echo "Usage: "
	@sed -n 's/^##?//p' ${MAKEFILE_LIST} | column -t -s ':' | sed -e 's/^//'

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
##? all: Build binaries .elf, .hex, .bin

clean:
##? clean: Clean build directory
	@echo "[CLEANING]"
	@-rm -fR $(BUILD_DIR)

flash: $(BUILD_DIR)/$(TARGET).bin
##? flash: Flash binary image into MCU
	@st-flash --reset \
	write $< \
	0x08000000

com:
##? com: minicom -b 115200 -o -D /dev/ttyACM0, ctrl+a, q to quit
	minicom -b 115200 -o -D /dev/ttyACM0

debug:
##? debug: Build the project in debug mode
	@$(MAKE) -j DEBUG=1 all
 
release: clean
##? release: Build the project in release mode
	@$(MAKE) -j DEBUG=0 OPT=-O2 all


###############################################################################
# dependencies
###############################################################################
-include $(wildcard $(BUILD_DIR)/*.d)

