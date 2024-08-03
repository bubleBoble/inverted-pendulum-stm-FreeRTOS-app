.PHONY: help all build cmake format-linux flash clean

PROJECT_NAME ?= firmware
BUILD_TYPE ?= Debug
BUILD_DIR ?= build_cmake
FIRMWARE := $(BUILD_DIR)/$(PROJECT_NAME).bin
PLATFORM = $(if $(OS),$(OS),$(shell uname -s))
PROJECT_DIR := LIP

# STM32 Device
DEVICE ?= STM32F429ZI

###############################################################################
# Build system for dev platform - cmake generator selection
###############################################################################
ifeq ($(PLATFORM),Windows_NT)
    BUILD_SYSTEM ?= MinGW Makefiles
else
    ifeq ($(PLATFORM),Linux)
        BUILD_SYSTEM ?= Unix Makefiles
    else
        @echo "Unsuported platform"
        exit 1
    endif
endif


###############################################################################
# Phony targets
###############################################################################
help:
##? help: This help message
	@echo "Usage: "
	@sed -n 's/^##?//p' ${MAKEFILE_LIST} | column -t -s ':' | sed -e 's/^//'

all: debug
##? all: Build binaries .elf, .hex, .bin, default is debug

debug: cmake_debug
	@$(MAKE) -C ${BUILD_DIR} --no-print-directory

cmake_debug: ${BUILD_DIR}/Makefile

${BUILD_DIR}/Makefile: CMakeLists.txt
	@cmake \
		-G "$(BUILD_SYSTEM)" \
		-B${BUILD_DIR} \
		-DPROJECT_NAME=$(PROJECT_NAME) \
		-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
		-DPROJECT_DIR=${PROJECT_DIR} \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		-DDUMP_ASM=OFF

# Formats all CubeMX generated sources to unix style - removes \r from line 
# endings
HIDDEN_FILES := .mxproject .project .cproject
FOUND_HIDDEN_FILES := $(shell for f in $(HIDDEN_FILES);do if [[ -e $$f ]]; then echo $$f;fi; done)
FORMAT_LINUX := $(shell find Core Drivers -name '*' -type f; find . -name '*.ioc') $(FOUND_HIDDEN_FILES)

format-linux: $(addsuffix .format-linux,$(FORMAT_LINUX))
##? format-linux: format \r\n to \n

%.format-linux: %
	$(if $(filter $(PLATFORM),Linux),dos2unix -q $<,)

flash: build
##? flash: Flash binary image into MCU
	@st-flash --reset write $< 0x08000000

com:
##? com: minicom -b 115200 -o -D /dev/ttyACM0, ctrl+a, q to quit
	minicom -b 115200 -o -D /dev/ttyACM0

clean:
##? clean: Clean build directory
	@echo "[CLEANING]"
	@cd $(BUILD_DIR); make clean --no-print-directory