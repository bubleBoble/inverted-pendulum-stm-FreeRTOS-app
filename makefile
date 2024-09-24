.PHONY: help all debug cmake_debug release cmake_release format-linux flash-debug flash-release clean-debug clean-release

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

debug: ${BUILD_DIR}/debug cmake_debug
##? debug: Build binary debug image
	@$(MAKE) -C ${BUILD_DIR}/debug --no-print-directory

${BUILD_DIR}/debug:
	@mkdir -p ${BUILD_DIR}/debug

cmake_debug: ${BUILD_DIR}/debug/Makefile

${BUILD_DIR}/debug/Makefile: CMakeLists.txt
	@cmake \
		-G "$(BUILD_SYSTEM)" \
		-B${BUILD_DIR}/debug \
		-DPROJECT_NAME=$(PROJECT_NAME) \
		-DCMAKE_BUILD_TYPE=Debug \
		-DPROJECT_DIR=${PROJECT_DIR} \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		-DDUMP_ASM=OFF

release: ${BUILD_DIR}/release $(BUILD_DIR)/release cmake_release
##? release: Build binary release image
	@$(MAKE) -C ${BUILD_DIR}/release --no-print-directory

${BUILD_DIR}/release:
	@mkdir -p ${BUILD_DIR}/release

cmake_release: ${BUILD_DIR}/release ${BUILD_DIR}/release/Makefile

${BUILD_DIR}/release/Makefile: CMakeLists.txt
	@cmake \
		-G "$(BUILD_SYSTEM)" \
		-B${BUILD_DIR}/release \
		-DPROJECT_NAME=$(PROJECT_NAME) \
		-DCMAKE_BUILD_TYPE=Release \
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

flash-debug: debug
##? flash-debug: Flash binary debug image into MCU
	@st-flash --reset write $< 0x08000000

flash-release: release
##? flash-release: Flash binary release image into MCU
	@st-flash --reset write $< 0x08000000

clean-debug:
##? clean-debug: Clean debug build directory
	@echo "[CLEANING-DEBUG]"
	@cd $(BUILD_DIR)/debug; make clean --no-print-directory

clean-release:
##? clean-release: Clean release build directory
	@echo "[CLEANING-RELEASE]"
	@cd $(BUILD_DIR)/release; make clean --no-print-directory

com:
##? com: minicom -b 115200 -o -D /dev/ttyACM0, ctrl+a, q to quit
	minicom -b 115200 -o -D /dev/ttyACM0