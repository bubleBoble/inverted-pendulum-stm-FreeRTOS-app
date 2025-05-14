.PHONY: \
	help all debug cmake_debug release cmake_release format-linux \
	flash-debug flash-release clean-debug clean-release podman-build-image \
	podman-run-container podman-build-release podman-build-debug

SHELL := /bin/bash

PROJECT_NAME ?= LIP
BUILD_TYPE ?= Debug
BUILD_DIR ?= build_cmake
FIRMWARE_DEBUG := $(BUILD_DIR)/debug/$(PROJECT_NAME).bin
FIRMWARE_RELEASE := $(BUILD_DIR)/release/$(PROJECT_NAME).bin
PLATFORM = $(if $(OS),$(OS),$(shell uname -s))
PROJECT_DIR := application

# STM32 Device
DEVICE ?= STM32F429ZI

################################################################################
# Build system for dev platform - cmake generator selection
################################################################################
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


################################################################################
# Targets
################################################################################
help:
##? help: This help message
	@echo "********************************************************************************"
	@echo "                                     Usage"
	@echo "********************************************************************************"
	@echo ""
	@sed -n 's/^##?//p' ${MAKEFILE_LIST} | column -t -s ':' | sed 's/^/   /'
	@echo ""
	@echo "********************************************************************************"
	@echo "                                     Note"
	@echo "********************************************************************************"
	@echo ""
	@sed -n 's/^#NOTE#//p' ${MAKEFILE_LIST} | column -t -s ':' | sed 's/^//'
	@echo ""

all: debug
##? all: Build binaries .elf, .hex, .bin, default is debug

debug: ${BUILD_DIR}/debug cmake_debug
##? debug: Build binary in debug mode
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
##? release: Build binary in release mode
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

# Formats all user modified source files (add ones that are missing)
SRCS := $(shell find ${PROJECT_DIR} -name '*.[ch]' -or -name '*.[ch]pp') Core/Src/main.c
format: $(addsuffix .format,$(SRCS))
%.format: %
	clang-format -i $<

# Formats all CubeMX generated sources to unix style - removes \r from line
# endings
HIDDEN_FILES := .mxproject .project .cproject
FOUND_HIDDEN_FILES := $(shell \
    for f in $(HIDDEN_FILES); do \
        if [[ -e $$f ]]; then \
            echo $$f; \
        fi; \
    done)

# maybe add `drivers` directory as well?
FORMAT_LINUX := $(shell \
    find application -name '*' -type f; \
    find . -name '*.ioc') \
    $(FOUND_HIDDEN_FILES)

format-linux: $(addsuffix .format-linux,$(FORMAT_LINUX))
##? format-linux: format \r\n to \n

%.format-linux: %
	$(if $(filter $(PLATFORM),Linux),dos2unix -q $<,)

flash-debug: debug
##? flash-debug: Flash binary debug image into MCU
	@st-flash --reset write $(FIRMWARE_DEBUG) 0x08000000

flash-release: release
##? flash-release: Flash binary release image into MCU
	@st-flash --reset write $(FIRMWARE_RELEASE) 0x08000000

clean-debug:
##? clean-debug: Clean debug build directory
	@echo "[CLEANING-DEBUG]"
	@cd $(BUILD_DIR)/debug; make clean --no-print-directory

clean-debug-full:
##? clean-debug-full: Clean debug build directory and all cmake generated
##? : files
	@echo "[CLEANING-DEBUG]"
	@rm -rf ./build_cmake/debug/*

clean-release:
##? clean-release: Clean release build directory
	@echo "[CLEANING-RELEASE]"
	@cd $(BUILD_DIR)/release; make clean --no-print-directory

clean-release-full:
##? clean-release-full: Clean release build directory and all cmake generated
##? : files
	@echo "[CLEANING-RELEASE]"
	@rm -rf ./build_cmake/release/*

clean-cmake-cache:
##? clean-cmake-cache: Clean cmake cache
	rm -rf ./build_cmake/debug/CMakeFiles
	rm -f ./build_cmake/debug/cmake_install.cmake
	rm -f ./build_cmake/debug/CMakeCache.txt
	rm -f ./build_cmake/debug/Makefile
	rm -rf ./build_cmake/release/CMakeFiles
	rm -f ./build_cmake/release/cmake_install.cmake
	rm -f ./build_cmake/release/CMakeCache.txt
	rm -f ./build_cmake/release/Makefile

com:
##? com: minicom -b 115200 -o -D /dev/ttyACM0, ctrl+a, q to
##? : quit
	minicom -b 115200 -o -D /dev/ttyACM0


################################################################################
# Container related
################################################################################
podman-build-image: build_podman/Containerfile
##? podman-build-image: Build container image for building the app
	./build_podman/podman_build.sh

podman-run-container:
##? podman-run-container: Run build container in interactive mode with bash
##? : shell
	./build_podman/podman_run.sh --it

podman-build-release:
##? podman-build-release: Build binary in release mode inside a container
	./build_podman/podman_run.sh --release

podman-build-debug:
##? podman-build-debug: Build binary in debug mode inside a container
	./build_podman/podman_run.sh --debug


#NOTE# : You should run `clean-release-full` and `clean-debug-full` only when
#NOTE# : switching from building with CMake on your system to building inside
#NOTE# : a container. Both methods use the same CMake build directory for output.