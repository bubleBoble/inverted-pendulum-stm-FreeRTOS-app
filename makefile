# Makefile based on: https://github.com/prtzl/stm32/blob/master/Makefile
.PHONY: help all debug cmake_debug release cmake_release format-linux flash-debug flash-release clean-debug clean-release

PROJECT_NAME ?= firmware
BUILD_TYPE ?= Debug
BUILD_DIR ?= build_cmake
FIRMWARE_DEBUG := $(BUILD_DIR)/debug/$(PROJECT_NAME).bin
FIRMWARE_RELEASE := $(BUILD_DIR)/release/$(PROJECT_NAME).bin
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
	@st-flash --reset write $(FIRMWARE_DEBUG) 0x08000000

flash-release: release
##? flash-release: Flash binary release image into MCU
	@st-flash --reset write $(FIRMWARE_RELEASE) 0x08000000

clean-debug:
##? clean-debug: Clean debug build directory
	@echo "[CLEANING-DEBUG]"
	@cd $(BUILD_DIR)/debug; make clean --no-print-directory

clean-release:
##? clean-release: Clean release build directory
	@echo "[CLEANING-RELEASE]"
	@cd $(BUILD_DIR)/release; make clean --no-print-directory

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
##? com: minicom -b 115200 -o -D /dev/ttyACM0, ctrl+a, q to quit
	minicom -b 115200 -o -D /dev/ttyACM0


###############################################################################
# Container related
###############################################################################
UID ?= $(shell id -u)
GID ?= $(shell id -g)
USER ?= $(shell id -un)
GROUP ?= $(if $(filter $(PLATFORM), Windows_NT),$(shell id -un),$(shell id -gn))

CONTAINER_FILE ?= build_podman/Containerfile
IMAGE_NAME := fedora-stm-build-base
IMAGE_VERSION := v1.0
CONTAINER_NAME := fedora-stm-build-base

podman-build-image: $(CONTAINER_FILE)
##? podman-build-image: Build container image for building the app
	podman build \
			--tag "$(IMAGE_NAME):$(IMAGE_VERSION)" \
			--file=$(CONTAINER_FILE) \
			--build-arg UID=$(UID) \
			--build-arg GID=$(GID) \
			--build-arg USERNAME=$(USER) \
			--build-arg GROUPNAME=$(GROUP) \
			./build_podman

REMOVE_AFTER_BUILD = true
CONTAINER_RUN = podman run \
						--name $(CONTAINER_NAME) \
						--rm=$(REMOVE_AFTER_BUILD) \
						--userns=keep-id \
						--volume $$(pwd):/workdir \
						--workdir /workdir \
						--security-opt label=disable \
						--hostname $(CONTAINER_NAME)

podman-run-container:
##? podman-run-container: Run build container in interactive mode with bash shell
	$(CONTAINER_RUN) \
			--interactive \
			--tty \
			"$(IMAGE_NAME):$(IMAGE_VERSION)" \
			bash

podman-build-release:
##? podman-build-release: Build binary in release mode inside a container
	rm -rf ./build_cmake/release/*
	$(CONTAINER_RUN) \
			"$(IMAGE_NAME):$(IMAGE_VERSION)" \
			bash -lc "make release -j"

podman-build-debug:
##? podman-build-debug: Build binary in debug mode inside a container
	rm -rf ./build_cmake/debug/*
	$(CONTAINER_RUN) \
			"$(IMAGE_NAME):$(IMAGE_VERSION)" \
			bash -lc "make debug -j"
