.PHONY: all build cmake clean format

BUILD_DIR := build_cmake
BUILD_TYPE ?= Debug

all: build

build: cmake
	$(MAKE) -C ${BUILD_DIR} --no-print-directory

cmake: ${BUILD_DIR}/Makefile

${BUILD_DIR}/Makefile: CMakeLists.txt
	cmake \
		-B${BUILD_DIR} \
		-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON

clean:
	@echo "[CLEANING]"
	@ cd $(BUILD_DIR); \
	make clean --no-print-directory