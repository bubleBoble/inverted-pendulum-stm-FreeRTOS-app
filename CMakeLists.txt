# STM32 Minimal CMake project for C/C++ projects
cmake_minimum_required(VERSION 3.12)

# Include toolchain cmake file
include(${CMAKE_CURRENT_SOURCE_DIR}/gcc-arm-none-eabi.cmake)

# Project name and main project source code folder location,
# PROJECT_NAME can be set in makefile
if ("${PROJECT_NAME}" STREQUAL "")
    set(PROJECT_NAME firmware) # LIP_app_1_f429zi
endif()
project(${PROJECT_NAME})

# Set debug build as default
if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE Debug)
endif()
message("Build type: ${CMAKE_BUILD_TYPE}")

# MCU info
set(MCU_FAMILY stm32f4xx)
set(MCU_MODEL STM32F429xx)
set(MCU_PARAMETERS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard)

# Startup and linker scripts
set(STARTUP_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f429xx.s)
set(MCU_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32F429ZITx_FLASH.ld)

# Executable
set(EXECUTABLE ${CMAKE_PROJECT_NAME})

# Language, standards and extensions
enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
set(CMAKE_CXX_EXTENSIONS ON)

# Includes
set(ST_DRIVERS_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Inc/Legacy)

set(CMSIS_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/cmsis-device-f4/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/CMSIS_5/CMSIS/Core/Include)

set(FREERTOS_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS-CLI)

set(PROJECT_DRIVERS_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/project_drivers/as5600_driver/src
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/project_drivers/as5600_driver/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/project_drivers/as5600_driver/example)

set(ST_CORE_INCLUDES
    ${PROJECT_DIR}/STCore/Inc)

set(PROJECT_INCLUDES
    ${PROJECT_DIR}/LIP/include)

# Sources
set(ST_DRIVERS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_adc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_ll_adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_i2c_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/${MCU_FAMILY}-hal-driver/Src/stm32f4xx_hal_uart.c)

set(FREERTOS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/croutine.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/event_groups.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/list.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/stream_buffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/timers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/portable/MemMang/heap_4.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/RTOS/FreeRTOS-CLI/FreeRTOS_CLI.c)

set(ST_CORE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/stm32f4xx_hal_timebase_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application/STCore/Src/system_stm32f4xx.c)

set(PROJECT_DRIVERS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/project_drivers/as5600_driver/src/driver_as5600.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/project_drivers/as5600_driver/interface/stm32f429_driver_as5600_interface.c
    ${CMAKE_CURRENT_SOURCE_DIR}/application_framework/project_drivers/as5600_driver/example/driver_as5600_basic.c)

set(PROJECT_SOURCES
    ${PROJECT_DIR}/${PROJECT_NAME}/source/cli_commands.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/com_driver.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/dcm_encoder_driver.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/FIR_filter.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/IIR_filter.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_bounceoff.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_cartWorker.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_communication.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_console.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_ctrl_downposition.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_ctrl_upposition.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_raw_communication.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_tasks_common.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_swingdown.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_swingup.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_test.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_util.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LIP_task_watchdog.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/LP_filter.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/main_LIP.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/motor_driver.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/pend_enc_driver.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/printf_reroute.c
    ${PROJECT_DIR}/${PROJECT_NAME}/source/swingup_input_voltage_lookup_table.c)

# Executables
add_executable(${EXECUTABLE}
               ${ST_DRIVERS_SOURCES}
               ${FREERTOS_SOURCES}
               ${ST_CORE_SOURCES}
               ${PROJECT_DRIVERS_SOURCES}
               ${PROJECT_SOURCES}
               ${STARTUP_SCRIPT})

# Defines
target_compile_definitions(${EXECUTABLE} PRIVATE
    $<$<CONFIG:Debug>:DEBUG> # add DEBUG to the compile definitions if current config is debug
    ${MCU_MODEL}
    USE_HAL_DRIVER)

# Add includes
# System headers: Any warnings from those headers will be suppressed - useful 
# for third-party or vendor libraries
target_include_directories(${EXECUTABLE} SYSTEM PRIVATE
                           ${ST_CORE_INCLUDES}
                           ${ST_DRIVERS_INCLUDES}
                           ${CMSIS_INCLUDES})

# Warnings from these headers will be visible
target_include_directories(${EXECUTABLE} PRIVATE
                           ${FREERTOS_INCLUDES}
                           ${PROJECT_DRIVERS_INCLUDES}
                           ${PROJECT_INCLUDES})

# Compile flags
# Note: Some flags are more "toolchain specific" so those are specified in 
# gcc-arm-none-eabi.cmake
target_compile_options(${EXECUTABLE} PRIVATE
    ${MCU_PARAMETERS}
    -Wall
    -Wdouble-promotion
    -Wshadow
    # -Wextra
    # -Werror
    -Wformat=2 -Wformat-truncation
    -pedantic
    -fsingle-precision-constant
    -ffast-math
    $<$<COMPILE_LANGUAGE:CXX>:
        -Wconversion # STM libraries
        -Wno-volatile
        -Wold-style-cast
        -Wuseless-cast
        -Wsuggest-override>
    $<$<CONFIG:Debug>:-O0 -g3 -ggdb>
    $<$<CONFIG:Release>:-Og -g0>)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
    ${MCU_PARAMETERS}
    -T${MCU_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    $<$<VERSION_GREATER:$<C_COMPILER_VERSION>,10.3.1>:-Wl,--no-warn-rwx-segments>
    -Wl,--start-group
    -lc
    -lm
    # -lstdc++
    -Wl,--end-group
    -Wl,--print-memory-usage)

add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> 
    ${EXECUTABLE}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> 
    ${EXECUTABLE}.bin)

if (${DUMP_ASM})
    add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
        COMMAND ${CMAKE_OBJDUMP} -D -C $<TARGET_FILE:${EXECUTABLE}> > ${EXECUTABLE}.s)
endif()
