/*
 *
 *    Driver source: https://github.com/hepingood/as5600#Usage
 *
 */

#ifndef DRIVER_AS5600_INTERFACE_H
#define DRIVER_AS5600_INTERFACE_H

#include "driver_as5600.h"

#ifdef __cplusplus
extern "C"{
#endif

uint8_t as5600_interface_iic_init(void);

uint8_t as5600_interface_iic_deinit(void);

uint8_t as5600_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t as5600_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

void as5600_interface_delay_ms(uint32_t ms);

void as5600_interface_debug_print(const char *const fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
