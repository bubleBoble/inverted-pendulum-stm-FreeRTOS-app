#ifndef DRIVER_AS5600_BASIC_H
#define DRIVER_AS5600_BASIC_H

#include "driver_as5600_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

uint8_t as5600_basic_init(void);

uint8_t as5600_basic_read(float *angle);

uint8_t as5600_basic_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
