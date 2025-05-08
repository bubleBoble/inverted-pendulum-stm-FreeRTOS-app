/*
 *
 *    Driver source: https://github.com/hepingood/as5600#Usage
 *
 */

#include "driver_as5600_interface.h"
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include <stdarg.h>

uint8_t as5600_interface_iic_init(void)
{
    // tutaj można dać kod taki jak init z hala
    // MX_I2C1_Init();
    // HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle);
    // albo zgodnie z docsami hala
    return 0;
}

uint8_t as5600_interface_iic_deinit(void)
{
    // HAL_I2C_MspDeInit(&hi2c1);
    return 0;
}

uint8_t as5600_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    /*   
     *    Interfejs do drivera przez Hala:
     *        hi2c1 - handle to hal i2c struct
     *        1     - internal memory unit size
     *        100   - timeout
     */
    HAL_I2C_Mem_Read(&hi2c1, addr, reg, 1, buf, len, 1000);
    // HAL_I2C_Master_Receive(&hi2c1, addr, buf, len, 1000);
    return 0;
}

uint8_t as5600_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_I2C_Mem_Write(&hi2c1, addr, reg, 1, buf, len, 1000);
    return 0;
}

void as5600_interface_delay_ms(uint32_t ms)
{
    // delay from FreeRTOS
    vTaskDelay(ms);
}

void as5600_interface_debug_print(const char *const fmt, ...)
{
    char str[256];
    uint16_t len;
    va_list args;
    
    memset((char *)str, 0, sizeof(char) * 256); 
    va_start(args, fmt);
    vsnprintf((char *)str, 255, (char const *)fmt, args);
    va_end(args);
    
    len = strlen((char *)str);
    HAL_UART_Transmit(&huart3, (uint8_t *)str, len, 500);
}
