/*
 * Description: Wrapper for AS5600 magnetic encoder driver functionality
 *
 * Created at: 07/10/2023
 * 
 * Main driver source: https://github.com/hepingood/as5600
 * 
 * GPIOs used: PB8 for i2c1 scl (alias I2C1_SCL) 
 *             PB9 for i2c1 sda (alias I2C1_SDA)
 *
 * The code is almost the same as in driver_as5600_basic.c/h
 * 
 */

#include "pend_enc_driver.h"

static as5600_handle_t gs_handle;        /**< as5600 handle */

static uint16_t angle_raw;

uint8_t pend_enc_init(void)
{
    uint8_t res;
    
     /* link interface function */
    DRIVER_AS5600_LINK_INIT(&gs_handle, as5600_handle_t);
    DRIVER_AS5600_LINK_IIC_INIT(&gs_handle, as5600_interface_iic_init);
    DRIVER_AS5600_LINK_IIC_DEINIT(&gs_handle, as5600_interface_iic_deinit);
    DRIVER_AS5600_LINK_IIC_READ(&gs_handle, as5600_interface_iic_read);
    DRIVER_AS5600_LINK_IIC_WRITE(&gs_handle, as5600_interface_iic_write);
    DRIVER_AS5600_LINK_DELAY_MS(&gs_handle, as5600_interface_delay_ms);
    DRIVER_AS5600_LINK_DEBUG_PRINT(&gs_handle, as5600_interface_debug_print);
    
    res = as5600_init(&gs_handle);
    if (res != 0)
    {
        as5600_interface_debug_print("as5600: init failed.\n");
       
        return 1;
    }

    as5600_set_start_position(&gs_handle, (uint16_t)0);
    uint16_t start_pos; 
    as5600_get_start_position(&gs_handle, &start_pos);
    as5600_interface_debug_print("start position: %u\n", start_pos);

    as5600_set_stop_position(&gs_handle, (uint16_t)4094);
    uint16_t stop_pos; 
    as5600_get_stop_position(&gs_handle, &stop_pos);
    as5600_interface_debug_print("stop position: %u\n", stop_pos);
    return 0;
}

uint8_t pend_enc_read_angle_deg(float *angle)
{
    /* Writes angle value in degree into &angle */
    as5600_read(&gs_handle, &angle_raw, angle);

    return 0;
}

uint8_t pend_enc_read_angle_rad(float *angle)
{
    /* Writes raw angle value into &angle_raw */
    as5600_get_raw_angle(&gs_handle, &angle_raw);

    *angle = (float)angle_raw / 4096.0f * PI2;
    
    return 0;
}

uint8_t pend_enc_deinit(void)
{
    if (as5600_deinit(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
