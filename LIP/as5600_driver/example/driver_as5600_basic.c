#include "driver_as5600_basic.h"

static as5600_handle_t gs_handle;        /**< as5600 handle */

uint8_t as5600_basic_init(void)
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
    
    /* as5600 init */
    res = as5600_init(&gs_handle);
    if (res != 0)
    {
        as5600_interface_debug_print("as5600: init failed.\n");
       
        return 1;
    }

    ///////////////////////////////////////////////
    // as5600_set_start_position(&gs_handle, 0);
    // as5600_set_max_angle(&gs_handle, 360);
    ///////////////////////////////////////////////

    return 0;
}

uint8_t as5600_basic_read(float *angle)
{
    uint8_t res;
    uint16_t angle_raw;
    
    /* read data */
    res = as5600_read(&gs_handle, &angle_raw, angle);
    if (res != 0)
    {
        as5600_interface_debug_print("as5600: read failed.\n");
       
        return 1;
    }
    
    return 0;
}

uint8_t as5600_basic_deinit(void)
{
    /* close as5600 */
    if (as5600_deinit(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
