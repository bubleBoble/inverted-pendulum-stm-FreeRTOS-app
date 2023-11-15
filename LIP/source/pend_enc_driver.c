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

static as5600_handle_t gs_handle;

static uint16_t angle_raw = 0;
static int32_t cumulative_count = 0;
static uint16_t last_count = 0;
static int32_t num_of_revolutions = 0;

uint8_t pend_enc_init( void )
{
    uint8_t res;
    
     /* link interface function */
    DRIVER_AS5600_LINK_INIT( &gs_handle, as5600_handle_t );
    DRIVER_AS5600_LINK_IIC_INIT( &gs_handle, as5600_interface_iic_init );
    DRIVER_AS5600_LINK_IIC_DEINIT( &gs_handle, as5600_interface_iic_deinit );
    DRIVER_AS5600_LINK_IIC_READ( &gs_handle, as5600_interface_iic_read );
    DRIVER_AS5600_LINK_IIC_WRITE( &gs_handle, as5600_interface_iic_write );
    DRIVER_AS5600_LINK_DELAY_MS( &gs_handle, as5600_interface_delay_ms );
    DRIVER_AS5600_LINK_DEBUG_PRINT( &gs_handle, as5600_interface_debug_print );
    
    res = as5600_init( &gs_handle );
    if ( res != 0 )
    {
        as5600_interface_debug_print( "as5600: init failed.\n" );
       
        return 1;
    }
    return 0;
}

uint8_t pend_enc_read_angle_deg( float *angle )
{
    /* Writes angle value in degree into &angle */
    as5600_read( &gs_handle, &angle_raw, angle );

    return 0;
}

uint8_t pend_enc_read_angle_rad( float *angle )
{
    /* Writes raw angle value into &angle_raw */
    as5600_get_raw_angle( &gs_handle, &angle_raw );

    *angle = ( float ) angle_raw * 0.001533980788; // 0.001533980788 = 1 / 4096.0f * PI2;
    
    return 0;
}

uint8_t pend_enc_deinit( void )
{
    if ( as5600_deinit( &gs_handle ) != 0 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/* Have to be called often enough, at least 3 times per revolution (from some arduino library) */
int32_t pend_enc_get_cumulative_count( void )
{
    // as5600_read_raw_fast(&gs_handle, &angle_raw);
    as5600_get_raw_angle(&gs_handle, &angle_raw);
    if ( ( last_count > 2048 ) && ( angle_raw < (last_count - 2048) ) )
    {
        cumulative_count = cumulative_count + 4096 - last_count + angle_raw;
        num_of_revolutions += 1;
    }
    else if ( ( angle_raw > 2048 ) && ( last_count < ( angle_raw - 2048 ) ) )
    {
        cumulative_count = cumulative_count - 4096 - last_count + angle_raw;
        num_of_revolutions -= 1;
    }
    else
    {
        cumulative_count = cumulative_count - last_count + angle_raw;
    }
    last_count = angle_raw;
    
    return cumulative_count;
}

/* Get number of full pendulum revolutions, negative number indicates negative revolution. */
int32_t get_num_of_revolutions( void )
{
    return num_of_revolutions;
}