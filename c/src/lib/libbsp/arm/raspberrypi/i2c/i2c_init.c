/**
 * @file i2c_init.c
 *
 * @ingroup raspberrypi_i2c
 *
 * @brief Raspberry Pi I2C bus initialization.
 */

/*
 *  Copyright (c) 2014 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/raspberrypi.h>
#include <bsp/gpio.h>
#include <bsp/i2c.h>

#include <libchip/mcp23008.h>

static rtems_libi2c_bus_ops_t bcm2835_i2c_ops = {
  init:             bcm2835_i2c_init,
  send_start:       bcm2835_i2c_send_start,
  send_stop:        bcm2835_i2c_stop,
  send_addr:        bcm2835_i2c_send_addr,
  read_bytes:       bcm2835_i2c_read_bytes,
  write_bytes:      bcm2835_i2c_write_bytes,
  ioctl:            bcm2835_i2c_ioctl
};

static bcm2835_i2c_desc_t bcm2835_i2c_bus_desc = {
  {
    ops:            &bcm2835_i2c_ops,
    size:           sizeof(bcm2835_i2c_bus_desc)
  },
  {
    initialized:    0
  }
};

/* Register drivers here for all the devices 
 * which require access to the I2C bus.
 *
 * The libi2c function "rtems_libi2c_register_drv" must be used to 
 * register each device driver, using the received i2c bus number. 
 *
 * This function returns 0 on success. */
int BSP_i2c_register_drivers(int i2c_bus_number)
{
  int rv = 0;

  rv = rtems_libi2c_register_drv("mcp23008", &i2c_mcp23008_drv_t, i2c_bus_number, MCP23008_ADDR);

  return rv;
}

int BSP_i2c_init(void)
{
  int rv;

  /* Initialize the libi2c API. */
  rtems_libi2c_initialize ();

  /* Enable the I2C interface on the Raspberry Pi P1 GPIO header. */
  gpio_initialize ();

  if ( gpio_select_i2c_p1_rev2() < 0 )
    return RTEMS_RESOURCE_IN_USE;

  /* Register the I2C bus. */
  rv = rtems_libi2c_register_bus("/dev/i2c", &(bcm2835_i2c_bus_desc.bus_desc));

  if ( rv < 0 )
    return -rv;
  
  /* Register SPI device drivers. */
  rv =  BSP_i2c_register_drivers(rv);

  return 0;
}
