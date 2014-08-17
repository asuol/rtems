/**
 * @file spi_init.c
 *
 * @ingroup raspberrypi_i2c
 *
 * @brief Raspberry Pi SPI bus initialization.
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

#include <libchip/23k256.h>

static rtems_libi2c_bus_ops_t bcm2835_spi_ops = {
  init:        bcm2835_spi_init,
  send_start:  bcm2835_spi_send_start,
  send_stop:   bcm2835_spi_stop,
  send_addr:   bcm2835_spi_send_addr,
  read_bytes:  bcm2835_spi_read_bytes,
  write_bytes: bcm2835_spi_write_bytes,
  ioctl:       bcm2835_spi_ioctl
};

static bcm2835_spi_desc_t bcm2835_spi_bus_desc = {
  {
    ops:  &bcm2835_spi_ops,
    size: sizeof(bcm2835_spi_bus_desc)
  },
  {
    initialized:    0
  }
};

/* Register drivers here for all the devices 
 * which require access to the SPI bus.
 *
 * The libi2c function "rtems_libi2c_register_drv" must be used to 
 * register each device driver, using the received spi bus number. 
 *
 * This function returns 0 on success. */
int BSP_spi_register_drivers(int spi_bus_number)
{
  int rv = 0;

  rv = rtems_libi2c_register_drv("23k256", &spi_23k256_rw_drv_t, spi_bus_number, 0x00);

  return rv;
}

int BSP_spi_init(void)
{
  int rv;

  /* Initialize the libi2c API. */
  rtems_libi2c_initialize ();

  /* Enable the SPI interface on the Raspberry Pi P1 GPIO header. */
  gpio_initialize ();

  if ( gpio_select_spi_p1() < 0 )
    return RTEMS_RESOURCE_IN_USE;

  /* Clear SPI control register and clear SPI FIFOs. */
  BCM2835_REG(BCM2835_SPI_CS) = 0x0000030;

  /* Register the SPI bus. */
  rv = rtems_libi2c_register_bus("/dev/spi", &(bcm2835_spi_bus_desc.bus_desc));

  if ( rv < 0 )
    return rv;
  
  /* Register SPI device drivers. */
  rv =  BSP_spi_register_drivers(rv);

  return rv;
}


