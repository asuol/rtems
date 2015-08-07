/**
 * @file spi_init.c
 *
 * @ingroup raspberrypi_spi
 *
 * @brief Raspberry Pi SPI bus initialization.
 */

/*
 *  Copyright (c) 2014-2015 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/raspberrypi.h>
#include <bsp/gpio.h>
#include <bsp/rpi-gpio.h>
#include <bsp/spi.h>
#include <assert.h>

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

void rpi_spi_init(void)
{
  int rv;

  /* Initialize the libi2c API. */
  rtems_libi2c_initialize ();

  /* Enable the SPI interface on the Raspberry Pi. */
  rtems_gpio_initialize ();

  assert ( rpi_gpio_select_spi() == RTEMS_SUCCESSFUL );

  /* Clear SPI control register and clear SPI FIFOs. */
  BCM2835_REG(BCM2835_SPI_CS) = 0x0000030;

  /* Register the SPI bus. */
  rv = rtems_libi2c_register_bus("/dev/spi", &(bcm2835_spi_bus_desc.bus_desc));

  assert ( rv == 0 );
}


