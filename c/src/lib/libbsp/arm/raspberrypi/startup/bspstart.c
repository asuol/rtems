/**
 * @file bspstart.c
 *
 * @ingroup arm_start
 *
 * @brief Raspberry pi startup code.
 */

/*
 *  Copyright (c) 2014 Andre Marques <andre.lousa.marques at gmail.com>
 *  Copyright (c) 2013 by Alan Cudmore
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *
 *  http://www.rtems.org/license/LICENSE
 */

#include <bsp.h>
#include <bsp/bootcard.h>
#include <bsp/irq-generic.h>
#include <bsp/irq.h>
#include <bsp/linker-symbols.h>
#include <bsp/stackalloc.h>
#include <bsp/raspberrypi.h>
#include <bsp/i2c.h>

void bsp_predriver_hook(void)
{
  if ( BSP_ENABLE_SPI == 1 )
    BSP_spi_init();

  if ( BSP_ENABLE_I2C == 1 )
    BSP_i2c_init();
}

void bsp_start(void)
{
  bsp_interrupt_initialize();
}
