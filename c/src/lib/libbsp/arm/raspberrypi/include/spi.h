/**
 * @file spi.h
 *
 * @ingroup raspberrypi_spi
 *
 * @brief Raspberry Pi specific SPI definitions.
 */

/*
 *  Copyright (c) 2014-2015 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_RASPBERRYPI_SPI_H
#define LIBBSP_ARM_RASPBERRYPI_SPI_H

#include <rtems/libi2c.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @name SPI constants.
 *
 * @{
 */

/**
 * @brief GPU processor core clock rate in Hz.
 *
 * Unless configured otherwise on a "config.txt" file present on the SD card
 * the GPU defaults to 250 MHz. Currently only 250 MHz is supported.
 */

/* TODO: It would be nice if this value could be probed at startup, probably
 *       using the Mailbox interface since the usual way of setting this on
 *       the hardware is through a "config.txt" text file on the SD card.
 *       Having this setup on the configure.ac script would require changing
 *       the same setting on two different places. */
#define GPU_CORE_CLOCK_RATE 250000000

/** @} */

/**
 * @name SPI data structures.
 *
 * @{
 */

/**
 * @brief Object containing the SPI bus configuration settings.
 *
 * Encapsulates the current SPI bus configuration.
 */
typedef struct
{
  int initialized;
  uint8_t bytes_per_char;

  /* Shift to be applied on data transfers with
   * least significative bit first (LSB) devices. */
  uint8_t bit_shift;
  uint32_t dummy_char;

  /* If set to 0 uses 3-wire SPI, with 2 separate data lines (MOSI and MISO),
   * if set to 1 uses 2-wire SPI, where the MOSI data line doubles as the
   * slave out (SO) and slave in (SI) data lines. */
  int bidirectional;
  uint32_t current_slave_addr;
  rtems_id irq_sema_id;
  int irq_write;
} bcm2835_spi_softc_t;

typedef struct
{
  rtems_libi2c_bus_t bus_desc;
  bcm2835_spi_softc_t softc;
} bcm2835_spi_desc_t;

/** @} */

/**
 * @name SPI directives.
 *
 * @{
 */

rtems_status_code bcm2835_spi_init(rtems_libi2c_bus_t *bushdl);

rtems_status_code bcm2835_spi_send_start(rtems_libi2c_bus_t *bushdl);

rtems_status_code bcm2835_spi_stop(rtems_libi2c_bus_t *bushdl);

rtems_status_code
bcm2835_spi_send_addr(rtems_libi2c_bus_t *bushdl, uint32_t addr, int rw);

int bcm2835_spi_read_bytes(
  rtems_libi2c_bus_t *bushdl,
  unsigned char *bytes,
  int nbytes
);

int bcm2835_spi_write_bytes(
  rtems_libi2c_bus_t *bushdl,
  unsigned char *bytes,
  int nbytes
);

int bcm2835_spi_ioctl(rtems_libi2c_bus_t *bushdl, int cmd, void *arg);

void rpi_spi_init(void);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_RASPBERRYPI_SPI_H */
