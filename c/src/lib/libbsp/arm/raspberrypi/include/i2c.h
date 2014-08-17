/**
 * @file i2c.h
 *
 * @ingroup raspberrypi_i2c
 *
 * @brief Raspberry Pi specific I2C and SPI definitions.
 */

/*
 *  Copyright (c) 2014 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_RASPBERRYPI_I2C_H
#define LIBBSP_ARM_RASPBERRYPI_I2C_H

#include <rtems/libi2c.h>

/**
 * @name SPI constants.
 *
 * @{
 */

/**
 * @brief  GPU processor core clock rate in Hz. 
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
 * @name  SPI data structures.
 *
 * @{
 */

/**
 * @brief Object containing the SPI bus configuration settings.
 *
 * Encapsulates the current SPI bus configuration.
 */
typedef struct {
  int                 initialized;
  uint8_t             bytes_per_char;

  /* Shift to be applied on data transfers with
   * least significative bit first (LSB) devices. */
  uint8_t             bit_shift; 
  uint32_t            dummy_char;

  /* If set to 0 uses 3-wire SPI, with 2 separate data lines (MOSI and MISO),
   * if set to 1 uses 2-wire SPI, where the MOSI data line doubles as the
   * slave out (SO) and slave in (SI) data lines. */
  int                 bidirectional; 
  uint32_t            current_slave_addr;
  rtems_id            irq_sema_id;
  int                 irq_write;
} bcm2835_spi_softc_t;

typedef struct {
  rtems_libi2c_bus_t  bus_desc;
  bcm2835_spi_softc_t softc;
} bcm2835_spi_desc_t;

/** @} */

/**
 * @name  SPI directives.
 *
 * @{
 */

rtems_status_code bcm2835_spi_init(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_spi_send_start(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_spi_stop(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_spi_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw);

int bcm2835_spi_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_spi_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_spi_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg);

int BSP_spi_register_drivers(int spi_bus_number);

int BSP_spi_init(void);

/** @} */

/**
 * @name  I2C constants.
 *
 * @{
 */


/**
 * @brief  BSC controller core clock rate in Hz. 
 *
 * This is set to 150 MHz as per the BCM2835 datasheet.
 */
#define BSC_CORE_CLK_HZ 150000000

/** @} */

/**
 * @name  I2C data structures.
 *
 * @{
 */

typedef struct {
  int                 initialized;
  rtems_id            irq_sema_id;
} bcm2835_i2c_softc_t;

typedef struct {
  rtems_libi2c_bus_t  bus_desc;
  bcm2835_i2c_softc_t softc;
} bcm2835_i2c_desc_t;

/** @} */

/**
 * @name  I2C directives.
 *
 * @{
 */

rtems_status_code bcm2835_i2c_init(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_i2c_send_start(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_i2c_stop(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_i2c_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw);

int bcm2835_i2c_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_i2c_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_i2c_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg);

int BSP_i2c_register_drivers(int i2c_bus_number);

int BSP_i2c_init(void);

/** @} */

#endif /* LIBBSP_ARM_RASPBERRYPI_I2C_H */
