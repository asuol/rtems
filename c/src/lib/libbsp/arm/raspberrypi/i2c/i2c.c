/**
 * @file i2c.c
 *
 * @ingroup raspberrypi_i2c
 *
 * @brief Support for the I2C bus on the Raspberry Pi GPIO P1 header (model A/B)
 * and GPIO J8 header on model B+.
 */

/*
 *  COPYRIGHT (c) 2014 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

/* TODO:
 * - Clock stretching (using default values)
 * - 10-bit addressing
 * - Falling/Rising edge delays (using default values)
 */

#include <bsp.h>
#include <bsp/raspberrypi.h>
#include <bsp/gpio.h>
#include <bsp/irq.h>
#include <bsp/i2c.h>

/**
 * @brief Calculates a clock divider to be used with the BSC core clock rate
 *        to set a I2C clock rate the closest (<=) to a desired frequency.
 *
 * @param[in] clock_hz The desired clock frequency for the I2C bus operation.
 * @param[out] clock_divider Pointer to a variable where the calculated
 *                          clock divider will be stored.
 *
 * @retval RTEMS_SUCCESSFUL Successfully calculated a valid clock divider.
 * @retval RTEMS_INVALID_NUMBER The resulting clock divider is invalid, due to
 *                              an invalid BSC_CORE_CLOCK_HZ 
 *                              or clock_hz value.
 */
static rtems_status_code bcm2835_i2c_calculate_clock_divider(uint32_t clock_hz, uint16_t *clock_divider)
{
  uint16_t divider;
  uint32_t clock_rate;

  /* Calculates an initial clock divider. */
  divider = BSC_CORE_CLK_HZ / clock_hz;

  if ( divider < 0 || divider > 0xFFFF )
    return RTEMS_INVALID_NUMBER;

  clock_rate = BSC_CORE_CLK_HZ / divider;

  /* If the resulting clock rate is greater than desired, try the next greater divider. */
  while ( clock_rate > clock_hz )
  {
    divider++;

    clock_rate = BSC_CORE_CLK_HZ / divider;
  }

  *clock_divider = divider;

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Set the I2C bus clock divider.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] tfr_mode Pointer to a libi2c API transfer mode data structure.
 *
 * @retval RTEMS_SUCCESSFUL Successfully setup the bus transfer mode as desired.
 * @retval RTEMS_INVALID_NUMBER @see bcm2835_i2c_calculate_clock_divider().
 */
static rtems_status_code bcm2835_i2c_set_tfr_mode(rtems_libi2c_bus_t *bushdl, const rtems_libi2c_tfr_mode_t *tfr_mode)
{
  rtems_status_code sc;
  uint16_t clock_divider;

  /* Calculate the most appropriate clock divider. */
  sc = bcm2835_i2c_calculate_clock_divider(tfr_mode->baudrate, &clock_divider);
  
  if ( sc != RTEMS_SUCCESSFUL )
    return sc;

  /* Set clock divider. */
  BCM2835_REG(BCM2835_I2C_DIV) = clock_divider;

  return sc;
}

/**
 * @brief Reads/writes to/from the I2C bus.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] rd_buf Read buffer. If not NULL the function will read from 
 *                   the bus and store the read on this buffer.
 * @param[in] wr_buf Write buffer. If not NULL the function will write the 
 *                   contents of this buffer to the bus.
 * @param[in] buffer_size Size of the non-NULL buffer.
 *
 * @retval -1 Could not send/receive data to/from the bus.
 * @retval >=0 The number of bytes read/written.
 */
static int bcm2835_i2c_read_write(rtems_libi2c_bus_t * bushdl, unsigned char *rd_buf, const unsigned char *wr_buf, int buffer_size)
{
  bcm2835_i2c_softc_t *softc_ptr = &(((bcm2835_i2c_desc_t *)(bushdl))->softc);

  uint32_t bytes_sent = buffer_size;

  /* Since there is a maximum of 0xFFFF packets per transfer 
   * (size of the DLEN register), count how many transfers will be 
   * needed and adjust each transfer size accordingly. */
  int transfer_count = buffer_size / 0xFFFF; 
  uint16_t dlen_buffer_size;

  /* If the buffer size is a multiple of the max size per transfer, 
   * round up the transfer count. */
  if ( buffer_size % 0xFFFF != 0 )
    transfer_count++;

  do {
    if (transfer_count > 1)
      dlen_buffer_size = 0xFFFF;
    else
      dlen_buffer_size = (buffer_size & 0xFFFF);

    /* Set the DLEN register, which specifies how many data packets will be transferred. */
    BCM2835_REG(BCM2835_I2C_DLEN) = dlen_buffer_size;

    /* Clear the acknowledgment and clock stretching error status. */
    BCM2835_REG(BCM2835_I2C_S) |= (3 << 8);

    /* While there is data to transfer. */
    while ( dlen_buffer_size >= 1 ) {

      /* If writing. */
      if ( rd_buf == NULL ) {

        /* If transfer is not active, send start bit. */
        if( (BCM2835_REG(BCM2835_I2C_S) & (1 << 0)) == 0)
          BCM2835_REG(BCM2835_I2C_C) |= (1 << 7);

        /* If using the I2C bus in interrupt-driven mode. */
        if ( I2C_IO_MODE == 1 ) {

          /* Generate interrupts on the TXW bit condition. */
          BCM2835_REG(BCM2835_I2C_C) |= (1 << 9);

          if ( rtems_semaphore_obtain(softc_ptr->irq_sema_id, RTEMS_WAIT, 50) != RTEMS_SUCCESSFUL )
            return -1;
        }

        /* If using the bus in polling mode. */
        else 
          /* Poll TXW bit until there is space available to write. */
          while ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 2)) == 0 )
            ;
        
        /* Write data to the TX FIFO. */
        BCM2835_REG(BCM2835_I2C_FIFO) = (*(uint8_t *)wr_buf);

        wr_buf++;

        /* Check for acknowledgment or clock stretching errors. */
        if ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 8)) || (BCM2835_REG(BCM2835_I2C_S) & (1 << 9)) )
          return -1;
      }

      /* If reading. */
      else {
        /* Send start bit. Before any read a libi2c_send_addr call should be made signaling a read operation. */
        BCM2835_REG(BCM2835_I2C_C) |= (1 << 7);

        /* Check for an acknowledgment error. */
        if ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 8)) != 0)
          return -1;
        
        /* Poll RXD bit until there is data on the RX FIFO to read. */
        while ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 5)) == 0 ) 
          ;

        /* Read data from the RX FIFO. */
        (*(uint8_t *)rd_buf) = BCM2835_REG(BCM2835_I2C_FIFO) & 0xFF;

        rd_buf++;

        /* Check for acknowledgment or clock stretching errors. */
        if ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 8)) || (BCM2835_REG(BCM2835_I2C_S) & (1 << 9)) )
          return -1;
      }

      dlen_buffer_size--;
      transfer_count--;
      buffer_size--;
    }
  } while ( transfer_count > 0 );

  /* If using the I2C bus in interrupt-driven mode. */
  if ( I2C_IO_MODE == 1 ) {
    /* Generate interrupts on the DONE bit condition. */
    BCM2835_REG(BCM2835_I2C_C) |= (1 << 8);

    if ( rtems_semaphore_obtain(softc_ptr->irq_sema_id, RTEMS_WAIT, 50) != RTEMS_SUCCESSFUL )
      return -1;
  }

  /* If using the bus in polling mode. */
  else
    /* Poll DONE bit until data has been sent. */
    while ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 1)) == 0 )
      ;

  bytes_sent -= buffer_size;

  return bytes_sent;
}

/**
 * @brief Handler function that is called on any I2C interrupt. 
 *        There are 3 situations that can generate an interrupt:
 *        
 *        1. Transfer (read/write) complete;
 *        2. The TX FIFO has space for more data (during a write transfer);
 *        3. The RX FIFO is full.
 *
 *        Because the I2C FIFO has a 16 byte size, the 3. situation is not 
 *        as useful to many applications as knowing that at least 1 byte can
 *        be read from the RX FIFO. For that reason this information is
 *        got through polling the RXD bit even in interrupt-driven mode.
 *
 *        This leaves only 2 interrupts to be caught. At any given time
 *        when no I2C bus transfer is taking place no I2C interrupts are 
 *        generated, and they do they are only enabled one at a time: 
 *
 *        - When trying to write, the 2. interrupt is enabled to signal that 
 *          data can be written on the TX FIFO, avoiding data loss in case
 *          it is full. When caught the handler disables that interrupt from
 *          being generated and releases the irq semaphore, which will allow
 *          the transfer process to continue (by writing to the TX FIFO);
 *
 *        - When the transfer is done on Raspberry side, the 1. interrupt is
 *          enabled for the device to signal it has finished the transfer as
 *          well. When caught the handler disables that interrupt from being 
 *          generated and releases the irq semaphore, marking the end of the
 *          transfer.
 *
 * @param[in] arg Void pointer to the bus data structure.
 */
static void i2c_handler(void* arg)
{
  bcm2835_i2c_softc_t *softc_ptr = (bcm2835_i2c_softc_t *) arg;

  /* If the current enabled interrupt is on the TXW condition, disable it. */
  if ( (BCM2835_REG(BCM2835_I2C_C) & (1 << 9)) ) 
    BCM2835_REG(BCM2835_I2C_C) &= ~(1 << 9);

  /* If the current enabled interrupt is on the DONE condition, disable it. */
  else if ( (BCM2835_REG(BCM2835_I2C_C) & (1 << 8)) )
    BCM2835_REG(BCM2835_I2C_C) &= ~(1 << 8);

    /* Release the irq semaphore. */
    rtems_semaphore_release(softc_ptr->irq_sema_id);
}

/**
 * @brief Low level function to initialize the I2C bus. 
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 *
 * @retval RTEMS_SUCCESSFUL SPI bus successfully initialized.
 * @retval Any other status code @see rtems_semaphore_create() and 
 *         @see rtems_interrupt_handler_install().
 */
rtems_status_code bcm2835_i2c_init(rtems_libi2c_bus_t * bushdl)
{
  bcm2835_i2c_softc_t *softc_ptr = &(((bcm2835_i2c_desc_t *)(bushdl))->softc);
  rtems_status_code sc = RTEMS_SUCCESSFUL;

  if ( softc_ptr->initialized == 1 )
    return sc;

  softc_ptr->initialized = 1;

  /* Enable the I2C BSC interface. */
  BCM2835_REG(BCM2835_I2C_C) |= (1 << 15);

  /* If the access to the bus is configured to be interrupt-driven. */
  if ( I2C_IO_MODE == 1 ) {
    sc = rtems_semaphore_create(rtems_build_name('i','2','c','s'), 0, RTEMS_FIFO | RTEMS_SIMPLE_BINARY_SEMAPHORE, 0, &softc_ptr->irq_sema_id);

    sc = rtems_interrupt_handler_install(BCM2835_IRQ_ID_I2C, NULL, RTEMS_INTERRUPT_UNIQUE, (rtems_interrupt_handler) i2c_handler, softc_ptr);
  }

  return sc;
}

/**
 * @brief Low level function that would send a start condition over the I2C bus.
 *        Because of the way the BSC controller implements the I2C protocol, the
 *        start sequence is sent whenever appropriate in bcm2835_i2c_read_write.
 *        Instead this function clears the bus FIFOS before each new data
 *        transfer.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 *
 * @retval RTEMS_SUCCESSFUL
 */
rtems_status_code bcm2835_i2c_send_start(rtems_libi2c_bus_t * bushdl)
{
  /* Clear FIFOs. */
  BCM2835_REG(BCM2835_I2C_C) |= (3 << 4);

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Low level function that would send a stop condition over the I2C bus,
 *        however the BSC controller send this condition automatically when the
 *        DLEN (data length - the number of bytes to be transferred) register
 *        value reaches 0.
 *        For that reason, it is here just to satisfy, the libi2c API,
 *         which requires this function.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 *
 * @retval RTEMS_SUCCESSFUL
 */
rtems_status_code bcm2835_i2c_stop(rtems_libi2c_bus_t * bushdl)
{
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Low level function which addresses a I2C device.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] addr Address of a connected I2C device
 * @param[in] rw Defines the nature of the transfer which will take place with 
 *               the addressed device - 0 to write and 1 to read.
 *
 * @retval RTEMS_SUCCESSFUL The device has been successfully addressed.
 */
rtems_status_code bcm2835_i2c_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw)
{
  /* Address slave device. */
  BCM2835_REG(BCM2835_I2C_A) = addr;

  /* Set read/write bit. 
   * If writing. */
  if ( rw == 0 )
    BCM2835_REG(BCM2835_I2C_C) &= ~(1 << 0);

  /* If reading. */
  else
    BCM2835_REG(BCM2835_I2C_C) |= (1 << 0);

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Low level function that reads a number of bytes from the I2C bus 
 *        on to a buffer.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] bytes Buffer where the data read from the bus will be stored.
 * @param[in] nbytes Number of bytes to be read from the bus to the bytes buffer.
 *
 * @retval @see bcm2835_i2c_read_write().
 */
int bcm2835_i2c_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes)
{
  return bcm2835_i2c_read_write(bushdl, bytes, NULL, nbytes);
}

/**
 * @brief Low level function that writes a number of bytes from a buffer
 *        to the I2C bus.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] bytes Buffer with data to send through the bus.
 * @param[in] nbytes Number of bytes to be written from the bytes buffer 
                     to the bus.
 *
 * @retval @see bcm2835_i2c_read_write().
 */
int bcm2835_i2c_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes)
{
  return bcm2835_i2c_read_write(bushdl, NULL, bytes, nbytes);
}

/**
 * @brief Low level function that is used to perform ioctl 
 *        operations on the bus. Currently only setups
 *        the bus transfer mode, namely the bus clock divider.
 *        This function is used by the libi2c API.
 *
 * @param[in] bushdl Pointer to the libi2c API bus driver data structure.
 * @param[in] cmd IOCTL request command.
 * @param[in] arg Arguments needed to fulfill the requested IOCTL command.
 *
 * @retval -1 Unknown request command.
 * @retval >=0 @see bcm2835_i2c_set_tfr_mode().
 */
int bcm2835_i2c_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg)
{
  switch ( cmd ) {
    case RTEMS_LIBI2C_IOCTL_SET_TFRMODE:

      return bcm2835_i2c_set_tfr_mode(bushdl, (const rtems_libi2c_tfr_mode_t *)arg);

    default:
      return -1;
  }

  return 0;
}
