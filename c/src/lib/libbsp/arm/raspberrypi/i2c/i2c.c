#include <bsp.h>
#include <bsp/raspberrypi.h>
#include <bsp/gpio.h>
#include <bsp/irq.h>
#include <bsp/i2c.h>

#define BSC_CORE_CLK_HZ 150000000

/* NOT implemented:
 * - Clock stretching (using default values)
 * - 10-bit addressing
 * - Falling/Rising edge delays (using default values)
 */

static rtems_status_code bcm2835_i2c_calculate_clock_divider(uint32_t clock_hz, uint16_t *clock_divider)
{
  uint16_t divider;
  uint32_t clock_rate;

  /* Calculate the appropriate clock divider */
  divider = BSC_CORE_CLK_HZ / clock_hz;

  if ( divider < 0 || divider > 0xFFFF )
    return RTEMS_INVALID_NUMBER;

  clock_rate = BSC_CORE_CLK_HZ / divider;

  /* If the resulting clock rate is greater than desired, try the next lower divider  */
  while ( clock_rate > clock_hz )
  {
    divider++;

    clock_rate = BSC_CORE_CLK_HZ / divider;
  }

  *clock_divider = divider;

  return RTEMS_SUCCESSFUL;
}

/* Set the I2c bus clock speed */
static rtems_status_code bcm2835_i2c_set_tfr_mode(rtems_libi2c_bus_t *bushdl, const rtems_libi2c_tfr_mode_t *tfr_mode)
{
  uint16_t clock_divider;

  rtems_status_code sc = RTEMS_SUCCESSFUL;

  /* Calculate the most appropriate clock divider */
  sc = bcm2835_i2c_calculate_clock_divider(tfr_mode->baudrate, &clock_divider);
  
  if ( sc != RTEMS_SUCCESSFUL )
    return sc;

  /* Set clock divider */
  BCM2835_REG(BCM2835_I2C_DIV) = clock_divider;

  return sc;
}

static int bcm2835_i2c_read_write(rtems_libi2c_bus_t * bushdl, unsigned char *rd_buf, const unsigned char *wr_buf, int buffer_size)
{
  bcm2835_i2c_softc_t *softc_ptr = &(((bcm2835_i2c_desc_t *)(bushdl))->softc);

  uint32_t bytes_sent = buffer_size;

  /* Since there is a maximum of 0xFFFF number of packets per transfer,
   * count how many transfers will be needed and adjust each transfer size accordingly.
   */
  int transfer_count = buffer_size / 0xFFFF; 
  uint16_t dlen_buffer_size;

  /* If the buffer size is a multiple of the max size per transfer, round up the transfer count */
  if ( buffer_size % 0xFFFF != 0 )
    transfer_count++;

  do
  {
    if (transfer_count > 1)
      dlen_buffer_size = 0xFFFF;
    else
      dlen_buffer_size = (buffer_size & 0xFFFF);

    /* Set the DLEN bit, which specifies how many data packets will be transferred */
    BCM2835_REG(BCM2835_I2C_DLEN) = dlen_buffer_size;

    /* Clear error status */
    BCM2835_REG(BCM2835_I2C_S) |= (3 << 8);

    /* While there is data to transfer */
    while ( dlen_buffer_size >= 1 )
    {
      /* If writting */
      if ( rd_buf == NULL )
      {
        /* If transfer is not active */
        if( (BCM2835_REG(BCM2835_I2C_S) & (1 << 0)) == 0)
        {
          /* Send start bit */
          BCM2835_REG(BCM2835_I2C_C) |= (1 << 7);
        }

        if ( I2C_IO_MODE == 1 )
        {
          /* Generate interrupts on the TXW bit condition */
          BCM2835_REG(BCM2835_I2C_C) |= (1 << 9);

	  if ( rtems_semaphore_obtain(softc_ptr->irq_sema_id,RTEMS_WAIT,500) != RTEMS_SUCCESSFUL )
	    return -1;
	}

        else 
          /* Poll TXW bit until there is space available to write */
          while ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 2)) == 0 )
            ;
	
        /* Write data to the TX fifo */
        BCM2835_REG(BCM2835_I2C_FIFO) = (*(uint8_t *)wr_buf);

        wr_buf++;

        /* Check for acknowledgment or clock stretching errors */
        if ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 8)) || (BCM2835_REG(BCM2835_I2C_S) & (1 << 9)) )
          return -1;
      }

      /* If reading */
      else
      {
        /* Send start bit. Before any read a libi2c_send_addr call should be made signaling a read operation */
        BCM2835_REG(BCM2835_I2C_C) |= (1 << 7);

        /* Check for an acknowledgment error */
        if ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 8)) != 0)
          return -1;
        
        /* Poll RXD bit until there is data to read */
        while ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 5)) == 0 ) 
          ;

        /* Read data from the RX FIFO */
        (*(uint8_t *)rd_buf) = BCM2835_REG(BCM2835_I2C_FIFO) & 0xFF;

        rd_buf++;

        /* Check for acknowledgment or clock stretching errors */
        if ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 8)) || (BCM2835_REG(BCM2835_I2C_S) & (1 << 9)) )
          return -1;
      }

      dlen_buffer_size--;
      transfer_count--;
      buffer_size--;
    }
  } while ( transfer_count > 0 );

  if ( I2C_IO_MODE == 1 )
  {
    /* Generate interrupts on the DONE bit condition */
    BCM2835_REG(BCM2835_I2C_C) |= (1 << 8);

    if ( rtems_semaphore_obtain(softc_ptr->irq_sema_id,RTEMS_WAIT,500) != RTEMS_SUCCESSFUL )
      return -1;
  }

  else
    /* Poll DONE bit until data has been sent */
    while ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 1)) == 0 )
      ;

  bytes_sent -= buffer_size;

  return bytes_sent;
}

static void i2c_handler(void* arg)
{
  bcm2835_i2c_softc_t *softc_ptr = (bcm2835_i2c_softc_t *) arg;

  rtems_status_code sc;

  int txw, rxd, done;

  txw = rxd = done = 0;

  if ( (BCM2835_REG(BCM2835_I2C_C) & (1 << 9)) )
    txw = 1;

  else if ( (BCM2835_REG(BCM2835_I2C_C) & (1 << 10)) )
    rxd = 1;

  else if ( (BCM2835_REG(BCM2835_I2C_C) & (1 << 8)) )
    done = 1;

  /* If waiting to write to the bus, expect the TXW bit to be set to release the irq semaphore 
   * If waiting to read from the bus, expect the RXD bit to be set before releasing the irq semaphore
   * If waiting for a transfer to be done, expect the DONE bit to be set before releasing the irq semaphore
   */
  if (
      ( txw && (BCM2835_REG(BCM2835_I2C_S) & (1 << 2)) != 0 )
      ||
      ( rxd && (BCM2835_REG(BCM2835_I2C_S) & (1 << 5)) != 0 )
      ||
      ( done && (BCM2835_REG(BCM2835_I2C_S) & (1 << 1)) != 0 )
     )
  {
    if ( txw )
      BCM2835_REG(BCM2835_I2C_C) &= ~(1 << 9);

    else if ( rxd )
      BCM2835_REG(BCM2835_I2C_C) &= ~(1 << 10);

    else if ( done )
      BCM2835_REG(BCM2835_I2C_C) &= ~(1 << 8);

    /* Release the irq semaphore */
    sc = rtems_semaphore_release(softc_ptr->irq_sema_id);
  }
}

rtems_status_code bcm2835_i2c_init(rtems_libi2c_bus_t * bushdl)
{
  bcm2835_i2c_softc_t *softc_ptr = &(((bcm2835_i2c_desc_t *)(bushdl))->softc);
  rtems_status_code sc = RTEMS_SUCCESSFUL;

  if ( softc_ptr->initialized == 1 )
    return sc;

  softc_ptr->initialized = 1;

  /* Enable the I2C BSC interface */
  BCM2835_REG(BCM2835_I2C_C) |= (1 << 15);

  /* If the access to the bus is configured to be interrupt-driven */
  if ( I2C_IO_MODE == 1 )
  {
    sc = rtems_semaphore_create(rtems_build_name('i','2','c','s'), 0, RTEMS_FIFO | RTEMS_SIMPLE_BINARY_SEMAPHORE, 0, &softc_ptr->irq_sema_id);

    sc = rtems_interrupt_handler_install(BCM2835_IRQ_ID_I2C, NULL, RTEMS_INTERRUPT_SHARED, (rtems_interrupt_handler) i2c_handler, softc_ptr);
  }

  return sc;
}

rtems_status_code bcm2835_i2c_send_start(rtems_libi2c_bus_t * bushdl)
{
  /* Clear FIFOs */
  BCM2835_REG(BCM2835_I2C_C) |= (3 << 4);

  return RTEMS_SUCCESSFUL;
}

/* Stop condition sent automatically by the BSC interface when DLEN reaches 0 */
rtems_status_code bcm2835_i2c_stop(rtems_libi2c_bus_t * bushdl)
{
  return RTEMS_SUCCESSFUL;
}

rtems_status_code bcm2835_i2c_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw)
{
  /* Address slave device */
  BCM2835_REG(BCM2835_I2C_A) = addr;

  /* Set read/write bit. 
   *
   * If writting
   */
  if ( rw == 0 )
    BCM2835_REG(BCM2835_I2C_C) &= ~(1 << 0);

  /* If reading */
  else
    BCM2835_REG(BCM2835_I2C_C) |= (1 << 0);

  return RTEMS_SUCCESSFUL;
}

int bcm2835_i2c_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes)
{
  return bcm2835_i2c_read_write(bushdl, bytes, NULL, nbytes);
}

int bcm2835_i2c_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes)
{
  return bcm2835_i2c_read_write(bushdl, NULL, bytes, nbytes);
}

int bcm2835_i2c_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg)
{
  switch ( cmd )
  {
    case RTEMS_LIBI2C_IOCTL_SET_TFRMODE:

      return bcm2835_i2c_set_tfr_mode(bushdl, (const rtems_libi2c_tfr_mode_t *)arg);

    default:
      return -1;
  }

  return 0;
}
