#include <bsp.h>
#include <bsp/raspberrypi.h>
#include <bsp/gpio.h>
#include <bsp/irq.h>
#include <bsp/i2c.h>

/* GPU processor core clock rate in Hz */ //MK: probe this value?
#define GPU_CORE_CLOCK_RATE 250000000

/* Calculates the clock divider that provides the closest (<=) clock rate to the desired */
static rtems_status_code bcm2835_spi_calculate_clock_divider(uint32_t clock_hz, uint16_t *clock_divider)
{
  uint16_t divider;
  uint32_t clock_rate;

  /* Calculate the appropriate clock divider */
  divider = GPU_CORE_CLOCK_RATE / clock_hz;

  if ( divider < 0 || divider > 65536 )
    return RTEMS_INVALID_NUMBER;

  /* Calculate the next greater power of two */
  divider--;

  divider |= (divider >> 1);
  divider |= (divider >> 2);
  divider |= (divider >> 4);
  divider |= (divider >> 8);

  divider++;

  clock_rate = GPU_CORE_CLOCK_RATE / divider;

  /* If the resulting clock rate is greater than desired, try the next greater power of two divider  */
  while ( clock_rate > clock_hz )
  {
    divider = (divider << 1);

    clock_rate = GPU_CORE_CLOCK_RATE / divider;
  }

  *clock_divider = divider;

  return RTEMS_SUCCESSFUL;
}

/* Set SPI bus transfer mode */
static rtems_status_code bcm2835_spi_set_tfr_mode(rtems_libi2c_bus_t *bushdl, const rtems_libi2c_tfr_mode_t *tfr_mode)
{
  bcm2835_spi_softc_t *softc_ptr = &(((bcm2835_spi_desc_t *)(bushdl))->softc);
  
  uint16_t clock_divider;

  rtems_status_code rc = RTEMS_SUCCESSFUL;

  /* Set dummy character */
  softc_ptr->dummy_char = tfr_mode->idle_char;

  /* Calculate the most appropriate clock divider */
  rc = bcm2835_spi_calculate_clock_divider(tfr_mode->baudrate, &clock_divider);
  
  if ( rc != RTEMS_SUCCESSFUL )
    return rc;

  /* Set clock divider */
  BCM2835_REG(BCM2835_SPI_CLK) = clock_divider;
   
  /* Calculate how many bytes each character has.
   *
   * Only multiples of 8 bits are accepted for the transaction.
   */
  switch ( tfr_mode->bits_per_char )
  {
    case 8:
    case 16:
    case 24:
    case 32:

      softc_ptr->bytes_per_char = tfr_mode->bits_per_char / 8;

      break;
   
    default:
      return RTEMS_INVALID_NUMBER;
  }

  /* Check the data mode and calculate the correcting bit shift value to apply */
  if ( tfr_mode->lsb_first )
    softc_ptr->bit_shift = 32 - tfr_mode->bits_per_char;

  /* If MSB first */
  else
    softc_ptr->bit_shift = 0;

  /* Set SPI clock polarity.
   *
   * If clock_inv == TRUE, active high.
   */
  if ( tfr_mode->clock_inv )
    /* Rest state of clock = low */
    BCM2835_REG(BCM2835_SPI_CS) &= ~(1 << 3);

  else
    /* Rest state of clock = high */
    BCM2835_REG(BCM2835_SPI_CS) |= (1 << 3);
  
  /* Set SPI clock phase.
   *
   * If clock_phs == true, clock starts toggling at the start of the data transfer
   */
  if ( tfr_mode->clock_phs )
    /* First SCLK transition at beginning of data bit */
    BCM2835_REG(BCM2835_SPI_CS) |= (1 << 2);

  else
    /* First SCLK transition at middle of data bit */
    BCM2835_REG(BCM2835_SPI_CS) &= ~(1 << 2);

  return rc;
}

static int bcm2835_spi_read_write(rtems_libi2c_bus_t * bushdl, unsigned char *rd_buf, const unsigned char *wr_buf, int buffer_size)
{
  bcm2835_spi_softc_t *softc_ptr = &(((bcm2835_spi_desc_t *)(bushdl))->softc);
  
  uint8_t bytes_per_char = softc_ptr->bytes_per_char;
  uint8_t bit_shift =  softc_ptr->bit_shift;
  uint32_t dummy_char = softc_ptr->dummy_char;

  uint32_t bytes_sent = buffer_size;
  uint32_t fifo_data;

  /* Clear SPI bus FIFOs */
  BCM2835_REG(BCM2835_SPI_CS) |= (3 << 4);

  /* Set SPI transfer active */
  BCM2835_REG(BCM2835_SPI_CS) |= (1 << 7);

  if ( SPI_IO_MODE == 1 )
  {
    softc_ptr->irq_write = 1;

    BCM2835_REG(BCM2835_SPI_CS) |= (3 << 9);

    if ( rtems_semaphore_obtain(softc_ptr->irq_sema_id,RTEMS_WAIT,50) != RTEMS_SUCCESSFUL )
      return -1;
  }

  else
    /* 
     * If in polling mode,
     * Poll TXD bit until there is space to write at least one byte on the TX FIFO
     */
    while ( (BCM2835_REG(BCM2835_SPI_CS) & (1 << 18)) == 0 )
      ;

  /* While there is data to be transferred */
  while ( buffer_size >= bytes_per_char )
  {
    /* If reading from the bus, send a dummy character to the device */
    if ( rd_buf != NULL )
      BCM2835_REG(BCM2835_SPI_FIFO) = dummy_char;

    /* If writting to the bus, move the buffer data to the TX FIFO */
    else
    {
      switch ( bytes_per_char )
      {
        case 1:

          BCM2835_REG(BCM2835_SPI_FIFO) = (((*wr_buf) & 0xFF) << bit_shift);
          break;

        case 2:

          BCM2835_REG(BCM2835_SPI_FIFO) = (((*wr_buf) & 0xFFFF) << bit_shift);
          break;

        case 3:

          BCM2835_REG(BCM2835_SPI_FIFO) = (((*wr_buf) & 0xFFFFFF) << bit_shift);

          break;

        case 4:

          BCM2835_REG(BCM2835_SPI_FIFO) = ((*wr_buf) << bit_shift);
          break;

        default:
          return -1;
      }

      wr_buf += bytes_per_char;

      buffer_size -= bytes_per_char;
    }
    
    /* If using bi-directional SPI */
    if ( softc_ptr->wire_mode == SPI_2_WIRE )
      /* Change bus direction to read from the slave */
      BCM2835_REG(BCM2835_SPI_CS) |= (1 << 12);

    if ( SPI_IO_MODE == 1 )
    {
      softc_ptr->irq_write = 0;

      BCM2835_REG(BCM2835_SPI_CS) |= (3 << 9);

      if ( rtems_semaphore_obtain(softc_ptr->irq_sema_id,RTEMS_WAIT,50) != RTEMS_SUCCESSFUL )
        return -1;
    }

    /* If in polling mode */
    else
    {
      /* Poll Done bit until the data transfer is complete */
      while ( (BCM2835_REG(BCM2835_SPI_CS) & (1 << 16)) == 0 )
        ;
        
      /* Poll RXD bit until there is at least one byte on the RX FIFO to be read */
      while ( (BCM2835_REG(BCM2835_SPI_CS) & (1 << 17)) == 0 )
        ;
    }

    /* If writting to the bus, read the dummy char sent by the slave device */
    if ( rd_buf == NULL )
      fifo_data = BCM2835_REG(BCM2835_SPI_FIFO) & 0xFF; 

    /* If reading from the bus, retrieve data from the RX FIFO and store it on the buffer */
    if ( rd_buf != NULL )
    {
      switch ( bytes_per_char )
      {
        case 1:

          fifo_data = BCM2835_REG(BCM2835_SPI_FIFO) & 0xFF;
          (*rd_buf) = (fifo_data >> bit_shift);
          break;

        case 2:

          fifo_data = BCM2835_REG(BCM2835_SPI_FIFO) & 0xFFFF;
          (*rd_buf) = (fifo_data >> bit_shift);
          break;

        case 3:

          fifo_data = BCM2835_REG(BCM2835_SPI_FIFO) & 0xFFFFFF;
          (*rd_buf) = (fifo_data >> bit_shift);
          break;

        case 4:

          fifo_data = BCM2835_REG(BCM2835_SPI_FIFO);
          (*rd_buf) = (fifo_data >> bit_shift);
          break;

        default:
          return -1;
      }

      rd_buf += bytes_per_char;

      buffer_size -= bytes_per_char;
    }

    /* If using bi-directional SPI */
    if ( softc_ptr->wire_mode == SPI_2_WIRE )
      /* Restore bus direction to write to the slave */
      BCM2835_REG(BCM2835_SPI_CS) &= ~(1 << 12);
  }

  if ( SPI_IO_MODE == 1 )
  {
    softc_ptr->irq_write = 1;

    BCM2835_REG(BCM2835_SPI_CS) |= (3 << 9);

    if ( rtems_semaphore_obtain(softc_ptr->irq_sema_id,RTEMS_WAIT,50) != RTEMS_SUCCESSFUL )
      return -1;
  }

  else
    /* 
     * If in polling mode,
     * Poll Done bit until the data transfer is complete.
     */
    while ( (BCM2835_REG(BCM2835_SPI_CS) & (1 << 16)) == 0 )
      ;

  bytes_sent -= buffer_size;
  
  return bytes_sent;
}

static void spi_handler(void* arg)
{
  bcm2835_spi_softc_t *softc_ptr = (bcm2835_spi_softc_t *) arg;

  /* If waiting to write to the bus, expect DONE and TXD bits to be set to release the irq semaphore 
   * If waiting to read from the bus, expect DONE and RXD bits to be set before releasing the irq semaphore
   */
  if (
      ( softc_ptr->irq_write == 1 && (BCM2835_REG(BCM2835_SPI_CS) & (1 << 16)) != 0 && (BCM2835_REG(BCM2835_SPI_CS) & (1 << 18)) != 0 )
      ||
      ( softc_ptr->irq_write == 0 && (BCM2835_REG(BCM2835_SPI_CS) & (1 << 16)) != 0 && (BCM2835_REG(BCM2835_SPI_CS) & (1 << 17)) != 0 )
     )
  {
    /* Disable the SPI interrupt generation */
    BCM2835_REG(BCM2835_SPI_CS) &= ~(3 << 9);

    /* Release the irq semaphore */
    rtems_semaphore_release(softc_ptr->irq_sema_id);
  }
}

rtems_status_code bcm2835_spi_init(rtems_libi2c_bus_t * bushdl)
{
  bcm2835_spi_softc_t *softc_ptr = &(((bcm2835_spi_desc_t *)(bushdl))->softc);
  rtems_status_code sc = RTEMS_SUCCESSFUL;

  if ( softc_ptr->initialized == 1 )
    return sc;

  softc_ptr->initialized = 1;

  // FIXME: this should be selectable elsewhere
  softc_ptr->wire_mode = SPI_3_WIRE;

  if ( SPI_IO_MODE == 1 )
  {
    sc = rtems_semaphore_create(rtems_build_name('s','p','i','s'), 0, RTEMS_FIFO | RTEMS_SIMPLE_BINARY_SEMAPHORE, 0, &softc_ptr->irq_sema_id);

    sc = rtems_interrupt_handler_install(BCM2835_IRQ_ID_SPI, NULL, RTEMS_INTERRUPT_SHARED, (rtems_interrupt_handler) spi_handler, softc_ptr);
  }
  
  return sc;
}

rtems_status_code bcm2835_spi_send_start(rtems_libi2c_bus_t * bushdl)
{
  return RTEMS_SUCCESSFUL;
}

rtems_status_code bcm2835_spi_stop(rtems_libi2c_bus_t * bushdl)
{
  bcm2835_spi_softc_t *softc_ptr = &(((bcm2835_spi_desc_t *)(bushdl))->softc);
  rtems_status_code sc = RTEMS_SUCCESSFUL;

  uint32_t addr = softc_ptr->current_slave_addr;
  uint32_t chip_select_bit = 21 + addr;

  /* Set SPI transfer as not active */
  BCM2835_REG(BCM2835_SPI_CS) &= ~(1 << 7);

  /* Unselect the active SPI slave */
  switch ( addr )
  {
    case 0:
    case 1:

      BCM2835_REG(BCM2835_SPI_CS) |= (1 << chip_select_bit);
      break;

    default:
      return RTEMS_INVALID_ADDRESS;
  }

  return sc;
}

/* Activates the SPI select line that matches addr */
rtems_status_code bcm2835_spi_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw)
{
  bcm2835_spi_softc_t *softc_ptr = &(((bcm2835_spi_desc_t *)(bushdl))->softc);

  uint32_t chip_select_bit = 21 + addr;

  /* Save which slave will be currently addressed */
  softc_ptr->current_slave_addr = addr;

  /* Select one of the two available SPI slaves */
  switch ( addr )
  {
    case 0:
    case 1:

      BCM2835_REG(BCM2835_SPI_CS) &= ~(1 << chip_select_bit);
      break;
   
    default:
      return RTEMS_INVALID_ADDRESS;
  }
  
  return RTEMS_SUCCESSFUL;
}

/* Read a number of bytes from the SPI bus */
int bcm2835_spi_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes)
{
  return bcm2835_spi_read_write(bushdl, bytes, NULL, nbytes);
}

/* Write a number of bytes to the SPI bus */
int bcm2835_spi_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes)
{
  return bcm2835_spi_read_write(bushdl, NULL, bytes, nbytes);
}

/* ioctl misc functions */
int bcm2835_spi_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg)
{
  switch ( cmd )
  {
    case RTEMS_LIBI2C_IOCTL_SET_TFRMODE:

      return bcm2835_spi_set_tfr_mode(bushdl, (const rtems_libi2c_tfr_mode_t *)arg);

    case RTEMS_LIBI2C_IOCTL_READ_WRITE:
    
      return bcm2835_spi_read_write(bushdl,  
                                   ((rtems_libi2c_read_write_t *)arg)->rd_buf,
                                   ((rtems_libi2c_read_write_t *)arg)->wr_buf,
                                   ((rtems_libi2c_read_write_t *)arg)->byte_cnt);

    default:
      return -1;
  }

  return 0;
}
