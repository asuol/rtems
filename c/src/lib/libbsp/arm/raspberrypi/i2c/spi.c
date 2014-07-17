#include <bsp/raspberrypi.h>
#include <bsp/gpio.h>
#include <bsp/i2c.h>

/* GPU processor core clock rate in MHz */ //MK: probe this value?
#define GPU_CORE_CLOCK_RATE 250

/* Calculates the clock divider which provides the closest clock rate */
rtems_status_code bcm2835_spi_calculate_clock_divider(uint32_t clock_mhz, uint16_t *clock_divider, uint32_t *new_clock_rate)
{
  uint16_t divider;
  uint32_t clock_rate;

  /* Calculate the appropriate clock divider */
  divider = GPU_CORE_CLOCK_RATE / clock_mhz;

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
  while ( clock_rate > clock_mhz )
  {
    divider = (divider << 1);

    clock_rate = GPU_CORE_CLOCK_RATE / divider;
  }

  *clock_divider = divider;  
  *new_clock_rate = clock_rate;

  return RTEMS_SUCCESSFUL;
}

/* Set SPI bus transfer mode */
static rtems_status_code bcm2835_spi_set_tfr_mode(rtems_libi2c_bus_t *bushdl, const rtems_libi2c_tfr_mode_t *tfr_mode)
{
  bcm2835_spi_softc_t *softc_ptr = &(((bcm2835_spi_desc_t *)(bushdl))->softc);
  
  uint16_t clock_divider;
  uint32_t new_clock_rate;

  rtems_status_code rc = RTEMS_SUCCESSFUL;

  /* Set dummy character */
  softc_ptr->dummy_char = tfr_mode->idle_char;

  //MK: evaluate if this call sholud stay here, and how to process the different clock rates
  rc = bcm2835_spi_calculate_clock_divider(tfr_mode->baudrate, &clock_divider, &new_clock_rate);
  
  if ( rc != RTEMS_SUCCESSFUL )
    return rc;

  softc_ptr->clk_divider = clock_divider;

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

  uint32_t fifo_data;

  rtems_status_code sc;

  /* Set SPI transfer active */
  BCM2835_REG(BCM2835_SPI_CS) |= (1 << 7);

  /* While there is data to be transferred */
  while ( buffer_size >= bytes_per_char )
  {
    buffer_size--;
 
    /* If reading from the bus, send dummy characters to the device */
    if ( rd_buf != NULL )
      BCM2835_REG(BCM2835_SPI_FIFO) = (dummy_char << bit_shift);

    /* If writting to the bus, write the data from the buffer to the RX FIFO */
    else
    {
      switch ( bytes_per_char )
      {
        case 1:

          BCM2835_REG(BCM2835_SPI_FIFO) = ((*(uint8_t *)wr_buf) << bit_shift);
	  break;

        case 2:

          BCM2835_REG(BCM2835_SPI_FIFO) = ((*(uint16_t *)wr_buf) << bit_shift);
	  break;

        case 3:

          BCM2835_REG(BCM2835_SPI_FIFO) = ((*(uint16_t *)wr_buf) << bit_shift);

          wr_buf += 2;

          BCM2835_REG(BCM2835_SPI_FIFO) = ((*(uint8_t *)wr_buf) << bit_shift);

          wr_buf++;

	  break;

        case 4:

          BCM2835_REG(BCM2835_SPI_FIFO) = ((*(uint32_t *)wr_buf) << bit_shift);
	  break;
      }

      if (bytes_per_char != 3 )
        wr_buf += bytes_per_char;

    }
    
    /* If in polled mode, check the done bit in the SPI control register every
     * second until the transfer is complete.
     */
    if ( softc_ptr->transfer_mode == SPI_POLLED )
      while ( (BCM2835_REG(BCM2835_SPI_CS) & (1 << 16)) == 0 )
      {
        sc = rtems_task_wake_after(rtems_clock_get_ticks_per_second());

        if ( sc != RTEMS_SUCCESSFUL )
          return -1;
      }

    /* If reading from the bus, retrive the data from the bus FIFO and store in the buffer */
    if ( rd_buf != NULL )
    {
      fifo_data = BCM2835_REG(BCM2835_SPI_FIFO);

      switch ( bytes_per_char )
      {
        case 1:

          (*(uint8_t *)rd_buf) = (fifo_data >> bit_shift);
	  break;

        case 2:

	  (*(uint16_t *)rd_buf) = (fifo_data >> bit_shift);
	  break;

        case 3:

          (*(uint16_t *)rd_buf) = (fifo_data >> bit_shift);

          rd_buf += 2;

          (*(uint8_t *)rd_buf) = (fifo_data >> bit_shift);

          rd_buf++;

	  break;

        case 4:

	  (*(uint32_t *)rd_buf) = (fifo_data >> bit_shift);
	  break;
      }

      if (bytes_per_char != 3 )
        rd_buf += bytes_per_char;

    }
  }

  return 0;
}

rtems_status_code bcm2835_spi_init(rtems_libi2c_bus_t * bushdl)
{
  bcm2835_spi_softc_t *softc_ptr = &(((bcm2835_spi_desc_t *)(bushdl))->softc);

  /* Enable the SPI interface on the Raspberry Pi P1 GPIO header */
  gpio_initialize ();

  if ( gpio_select_spi_p1() < 0 )
    return RTEMS_RESOURCE_IN_USE;

  /* Set chip select polarity (ative low) */
  BCM2835_REG(BCM2835_SPI_CS) &= ~(1 << 6);

  softc_ptr->initialized = 1; // MK: use this info

  return RTEMS_SUCCESSFUL;
}
		        
rtems_status_code bcm2835_spi_send_start(rtems_libi2c_bus_t * bushdl)
{
  return RTEMS_SUCCESSFUL;
}

rtems_status_code bcm2835_spi_stop(rtems_libi2c_bus_t * bushdl);

/* Activates the SPI select line that matches addr */
rtems_status_code bcm2835_spi_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw)
{
  /* Select one of the two available SPI slaves */
  switch ( addr )
  {
    case 0:

      BCM2835_REG(BCM2835_SPI_CS) &= ~(1 << addr);
      break;

    case 1:

      BCM2835_REG(BCM2835_SPI_CS) |= addr;
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
  }

  return 0;
}
