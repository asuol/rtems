#include <bsp/raspberrypi.h>
#include <bsp/gpio.h>
#include <bsp/irq.h>
#include <bsp/i2c.h>

#include <libchip/mcp23008.h>

#define BSC_CORE_CLK_HZ 150000000

/* NOT implemented:
 * - Clock stretching (using default values)
 * - 10-bit addressing
 * - Falling/Rising edge delays (using default values)
 * - Interrupt mode
 */

rtems_libi2c_bus_ops_t bcm2835_i2c_ops = {
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

int i2c_bus_no_p1;

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
        /* Poll TXD bit until there is space available to write */
        while ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 4)) == 0 )
          ;

        /* Write data to the TX fifo */
        BCM2835_REG(BCM2835_I2C_FIFO) = (*(uint8_t *)wr_buf);

        wr_buf++;

        /* If transfer is not active */
        if( (BCM2835_REG(BCM2835_I2C_S) & (1 << 0)) == 0)
          {
            /* Send start bit */
            BCM2835_REG(BCM2835_I2C_C) |= (1 << 7);
          }

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

  /* Poll DONE bit until data has been sent */
  while ( (BCM2835_REG(BCM2835_I2C_S) & (1 << 1)) == 0 )
    ;

  bytes_sent -= buffer_size;

  return bytes_sent;
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

rtems_status_code bcm2835_register_i2c(void)
{
  int rv = 0;

  /* Initialize the libi2c API */
  rtems_libi2c_initialize ();

  /* Enable the I2C interface on the Raspberry Pi P1 GPIO header */
  gpio_initialize ();

  if ( gpio_select_i2c_p1_rev2() < 0 )
    return RTEMS_RESOURCE_IN_USE;

  /* Register the I2C bus */
  rv = rtems_libi2c_register_bus("/dev/i2c", &(bcm2835_i2c_bus_desc.bus_desc));

  if ( rv < 0 )
    return -rv;
  
  i2c_bus_no_p1 = rv;

  return 0;
}

int bcm2835_mcp23008_init(void)
{
  return rtems_libi2c_register_drv("mcp23008", &i2c_mcp23008_drv_t, i2c_bus_no_p1, MCP23008_ADDR);
}
