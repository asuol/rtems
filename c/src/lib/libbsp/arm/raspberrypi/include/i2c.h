#include <rtems/libi2c.h>

/* Transfer mode macros */
#define SPI_POLLED  0
#define SPI_IRQ     1
#define SPI_DMA     2

/* SPI data structures */

typedef struct {
  int                 initialized;
  uint8_t             bytes_per_char;
  uint8_t             bit_shift; /* To correct the data significant bit position */
  uint16_t            clk_divider;
  uint32_t            dummy_char;
  int                 transfer_mode;
} bcm2835_spi_softc_t;

typedef struct {
  rtems_libi2c_bus_t  bus_desc;
  bcm2835_spi_softc_t softc;
} bcm2835_spi_desc_t;

/* SPI directives */

rtems_status_code bcm2835_spi_calculate_clock_divider(uint32_t clock_mhz, uint16_t *clock_divider, uint32_t *new_clock_rate);

rtems_status_code bcm2835_spi_init(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_spi_send_start(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_spi_stop(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_spi_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw);

int bcm2835_spi_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_spi_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_spi_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg);
