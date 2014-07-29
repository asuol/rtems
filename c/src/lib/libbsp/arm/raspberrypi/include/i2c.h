#include <rtems/libi2c.h>

/* Transfer mode macros */
#define SPI_POLLED  0
#define SPI_IRQ     1
#define SPI_DMA     2

/* SPI wire version */
#define SPI_3_WIRE  0 /* MOSI, MISO and SCLK */
#define SPI_2_WIRE  1 /* Bi-directional MOSI data line and SCLK */

/* SPI data structures */

typedef struct {
  int                 initialized;
  uint8_t             bytes_per_char;
  uint8_t             bit_shift; /* To correct the data significant bit position */
  uint32_t            dummy_char;
  int                 transfer_mode;
  int                 wire_mode;
  uint32_t            current_slave_addr;
  rtems_id            irq_sema_id;
  int                 irq_write;
} bcm2835_spi_softc_t;

typedef struct {
  rtems_libi2c_bus_t  bus_desc;
  bcm2835_spi_softc_t softc;
} bcm2835_spi_desc_t;

/* SPI bus number for the GPIO P1 header SPI interface */
extern int spi_bus_no_p1;

/* SPI directives */

rtems_status_code bcm2835_spi_init(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_spi_send_start(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_spi_stop(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_spi_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw);

int bcm2835_spi_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_spi_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_spi_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg);

rtems_status_code bcm2835_register_spi(void);

int bcm2835_23k256_init(void);

/* I2C directives */

rtems_status_code bcm2835_i2c_init(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_i2c_send_start(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_i2c_stop(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_i2c_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw);

int bcm2835_i2c_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_i2c_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_i2c_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg);
