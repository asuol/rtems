#include <bsp/raspberrypi.h>
#include <bsp/gpio.h>
#include <bsp/irq.h>
#include <bsp/i2c.h>

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

rtems_status_code bcm2835_i2c_init(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_i2c_send_start(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_i2c_stop(rtems_libi2c_bus_t * bushdl);

rtems_status_code bcm2835_i2c_send_addr(rtems_libi2c_bus_t * bushdl, uint32_t addr, int rw);

int bcm2835_i2c_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_i2c_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes, int nbytes);

int bcm2835_i2c_ioctl(rtems_libi2c_bus_t * bushdl, int cmd, void *arg);

