/**
 * @file rpi-gpio.c
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Support for the Raspberry PI GPIO.
 */

/*
 *  Copyright (c) 2014-2015 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/raspberrypi.h>
#include <bsp/irq-generic.h>
#include <bsp/gpio.h>
#include <bsp/rpi-gpio.h>

#include <stdlib.h>

/* Calculates a bitmask to assign an alternate function to a given pin. */
#define SELECT_PIN_FUNCTION(fn, pn) (fn << ((pn % 10) * 3))

rtems_gpio_specific_data alt_func_def[] = {
  {io_function: RPI_ALT_FUNC_0, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_1, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_2, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_3, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_4, pin_data: NULL},
  {io_function: RPI_ALT_FUNC_5, pin_data: NULL}
};

/* Raspberry Pi 1 Rev 2 gpio interface definitions. */
#include "gpio-interfaces-pi1-rev2.c"

/* Waits a number of CPU cycles. */
static void arm_delay (int cycles)
{
  int i;

  for ( i = 0; i < cycles; ++i ) {
    asm volatile ("nop");
  }
}

static rtems_status_code rpi_select_pin_function(uint32_t bank, uint32_t pin, uint32_t type)
{
  /* Calculate the pin function select register address. */
  volatile unsigned int *pin_addr = (unsigned int *)BCM2835_GPIO_REGS_BASE +
                                    (pin / 10);

  *(pin_addr) |= SELECT_PIN_FUNCTION(type, pin);

  return RTEMS_SUCCESSFUL;
}

rtems_gpio_layout rtems_bsp_gpio_initialize()
{
  rtems_gpio_layout rpi_layout;

  rpi_layout.pin_count = GPIO_COUNT;
  rpi_layout.pins_per_bank = GPIO_COUNT;

  return rpi_layout;
}

rtems_status_code rtems_bsp_gpio_multi_set(uint32_t bank, uint32_t bitmask)
{
  BCM2835_REG(BCM2835_GPIO_GPSET0) |= bitmask;

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_bsp_gpio_multi_clear(uint32_t bank, uint32_t bitmask)
{
  BCM2835_REG(BCM2835_GPIO_GPCLR0) |= bitmask;

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_bsp_gpio_set(uint32_t bank, uint32_t pin)
{
  BCM2835_REG(BCM2835_GPIO_GPSET0) = (1 << pin);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_bsp_gpio_clear(uint32_t bank, uint32_t pin)
{
  BCM2835_REG(BCM2835_GPIO_GPCLR0) = (1 << pin);

  return RTEMS_SUCCESSFUL;
}

int rtems_bsp_gpio_get_value(uint32_t bank, uint32_t pin)
{
  return (BCM2835_REG(BCM2835_GPIO_GPLEV0) & (1 << pin));
}

rtems_status_code rtems_bsp_gpio_select_input(uint32_t bank, uint32_t pin, void* bsp_specific)
{
  /* Calculate the pin function select register address. */
  volatile unsigned int *pin_addr = (unsigned int *)BCM2835_GPIO_REGS_BASE +
                                    (pin / 10);

  *(pin_addr) &= ~SELECT_PIN_FUNCTION(RPI_DIGITAL_IN, pin);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_bsp_gpio_select_output(uint32_t bank, uint32_t pin, void* bsp_specific)
{
  return rpi_select_pin_function(bank, pin, RPI_DIGITAL_OUT);
}

rtems_status_code rtems_bsp_select_specific_io(uint32_t bank, uint32_t pin, uint32_t function, void* pin_data)
{
  return rpi_select_pin_function(bank, pin, function);
}

rtems_status_code rtems_bsp_gpio_set_resistor_mode(uint32_t bank, uint32_t pin, rtems_gpio_pull_mode mode)
{
  /* Set control signal. */
  switch ( mode ) {
    case PULL_UP:
      BCM2835_REG(BCM2835_GPIO_GPPUD) = (1 << 1);
      break;
    case PULL_DOWN:
      BCM2835_REG(BCM2835_GPIO_GPPUD) = (1 << 0);
      break;
    case NO_PULL_RESISTOR:
      BCM2835_REG(BCM2835_GPIO_GPPUD) = 0;
      break;
    default:
      return RTEMS_UNSATISFIED;
  }

  /* Wait 150 cyles, as per BCM2835 documentation. */
  arm_delay(150);

  /* Setup clock for the control signal. */
  BCM2835_REG(BCM2835_GPIO_GPPUDCLK0) = (1 << pin);

  arm_delay(150);

  /* Remove the control signal. */
  BCM2835_REG(BCM2835_GPIO_GPPUD) = 0;

  /* Remove the clock. */
  BCM2835_REG(BCM2835_GPIO_GPPUDCLK0) = 0;

  return RTEMS_SUCCESSFUL;
}

rtems_vector_number rtems_bsp_gpio_get_vector(uint32_t bank)
{
  return BCM2835_IRQ_ID_GPIO_0;
}

uint32_t rtems_bsp_gpio_interrupt_line(rtems_vector_number vector)
{
  return BCM2835_REG(BCM2835_GPIO_GPEDS0);
}

void rtems_bsp_gpio_clear_interrupt_line(rtems_vector_number vector, uint32_t event_status)
{
  BCM2835_REG(BCM2835_GPIO_GPEDS0) = event_status;
}

rtems_status_code rtems_bsp_enable_interrupt(uint32_t bank, uint32_t pin, rtems_gpio_interrupt interrupt)
{
  switch ( interrupt ) {
    case FALLING_EDGE:
      /* Enables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) |= (1 << pin);
      break;
    case RISING_EDGE:
      /* Enables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) |= (1 << pin);
      break;
    case BOTH_EDGES:
      /* Enables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) |= (1 << pin);

      /* Enables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) |= (1 << pin);
      break;
    case LOW_LEVEL:
      /* Enables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) |= (1 << pin);
      break;
    case HIGH_LEVEL:
      /* Enables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) |= (1 << pin);
      break;
    case BOTH_LEVELS:
      /* Enables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) |= (1 << pin);

      /* Enables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) |= (1 << pin);
      break;
    case NONE:
    default:
      return RTEMS_UNSATISFIED;
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_bsp_disable_interrupt(uint32_t bank, uint32_t pin, rtems_gpio_interrupt enabled_interrupt)
{
  switch ( enabled_interrupt ) {
    case FALLING_EDGE:
      /* Disables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) &= ~(1 << pin);
      break;
    case RISING_EDGE:
      /* Disables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) &= ~(1 << pin);
      break;
    case BOTH_EDGES:
      /* Disables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) &= ~(1 << pin);

      /* Disables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) &= ~(1 << pin);
      break;
    case LOW_LEVEL:
      /* Disables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) &= ~(1 << pin);
      break;
    case HIGH_LEVEL:
      /* Disables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) &= ~(1 << pin);
      break;
    case BOTH_LEVELS:
      /* Disables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) &= ~(1 << pin);

      /* Disables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) &= ~(1 << pin);
      break;
    case NONE:
    default:
      return RTEMS_UNSATISFIED;
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rpi_gpio_select_jtag(void)
{
  rtems_status_code sc;

  sc = rtems_gpio_request_conf(&arm_tdi);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_request_conf(&arm_trst);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_request_conf(&arm_tdo);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_request_conf(&arm_tck);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_request_conf(&arm_tms);

  return sc;
}

rtems_status_code rpi_gpio_select_spi(void)
{
  rtems_status_code sc;

  sc = rtems_gpio_request_conf(&spi_miso);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_request_conf(&spi_mosi);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_request_conf(&spi_sclk);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_request_conf(&spi_ce_0);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_request_conf(&spi_ce_1);

  return sc;
}

rtems_status_code rpi_gpio_select_i2c(void)
{
  rtems_status_code sc;

  sc = rtems_gpio_request_conf(&i2c_sda);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_request_conf(&i2c_scl);

  return sc;
}
