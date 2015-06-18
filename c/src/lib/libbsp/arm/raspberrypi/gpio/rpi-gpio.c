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

#include "gpio-interfaces.c"

/* Calculates a bitmask to assign an alternate function to a given pin. */
#define SELECT_PIN_FUNCTION(fn, pn) (fn << ((pn % 10) * 3))

/**
 * @brief Waits a number of CPU cycles.
 *
 * @param[in] cycles The number of CPU cycles to wait.
 */
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

//TODO: receive pointer, and fill struct??
gpio_layout bsp_gpio_initialize()
{
  gpio_layout rpi_layout;
  
  rpi_layout.pin_count = 54;
  rpi_layout.pins_per_bank = 54;

  return rpi_layout;
}

rtems_status_code bsp_gpio_set(uint32_t bank, uint32_t pin)
{
  BCM2835_REG(BCM2835_GPIO_GPSET0) = (1 << pin);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_gpio_clear(uint32_t bank, uint32_t pin)
{
  BCM2835_REG(BCM2835_GPIO_GPCLR0) = (1 << pin);

  return RTEMS_SUCCESSFUL;
}

int bsp_gpio_get_value(uint32_t bank, uint32_t pin)
{
  return (BCM2835_REG(BCM2835_GPIO_GPLEV0) & (1 << pin));
}

rtems_status_code bsp_gpio_select_input(uint32_t bank, uint32_t pin, void* bsp_specific)
{
  /* Calculate the pin function select register address. */
  volatile unsigned int *pin_addr = (unsigned int *)BCM2835_GPIO_REGS_BASE +
                                    (pin / 10);

  *(pin_addr) &= ~SELECT_PIN_FUNCTION(RPI_DIGITAL_IN, pin);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_gpio_select_output(uint32_t bank, uint32_t pin, void* bsp_specific)
{
  return rpi_select_pin_function(bank, pin, RPI_DIGITAL_OUT);
}

rtems_status_code bsp_select_specific_io(uint32_t bank, uint32_t pin, uint32_t function, void* pin_data)
{
  return rpi_select_pin_function(bank, pin, function);
}

rtems_status_code bsp_gpio_set_input_mode(uint32_t bank, uint32_t pin, gpio_pull_mode mode)
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

rtems_vector_number bsp_gpio_get_vector(uint32_t bank)
{
  return BCM2835_IRQ_ID_GPIO_0;
}

uint32_t bsp_gpio_interrupt_line(rtems_vector_number vector)
{
  return BCM2835_REG(BCM2835_GPIO_GPEDS0);
}

void bsp_gpio_clear_interrupt_line(rtems_vector_number vector, uint32_t event_status)
{
  BCM2835_REG(BCM2835_GPIO_GPEDS0) = event_status;
}

rtems_status_code bsp_enable_interrupt(uint32_t pin, gpio_interrupt interrupt)
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

rtems_status_code bsp_disable_interrupt(uint32_t pin, gpio_interrupt enabled_interrupt)
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

/**
 * @brief Setups a JTAG interface using the P1 GPIO pin header
 *        for the models A/B and J8 header on the B+. 
 *        The following pins should be unused before calling this function:
 *        GPIO 4, 22, 24, 25 and 27.
 *
 * @retval RTEMS_SUCCESSFUL JTAG interface successfully configured.
 * @retval RTEMS_RESOURCE_IN_USE At least one of the required pins is currently
 *                               occupied, @see gpio_select_pin().
 */
rtems_status_code gpio_select_jtag(void)
{
  rtems_status_code sc;

  sc = gpio_request_conf(&arm_tdi);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = gpio_request_conf(&arm_trst);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = gpio_request_conf(&arm_tdo);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = gpio_request_conf(&arm_tck);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = gpio_request_conf(&arm_tms);

  return sc;
}

/**
 * @brief Setups a SPI interface using the P1 GPIO pin header
 *        for the models A/B and J8 header on the B+. 
 *        The following pins should be unused before calling this function:
 *        GPIO 7, 8, 9, 10 and 11. 
 *
 * @retval RTEMS_SUCCESSFUL SPI interface successfully configured.
 * @retval RTEMS_RESOURCE_IN_USE At least one of the required pins is currently
 *                               occupied, @see gpio_select_pin().
 */
rtems_status_code gpio_select_spi_p1(void)
{
  rtems_status_code sc;
  
  sc = gpio_request_conf(&spi_p1_miso);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = gpio_request_conf(&spi_p1_mosi);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = gpio_request_conf(&spi_p1_sclk);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = gpio_request_conf(&spi_p1_ce_0);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = gpio_request_conf(&spi_p1_ce_1);

  return sc;
}

/**
 * @brief Setups a I2C interface using the P1 GPIO pin header
 *        for the models A/B and J8 header on the B+. 
 *        The following pins should be unused before calling this function:
 *        GPIO 2 and 3.
 *
 * @retval RTEMS_SUCCESSFUL I2C interface successfully configured.
 * @retval RTEMS_RESOURCE_IN_USE At least one of the required pins is currently
 *                               occupied, @see gpio_select_pin().
 */
rtems_status_code gpio_select_i2c_p1_rev2(void)
{
  rtems_status_code sc;
  
  sc = gpio_request_conf(&i2c_p1_rev2_sda);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }
  
  sc = gpio_request_conf(&i2c_p1_rev2_scl);

  return sc;
}
