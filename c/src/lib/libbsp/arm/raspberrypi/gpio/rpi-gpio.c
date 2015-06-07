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
#include <bsp/irq.h>
#include <bsp/gpio.h>

/*
 *
 * Raspberry PI specific functions, which any BSP would need to implement.
 *
 */

void bsp_gpio_set(int pin)
{
  BCM2835_REG(BCM2835_GPIO_GPSET0) = (1 << pin);
}

void bsp_gpio_clear(int pin)
{
  BCM2835_REG(BCM2835_GPIO_GPCLR0) = (1 << pin);
}

void interrupt_vector_disable(void)// TODO: return the vector ID instead
{
  bsp_interrupt_vector_disable(BCM2835_IRQ_ID_GPIO_0);
}

void interrupt_vector_enable(void)// TODO: return the vector ID instead
{
  bsp_interrupt_vector_enable(BCM2835_IRQ_ID_GPIO_0);
}

void bsp_enable_interrupt(int dev_pin, gpio_interrupt interrupt)
{
  switch ( interrupt ) {
  case FALLING_EDGE:
    /* Enables asynchronous falling edge detection. */
    BCM2835_REG(BCM2835_GPIO_GPAFEN0) |= (1 << dev_pin);
    break;

  case RISING_EDGE:
    /* Enables asynchronous rising edge detection. */
    BCM2835_REG(BCM2835_GPIO_GPAREN0) |= (1 << dev_pin);
    break;

  case BOTH_EDGES:
    /* Enables asynchronous falling edge detection. */
    BCM2835_REG(BCM2835_GPIO_GPAFEN0) |= (1 << dev_pin);

    /* Enables asynchronous rising edge detection. */
    BCM2835_REG(BCM2835_GPIO_GPAREN0) |= (1 << dev_pin);
    break;

  case LOW_LEVEL:
    /* Enables pin low level detection. */
    BCM2835_REG(BCM2835_GPIO_GPLEN0) |= (1 << dev_pin);
    break;

  case HIGH_LEVEL:
    /* Enables pin high level detection. */
    BCM2835_REG(BCM2835_GPIO_GPHEN0) |= (1 << dev_pin);
    break;

  case BOTH_LEVELS:
    /* Enables pin low level detection. */
    BCM2835_REG(BCM2835_GPIO_GPLEN0) |= (1 << dev_pin);

    /* Enables pin high level detection. */
    BCM2835_REG(BCM2835_GPIO_GPHEN0) |= (1 << dev_pin);
    break;
      
  case NONE:
    break;
  }
}

void bsp_disable_interrupt(int dev_pin, gpio_interrupt enabled_interrupt)
{
  switch ( enabled_interrupt ) {
    case FALLING_EDGE:
      /* Disables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) &= ~(1 << dev_pin);
      break;

    case RISING_EDGE:
      /* Disables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) &= ~(1 << dev_pin);
      break;

    case BOTH_EDGES:
      /* Disables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) &= ~(1 << dev_pin);

      /* Disables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) &= ~(1 << dev_pin);
      break;

    case LOW_LEVEL:
      /* Disables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) &= ~(1 << dev_pin);
      break;

    case HIGH_LEVEL:
      /* Disables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) &= ~(1 << dev_pin);
      break;

    case BOTH_LEVELS:
      /* Disables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) &= ~(1 << dev_pin);

      /* Disables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) &= ~(1 << dev_pin);
      break;
      
    case NONE:
      break;
  }
}

/**
 * @brief Setups a JTAG interface using the P1 GPIO pin header
 *        for the models A/B and J8 header on the B+. 
 *        The following pins should be unused before calling this function:
 *        GPIO 4, 22, 24, 25 and 27.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL JTAG interface successfully configured.
 * @retval RTEMS_RESOURCE_IN_USE At least one of the required pins is currently
 *                               occupied, @see gpio_select_pin().
 */
rtems_status_code gpio_select_jtag(void)
{
  rtems_status_code sc;

  int jtag_pins[] = {4, 22, 24, 25, 27};
  
  if ( (sc = gpio_setup_input_mode(jtag_pins, 5, NO_PULL_RESISTOR)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }
  
  /* Setup gpio 4 alt5 ARM_TDI. */
  if ( (sc = gpio_select_pin(4, ALT_FUNC_5)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* Setup gpio 22 alt4 ARM_TRST. */
  if ( (sc = gpio_select_pin(22, ALT_FUNC_4)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* Setup gpio 24 alt4 ARM_TDO. */
  if ( (sc = gpio_select_pin(24, ALT_FUNC_4)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* Setup gpio 25 alt4 ARM_TCK. */
  if ( (sc = gpio_select_pin(25, ALT_FUNC_4)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }
  
  /* Setup gpio 27 alt4 ARM_TMS. */
  if ( (sc = gpio_select_pin(27, ALT_FUNC_4)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }
    
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Setups a SPI interface using the P1 GPIO pin header
 *        for the models A/B and J8 header on the B+. 
 *        The following pins should be unused before calling this function:
 *        GPIO 7, 8, 9, 10 and 11.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL SPI interface successfully configured.
 * @retval RTEMS_RESOURCE_IN_USE At least one of the required pins is currently
 *                               occupied, @see gpio_select_pin().
 */
rtems_status_code gpio_select_spi_p1(void)
{
  rtems_status_code sc;

  /* SPI master 0 MISO data line. */
  if ( (sc = gpio_select_pin(9, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* SPI master 0 MOSI data line. */
  if ( (sc = gpio_select_pin(10, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* SPI master 0 SCLK clock line. */
  if ( (sc = gpio_select_pin(11, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* SPI master 0 CE_0 chip enable line. */
  if ( (sc = gpio_select_pin(8, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* SPI master 0 CE_1 chip enable line. */
  if ( (sc = gpio_select_pin(7, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }
    
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Setups a I2C interface using the P1 GPIO pin header
 *        for the models A/B and J8 header on the B+. 
 *        The following pins should be unused before calling this function:
 *        GPIO 2 and 3.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL JTAG interface successfully configured.
 * @retval RTEMS_RESOURCE_IN_USE At least one of the required pins is currently
 *                               occupied, @see gpio_select_pin().
 */
rtems_status_code gpio_select_i2c_p1_rev2(void)
{
  rtems_status_code sc;
  int pins[] = {2,3};

  /* I2C BSC1 SDA data line. */
  if ( (sc = gpio_select_pin(2, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* I2C BSC1 SCL clock line. */
  if ( (sc = gpio_select_pin(3, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* Enable pins 2 and 3 pull-up resistors. */
  if ( (sc = gpio_setup_input_mode(pins, 2, PULL_UP)) != RTEMS_SUCCESSFUL ) {
    return sc;
  }
    
  return RTEMS_SUCCESSFUL;
}
