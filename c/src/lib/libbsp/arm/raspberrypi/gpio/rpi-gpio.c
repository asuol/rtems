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
#include <bsp/irq-generic.h>
#include <bsp/gpio.h>
#include <bsp/rpi-gpio.h>

#include <stdlib.h>

/* Calculates a bitmask to assign an alternate function to a given pin. */
#define SELECT_PIN_FUNCTION(fn, pn) (fn << ((pn % 10) * 3))

#define ELEM_COUNT(i) sizeof(i) / sizeof(i[0])

typedef struct
{
  int pin;
  int gpio_function;
} rpi_gpio_pin;

/* GPIO interfaces pins. */
static rpi_gpio_pin jtag_pins[] = {
  {.pin = 4, .gpio_function = RPI_ALT_FUNC_5},
  {.pin = 22, .gpio_function = RPI_ALT_FUNC_4},
  {.pin = 24, .gpio_function = RPI_ALT_FUNC_4},
  {.pin = 25, .gpio_function = RPI_ALT_FUNC_4},
  {.pin = 27, .gpio_function = RPI_ALT_FUNC_4}
};

static int spi_p1_pins[] = {7, 8, 9, 10, 11};
static int i2c_p1_rev2_pins[] = {2, 3};

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

gpio_layout bsp_gpio_initialize()
{
  gpio_layout rpi_layout;
  gpio_io_type* rpi_functions;

  rpi_functions = (gpio_io_type *) malloc(6 * sizeof(gpio_io_type));
  
  rpi_functions[0].io_type = RPI_ALT_FUNC_0;
  rpi_functions[0].config_io = NULL;
  
  rpi_functions[1].io_type = RPI_ALT_FUNC_1;
  rpi_functions[1].config_io = NULL;

  rpi_functions[2].io_type = RPI_ALT_FUNC_2;
  rpi_functions[2].config_io = NULL;

  rpi_functions[3].io_type = RPI_ALT_FUNC_3;
  rpi_functions[3].config_io = NULL;

  rpi_functions[4].io_type = RPI_ALT_FUNC_4;
  rpi_functions[4].config_io = NULL;

  rpi_functions[5].io_type = RPI_ALT_FUNC_5;
  rpi_functions[5].config_io = NULL;
  
  rpi_layout.pin_count = 54;
  rpi_layout.pins_per_bank = 54;
  rpi_layout.bsp_functions = rpi_functions;

  return rpi_layout;
}

rtems_status_code bsp_gpio_set(int bank, int pin)
{
  BCM2835_REG(BCM2835_GPIO_GPSET0) = (1 << pin);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_gpio_clear(int bank, int pin)
{
  BCM2835_REG(BCM2835_GPIO_GPCLR0) = (1 << pin);

  return RTEMS_SUCCESSFUL;
}

int bsp_gpio_get_value(int bank, int pin)
{
  return (BCM2835_REG(BCM2835_GPIO_GPLEV0) & (1 << pin));
}

rtems_status_code bsp_gpio_select(int bank, int pin, gpio_function type)
{
  /* Calculate the pin function select register address. */
  volatile unsigned int *pin_addr = (unsigned int *)BCM2835_GPIO_REGS_BASE +
                                    (pin / 10);

  /* Sets pin function select bits.*/
  if ( type == DIGITAL_INPUT ) {
    *(pin_addr) &= ~SELECT_PIN_FUNCTION(7, pin);
  }
  else {
    *(pin_addr) |= SELECT_PIN_FUNCTION(type, pin);
  }

  return RTEMS_SUCCESSFUL;
}

// should return UNSTATISFIED IF FAILED
rtems_status_code bsp_gpio_set_input_mode(int bank, int pin, gpio_pull_mode mode)
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

rtems_vector_number bsp_gpio_get_vector(int bank)
{
  return BCM2835_IRQ_ID_GPIO_0;
}

uint32_t bsp_gpio_interrupt_line(rtems_vector_number vector)
{
  // TODO: get event register

  return BCM2835_REG(BCM2835_GPIO_GPEDS0);
}

void bsp_gpio_clear_interrupt_line(rtems_vector_number vector, uint32_t event_status)
{
  // TODO: get event register
  
  BCM2835_REG(BCM2835_GPIO_GPEDS0) = event_status;
}

void interrupt_vector_disable(void)// TODO: return the vector ID instead
{
  bsp_interrupt_vector_disable(BCM2835_IRQ_ID_GPIO_0);
}

void interrupt_vector_enable(void)// TODO: return the vector ID instead
{
  bsp_interrupt_vector_enable(BCM2835_IRQ_ID_GPIO_0);
}

rtems_status_code bsp_enable_interrupt(int dev_pin, gpio_interrupt interrupt)
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

  return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_disable_interrupt(int dev_pin, gpio_interrupt enabled_interrupt)
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

  return RTEMS_SUCCESSFUL;
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
  int pin_count;
  int i;

  pin_count = ELEM_COUNT(jtag_pins);

  /* alt5 ARM_TDI. 
   * alt4 ARM_TRST. 
   * alt4 ARM_TDO.
   * alt4 ARM_TCK. 
   * alt4 ARM_TMS. */
  for ( i = 0; i < pin_count; ++i ) {
    sc = gpio_select_pin(jtag_pins[i].pin, jtag_pins[i].gpio_function);

    if ( sc != RTEMS_SUCCESSFUL ) {
      return sc;
    }
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
  int pin_count;

  pin_count = ELEM_COUNT(spi_p1_pins);

  /* SPI master 0 MISO data line. 
   * SPI master 0 MOSI data line. 
   * SPI master 0 SCLK clock line.
   * SPI master 0 CE_0 chip enable line. 
   * SPI master 0 CE_1 chip enable line. */
  sc = gpio_select_pin_group(spi_p1_pins, pin_count, RPI_ALT_FUNC_0);

  return sc;
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
  int pin_count;

  pin_count = ELEM_COUNT(i2c_p1_rev2_pins);

  /* I2C BSC1 SDA data line. 
   * I2C BSC1 SCL clock line. */
  sc = gpio_select_pin_group(i2c_p1_rev2_pins, pin_count, RPI_ALT_FUNC_0);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = gpio_input_pin_group(i2c_p1_rev2_pins, pin_count, PULL_UP);

  return sc;
}
