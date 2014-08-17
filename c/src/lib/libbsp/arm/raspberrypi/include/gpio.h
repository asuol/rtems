/**
 * @file gpio.h
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Raspberry Pi specific GPIO definitions.
 */

/*
 *  Copyright (c) 2014 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_RASPBERRYPI_GPIO_H
#define LIBBSP_ARM_RASPBERRYPI_GPIO_H

#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief  Number of total GPIOS on the Raspberry Pi,
 *         including the internal ones.
 */
#define GPIO_PIN_COUNT 54

/**
 * @brief The set of possible configurations for a GPIO pull-up resistor.
 *
 * Enumerated type to define the possible pull-up resistor configuratons 
 * for an input pin.
 */
typedef enum
{
  PULL_UP,
  PULL_DOWN,
  NO_PULL_RESISTOR
} rpi_gpio_input_mode;

/**
 * @brief The set of possible functions a pin can have.
 *
 * Enumerated type to define a pin function.
 */
typedef enum
{
  DIGITAL_INPUT,
  DIGITAL_OUTPUT,
  ALT_FUNC_0,
  ALT_FUNC_1,
  ALT_FUNC_2,
  ALT_FUNC_3,
  ALT_FUNC_4,
  ALT_FUNC_5,
  NOT_USED
} rpi_pin;

/**
 * @brief The set of possible interrupts an input pin can generate.
 *
 * Enumerated type to define an input pin interrupt.
 */
typedef enum
{
  FALLING_EDGE,
  RISING_EDGE,
  BOTH_EDGES,
  LOW_LEVEL,
  HIGH_LEVEL,
  BOTH_LEVELS,
  NONE
} gpio_interrupt;

/**
 * @brief Object containing relevant information to a interrupt handler.
 *
 * Encapsulates relevant data for a GPIO interrupt handler.
 */
typedef struct
{
  int pin_number;

  void (*handler) (void);

  int debouncing_tick_count;
 
  rtems_interval last_isr_tick;

} handler_arguments;

/**
 * @brief Object containing information on a GPIO pin.
 *
 * Encapsulates relevant data about a GPIO pin.
 */
typedef struct
{ 
  /* The pin type */
  rpi_pin pin_type;

  /* Interrupt handler arguments*/
  handler_arguments h_args;

  gpio_interrupt enabled_interrupt;

  /* GPIO input pin mode. */
  rpi_gpio_input_mode input_mode;

} rpi_gpio_pin;
 
/** @} */

/**
 * @name gpio Usage
 *
 * @{
 */

/**
 * @brief Initializes the GPIO API.
 */
extern void gpio_initialize(void);

/**
 * @brief Turns on the given pin.
 */
extern int gpio_set(int pin);

/**
 * @brief Turns off the given pin.
 */
extern int gpio_clear(int pin);

/**
 * @brief Returns the current value of a GPIO pin.
 */
extern int gpio_get_val(int pin);

/**
 * @brief Selects a GPIO pin for a specific function.
 */
extern int gpio_select_pin(int pin, rpi_pin type);

/**
 * @brief Setups a JTAG pin configuration.
 */
extern int gpio_select_jtag(void);

/**
 * @brief Setups the SPI interface on the RPI P1 GPIO header.
 */
extern int gpio_select_spi_p1(void);

/**
 * @brief Setups the I2C interface on the main (P1) GPIO pin header (rev2).
 */
extern int gpio_select_i2c_p1_rev2(void); 

/**
 * @brief Configures a input GPIO pin pull-up resistor.
 */
extern int gpio_input_mode(int pin, rpi_gpio_input_mode mode);

/**
 * @brief Configures several input GPIO pins to the same pull-up resistor setup.
 */
extern int gpio_setup_input_mode(int *pin, int pin_count, rpi_gpio_input_mode mode);

/**
 * @brief Discards any configuration made on this pin.
 */
extern int gpio_disable_pin(int dev_pin);

/**
 * @brief Debouces a switch by requiring a number of clock ticks to pass between interruts.
 */
extern int gpio_debounce_switch(int pin, int ticks);

/**
 * @brief Enables interrupts on the given GPIO pin.
 */
extern int gpio_enable_interrupt(int dev_pin, gpio_interrupt interrupt, void (*handler) (void));

/**
 * @brief Disables any interrupt enabled on the given GPIO pin.
 */
extern int gpio_disable_interrupt(int dev_pin);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_RASPBERRYPI_GPIO_H */
