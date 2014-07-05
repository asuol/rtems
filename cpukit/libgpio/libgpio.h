/**
 * @file
 *
 * @ingroup LibGPIO
 *
 * @brief libgpio API.
 *
 * Provides an API to control GPIO hardware
 */

/*
 * Copyright (c) 2014 Andre Marques.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef _RTEMS_LIBGPIO_H
#define _RTEMS_LIBGPIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief The set of possible input drive modes.
 *
 * Enumerated type to define the drive modes for an input pin.
 */
typedef enum
{
  PULL_UP,
  PULL_DOWN,
  NO_PULL_RESISTOR
} rtems_multiio_input_mode;

/**
 * @brief The set of possible output drive modes.
 *
 * Enumerated type to define the drive modes for an output pin.
 */
typedef enum
{
  PUSH_PULL,
  OPEN_DRAIN,
  NEUTRAL
} rtems_multiio_output_mode;

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
} rtems_pin;

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
} rtems_gpio_interrupt;

typedef struct
{
  int pin_number;

  void (*handler) (void);

  int debouncing_tick_count;

  rtems_interval last_isr_tick;
} handler_arguments;

/**
 * @brief Object containing information on a generic pin.
 *
 * Encapsulates relevant data about any type of pin.
 */
typedef struct
{ 
  /* The pin type */
  rtems_pin pin_type;

  /* Interrupt handler arguments*/
  handler_arguments h_args;

  rtems_gpio_interrupt enabled_interrupt;

  /* The pin mode */
  union
  {
    rtems_multiio_input_mode input;
    rtems_multiio_output_mode output;
  }mode;
} rtems_gpio_pin;

/**
 * @brief Object defining a specific pin arrangement.
 *
 * Defines a pin setup.
 */
typedef struct
{
  int pin_number;
  
  rtems_pin pin_function;
} rtems_gpio_configuration;

/**
 * @name External variables.
 *
 * @{
 */

/* GPIO pin array, to be setup on the rtems_gpio_initialize function */
extern rtems_gpio_pin *gpio_pin;

/** @} */

/**
 * @name libgpio Usage
 *
 * @{
 */

/**
 * @brief libgpio initialization.
 * 
 * @param[in] gpio_count The total ammount of GPIO pins in the target hardware.
 */

/* Initializes the API */
extern void rtems_gpio_initialize(int gpio_count);

/* Turns on the masked pins on the given port */
extern int rtems_gpio_set_mb(int port, int mask);

/* Turns on the given pin */
extern int rtems_gpio_set(int pin);

/* Turns off the masked pins on the given port */
extern int rtems_gpio_clear_mb(int port, int mask);

/* Turns off the given pin */
extern int rtems_gpio_clear(int pin);

/* Returns the current value of a GPIO pin */
extern int rtems_gpio_get_val(int pin);

/* Selects a GPIO pin for a specific type */
extern int rtems_gpio_select_pin(int pin, rtems_pin type);

/* Setups a number of GPIO pins, each with a specific function */
extern int rtems_gpio_select_config(rtems_gpio_configuration *pin_setup, int pin_count);

/* Sets a GPIO input pin mode */
extern int rtems_gpio_input_mode(int pin, rtems_multiio_input_mode mode);

/* Sets a GPIO input pin mode for a number of pins */
extern int rtems_gpio_setup_input_mode(int *pin, int pin_count, rtems_multiio_input_mode mode);

/* Sets a GPIO output pin mode */
extern int rtems_gpio_output_mode(int pin, rtems_multiio_output_mode mode);

/* Configures a GPIO pin as NOT_USED */
extern void rtems_gpio_disable_pin(int pin);

/* Debouces a switch by requiring a number of clock ticks to pass between interruts */
extern int rtems_gpio_debounce_switch(int pin, int ticks);

/* Enables interrupts on the given GPIO pin */
extern int rtems_gpio_enable_interrupt(int dev_pin, rtems_gpio_interrupt interrupt, void (*handler) (void));

/* Disables any interrupt enabled on the given GPIO pin */
extern int rtems_gpio_disable_interrupt(int dev_pin);

/** @} */

#ifdef __cplusplus
    }
#endif /* __cplusplus */

#endif /* _RTEMS_LIBGPIO_H */
