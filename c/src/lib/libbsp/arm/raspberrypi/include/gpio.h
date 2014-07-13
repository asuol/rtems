/**
 * @file
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Raspberry Pi specific GPIO information.
 */

/*
 * Copyright (c) 2014 Andre Marques.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_ARM_RASPBERRYPI_GPIO_H
#define LIBBSP_ARM_RASPBERRYPI_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define GPIO_PIN_COUNT 54

#define JTAG_PIN_COUNT 5

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
} rpi_gpio_input_mode;

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
} rpi_gpio_output_mode;

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
  rpi_pin pin_type;

  /* Interrupt handler arguments*/
  handler_arguments h_args;

  gpio_interrupt enabled_interrupt;

  /* The pin mode */
  union
  {
    rpi_gpio_input_mode input;
    rpi_gpio_output_mode output;
  }mode;
} rpi_gpio_pin;

/**
 * @brief Object defining a specific pin arrangement.
 *
 * Defines a pin setup.
 */
typedef struct
{
  int pin_number;
  
  rpi_pin pin_function;
} gpio_configuration;

/** @} */

/**
 * @name gpio Usage
 *
 * @{
 */

/**
 * @brief gpio initialization.
 */

/* Initializes the API */
extern void gpio_initialize(void);

/* Turns on the given pin */
extern int gpio_set(int pin);

/* Turns off the given pin */
extern int gpio_clear(int pin);

/* Returns the current value of a GPIO pin */
extern int gpio_get_val(int pin);

/* Selects a GPIO pin for a specific type */
extern int gpio_select_pin(int pin, rpi_pin type);

/* Setups a JTAG pin configuration */
extern int gpio_select_jtag(void);

/* Sets a GPIO input pin mode */
extern int gpio_input_mode(int pin, rpi_gpio_input_mode mode);

/* Sets a GPIO input pin mode for a number of pins */
extern int gpio_setup_input_mode(int *pin, int pin_count, rpi_gpio_input_mode mode);

/* Sets a GPIO output pin mode */
extern int gpio_output_mode(int pin, rpi_gpio_output_mode mode);

/* Configures a GPIO pin as NOT_USED */
extern void gpio_disable_pin(int pin);

/* Debouces a switch by requiring a number of clock ticks to pass between interruts */
extern int gpio_debounce_switch(int pin, int ticks);

/* Enables interrupts on the given GPIO pin */
extern int gpio_enable_interrupt(int dev_pin, gpio_interrupt interrupt, void (*handler) (void));

/* Disables any interrupt enabled on the given GPIO pin */
extern int gpio_disable_interrupt(int dev_pin);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_RASPBERRYPI_GPIO_H */
