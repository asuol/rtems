#ifndef _RTEMS_LIBGPIO_H
#define _RTEMS_LIBGPIO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  PULL_UP,
  PULL_DOWN,
  NO_PULL_RESISTOR
} rtems_multiio_input_mode;

typedef enum
{
  PUSH_PULL,
  OPEN_DRAIN,
  NEUTRAL
} rtems_multiio_output_mode;

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

typedef struct
{
  /* Pin mode */
  rtems_multiio_output_mode mode;

  /* Drive strength */
  void *drive_strength;
} rtems_dout_pin;

typedef struct
{
  /* Pin mode */
  rtems_multiio_input_mode mode;

  /* Enabled interrupts */
  bool falling_edge;
  bool rising_edge;
  bool both_edges;

  bool low_level;
  bool high_level;
  bool both_levels;
} rtems_din_pin;

typedef struct
{ 
  /* The address that controls the pin */ 
  void *address;

  /* The pin type */
  rtems_pin pin_type;

  /* The pin data */
  union
  {
    rtems_din_pin din;
    rtems_dout_pin dout;
  }pin_data;
} rtems_gpio_pin;

typedef struct
{
  int pin_number;
  
  rtems_pin pin_function;
}rtems_gpio_configuration;

/* GPIO pin array, to be setup on the rtems_gpio_initialize function */
extern rtems_gpio_pin *gpio_pin;

/* Initializes the API */
extern void rtems_gpio_initialize (void);

/* Turns on the masked pins on the given port */
extern int rtems_gpio_set_mb (int port, int mask);

/* Turns on the given pin */
extern int rtems_gpio_set (int pin);

/* Turns off the masked pins on the given port */
extern int rtems_gpio_clear_mb (int port, int mask);

/* Turns off the given pin */
extern int rtems_gpio_clear (int pin);

/* Returns the current value of a GPIO pin */
extern int rtems_gpio_get_val (int pin);

/* Configures a GPIO pin to a given setup */
extern int rtems_gpio_setup_pin (int pin, rtems_pin type);

/* Setups a number of GPIO pins, each with a specific function */
extern int rtems_gpio_setup_config (rtems_gpio_configuration *pin_setup, int pin_count);

/* Sets a GPIO input pin mode */
extern int rtems_gpio_input_mode (int pin, rtems_multiio_input_mode mode);

/* Sets a GPIO input pin mode for a number of pins */
extern int rtems_gpio_setup_input_mode (int *pin, int pin_count, rtems_multiio_input_mode mode);

/* Sets a GPIO output pin mode */
extern int rtems_gpio_output_mode (int pin, rtems_multiio_output_mode mode);

/* Configures a GPIO pin as NOT_USED */
extern void rtems_gpio_disable_pin (int pin);

#ifdef __cplusplus
    }
#endif

#endif /* _RTEMS_LIBGPIO_H */
