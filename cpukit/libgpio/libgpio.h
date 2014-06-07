#ifndef _RTEMS_LIBGPIO_H
#define _RTEMS_LIBGPIO_H

#include <stdint.h>
#include <stdbool.h>

#ifndef RTEMS_GPIO_COUNT
#error "GPIO pin count not defined."
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  PULL_UP,
  PULL_DOWN,
  NO_PULL_RESISTOR
} rtems_multiio_input_mode_t;

typedef enum
{
  PUSH_PULL,
  OPEN_DRAIN,
  NEUTRAL
} rtems_multiio_output_mode_t;

typedef enum
{
  DIGITAL_INPUT,
  DIGITAL_OUTPUT,
  NOT_USED
} rtems_pin_t;

typedef struct
{
  /* Pin mode */
  rtems_multiio_output_mode_t mode;

  /* Drive strength */
  void *drive_strength;
} rtems_dout_pin_t;

typedef struct
{
  /* Pin mode */
  rtems_multiio_input_mode_t mode;

  /* Enabled interrupts */
  bool falling_edge;
  bool rising_edge;
  bool both_edges;

  bool low_level;
  bool high_level;
  bool both_levels;
} rtems_din_pin_t;

typedef struct
{ 
  /* The address that controls the pin */ 
  void *address;

  /* The pin type */
  rtems_pin_t pin_type;

  /* The pin data */
  union
  {
    rtems_din_pin_t din;
    rtems_dout_pin_t dout;
  }pin_data;
} rtems_gpio_pin_t;

/* GPIO pin array */
rtems_gpio_pin_t gpio_pin[RTEMS_GPIO_COUNT];

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
extern int rtems_gpio_setup_pin (int pin, rtems_pin_t type);

/* Sets a GPIO input pin mode */
extern int rtems_gpio_input_mode (int pin, rtems_multiio_input_mode_t mode);

/* Sets a GPIO output pin mode */
extern int rtems_gpio_output_mode (int pin, rtems_multiio_output_mode_t mode);

/* Configures a GPIO pin as NOT_USED */
extern void rtems_gpio_disable_pin (int pin);

#ifdef __cplusplus
    }
#endif

#endif /* _RTEMS_LIBGPIO_H */
