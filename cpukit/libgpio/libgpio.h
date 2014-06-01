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
} rtems_multiio_input_mode_t;

typedef enum
{
  PUSH_PULL,
  OPEN_DRAIN
} rtems_multiio_output_mode_t;

typedef struct
{ 
  /* The address that controls the pin */ 
  void *address;

  /* The adc, dac, din, dout... identifier */ 
  int pin_id;
} rtems_multiio_pin_t;

typedef struct
{
  /* Multiio pin */
  rtems_multiio_pin_t pin;

  /* Pin mode */
  rtems_multiio_output_mode_t mode;

  /* Drive strength (in mA) */
  int drive_strength;
} rtems_dout_pin_t;

typedef struct
{
  /* Multiio pin */
  rtems_multiio_pin_t pin;

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

/* Writes one to the masked pins on the given port */
extern void rtems_gpio_set_mb ( int port, int mask );

/* Writes one to the given pin */
extern void rtems_gpio_set_sb ( int pin );

/* Writes zero to the masked pins on the given port */
extern void rtems_gpio_clear_mb ( int port, int mask );

/* Writes zero to the given pin */
extern void rtems_gpio_clear_sb ( int pin );

#ifdef __cplusplus
    }
#endif

#endif /* _RTEMS_LIBGPIO_H */
