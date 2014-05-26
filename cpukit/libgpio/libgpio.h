#ifndef _RTEMS_LIBGPIO_H
#define _RTEMS_LIBGPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_INPUT 0
#define GPIO_OUTPUT 1

typedef struct rtems_gpio_pin_t_
{
    /* GPIO pin address */
    uint32_t address;

    /* GPIO direction */
    int direction;

    /* GPIO pull resistor */
    int pull;

    /* GPIO pad drive strength (in mA) */
    int drive_strength;

    /* Enabled interrupts */
    bool falling_edge;
    bool rising_edge;
    bool both_edges;

    bool low_level;
    bool high_level;
    bool both_levels;

    /* asynchronous rising/falling edge interrups -> not sure if RPi specific */
} rtems_gpio_pin_t;

typedef struct rtems_gpio_port_t_
{
    /* GPIO pin count */
    int pin_count;
 
    /* GPIO port */
    rtems_gpio_pin_t *port;
} rtems_gpio_port_t;

typedef struct rtems_gpio_config_t_
{
    /* GPIO port (group of GPIO pins) count */
    int gpio_port_count;
 
    /* GPIO ports */
    rtems_gpio_port_t *gpio_port;
    
} rtems_gpio_config_t;

extern int rtems_gpio_initialize ( void );

extern void rtems_gpio_set_direction ( int pin, int dir );

extern void rtems_gpio_set ( int pin, int dir );

extern void rtems_gpio_clear ( int pin, int dir );

#ifdef __cplusplus
    }
#endif
