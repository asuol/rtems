/**
 * @file gpio.h
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Raspberry Pi specific GPIO definitions.
 */

/*
 *  Copyright (c) 2014-2015 Andre Marques <andre.lousa.marques at gmail.com>
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
 * @brief  Total number of GPIOs on the Raspberry Pi,
 *         including physical GPIO pins (available on the board hardware)
 *         and system GPIOs (only accessible to the system).
 */
#define GPIO_COUNT 54

/**
 * @brief  Highest GPIO index physically accessible on the Raspberry Pi board.
 */
#define GPIO_PHYSICAL_PIN_COUNT 32

/**
 * @brief GPIO API mutex atributes.
 */
#define MUTEX_ATRIBUTES                         \
  ( RTEMS_LOCAL                                 \
    | RTEMS_PRIORITY                            \
    | RTEMS_BINARY_SEMAPHORE                    \
    | RTEMS_INHERIT_PRIORITY                    \
    | RTEMS_NO_PRIORITY_CEILING                 \
    )

/**
 * @brief The set of possible configurations for a GPIO pull-up resistor.
 *
 * Enumerated type to define the possible pull-up resistor configuratons 
 * for an input pin.
 */
typedef enum
{
  PULL_UP = 1,
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
  DIGITAL_INPUT = 0,
  DIGITAL_OUTPUT,
  ALT_FUNC_5,
  ALT_FUNC_4,
  ALT_FUNC_0,
  ALT_FUNC_1,
  ALT_FUNC_2,
  ALT_FUNC_3,
  NOT_USED
} rpi_pin;

/**
 * @brief The set of possible interrupts a GPIO pin can generate.
 *
 * Enumerated type to define a GPIO pin interrupt.
 */
typedef enum
{
  FALLING_EDGE = 0,
  RISING_EDGE,
  LOW_LEVEL,
  HIGH_LEVEL,
  BOTH_EDGES,
  BOTH_LEVELS,
  NONE
} gpio_interrupt;

/**
 * @brief The set of possible handled states an user-defined interrupt
 *        handler can return.
 *
 * Enumerated type to define an interrupt handler handled state.
 */  
typedef enum
{
  IRQ_HANDLED,
  IRQ_NONE
} gpio_irq_state;

/**
 * @brief The set of flags to specify an user-defined interrupt handler
 *        uniqueness on a GPIO pin.
 *
 * Enumerated type to define an interrupt handler shared flag.
 */  
typedef enum
{
  SHARED_HANDLER,
  UNIQUE_HANDLER
} gpio_handler_flag;
  
/**
 * @brief Object containing relevant information to a list of user-defined
 *        interrupt handlers.
 *
 * Encapsulates relevant data for a GPIO interrupt handler.
 */
typedef struct _gpio_handler_list
{
  struct _gpio_handler_list *next_isr;

  gpio_irq_state (*handler) (void *arg);

  void *arg;
  
} gpio_handler_list;

/**
 * @brief Object containing information on a GPIO pin.
 *
 * Encapsulates relevant data about a GPIO pin.
 */
typedef struct
{ 
  rpi_pin pin_type;

  /* Type of event which will trigger an interrupt. */
  gpio_interrupt enabled_interrupt;

  /* If more than one interrupt handler is to be used, a task will be created
   * to call each handler sequentially. */
  rtems_id task_id;

  gpio_handler_flag handler_flag;
  
  /* Linked list of interrupt handlers. */
  gpio_handler_list *handler_list;

  /* GPIO input pin mode. */
  rpi_gpio_input_mode input_mode;

  /* Switch-deboucing information. */
  int debouncing_tick_count;
  rtems_interval last_isr_tick;
  
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
extern rtems_status_code gpio_set(int pin);

/**
 * @brief Turns off the given pin.
 */
extern rtems_status_code gpio_clear(int pin);

/**
 * @brief Returns the current value of a GPIO pin.
 */
extern int gpio_get_value(int pin);

/**
 * @brief Selects a GPIO pin for a specific function.
 */
extern rtems_status_code gpio_select_pin(int pin, rpi_pin type);

/**
 * @brief Setups a JTAG pin configuration.
 */
extern rtems_status_code gpio_select_jtag(void);

/**
 * @brief Setups the SPI interface on the RPI P1 GPIO header.
 */
extern rtems_status_code gpio_select_spi_p1(void);

/**
 * @brief Setups the I2C interface on the main (P1) GPIO pin header (rev2).
 */
extern rtems_status_code gpio_select_i2c_p1_rev2(void); 

/**
 * @brief Configures a input GPIO pin pull-up resistor.
 */
extern rtems_status_code gpio_input_mode(int pin, rpi_gpio_input_mode mode);

/**
 * @brief Configures several input GPIO pins to the same pull-up resistor setup.
 */
extern rtems_status_code 
gpio_setup_input_mode(int *pin, int pin_count, rpi_gpio_input_mode mode);

/**
 * @brief Discards any configuration made on this pin.
 */
extern rtems_status_code gpio_disable_pin(int dev_pin);

/**
 * @brief Debouces a switch by requiring a number of clock ticks to 
 *        pass between interruts.
 */
extern rtems_status_code gpio_debounce_switch(int pin, int ticks);

/**
 * @brief Connects a new user-defined interrupt handler to a given pin.
 */
extern rtems_status_code gpio_interrupt_handler_install(
int dev_pin,
gpio_irq_state (*handler) (void *arg),
void *arg
);
  
/**
 * @brief Enables interrupts to be generated on a given GPIO pin.
 *        When fired that interrupt will call the given handler.
 */
extern rtems_status_code gpio_enable_interrupt(
int dev_pin, 
gpio_interrupt interrupt,
gpio_handler_flag flag,
gpio_irq_state (*handler) (void *arg),
void *arg
);

/**
 * @brief Disconnects a new user-defined interrupt handler from the given pin.
 *        If in the end there are no more user-defined handler connected
 *        to the pin interrupts are disabled on the given pin.
 */
rtems_status_code gpio_interrupt_handler_remove(
int dev_pin,
gpio_irq_state (*handler) (void *arg),
void *arg
);
  
/**
 * @brief Disables any interrupt enabled on the given GPIO pin.
 */
extern rtems_status_code gpio_disable_interrupt(int dev_pin);

/**
 * @brief Disables the interrupt vector. This must be implemented by each BSP.
 */
extern void interrupt_vector_disable(void);

/**
 * @brief Enables the interrupt vector. This must be implemented by each BSP.
 */
extern void interrupt_vector_enable(void);

/**
 * @brief Disables interrupt on hardware. This must be implemented by each BSP.
 */
extern void bsp_disable_interrupt(int dev_pin, gpio_interrupt enabled_interrupt);
  
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_RASPBERRYPI_GPIO_H */
