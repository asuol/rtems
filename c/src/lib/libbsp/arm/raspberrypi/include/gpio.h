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
} gpio_pull_mode;

/**
 * @brief The set of possible functions a pin can have.
 *
 * Enumerated type to define a pin function.
 */
typedef enum
{
  DIGITAL_INPUT = 0,
  DIGITAL_OUTPUT,
  BSP_SPECIFIC,
  NOT_USED
} gpio_function;

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

  typedef struct
  {
    /* The bsp defined function code. */
    uint32_t io_type;
 
    void* pin_data;
  } gpio_specific_data;
  
typedef struct
{
  /* I/O function code, specified by the BSP. */
  uint32_t io_type;

  /* Pointer to the function which setups this I/O type. */ //TODO: Function should return if the I/0 cannot be performed 
  rtems_status_code (*config_io) (uint32_t bank, uint32_t pin, void* arg);
} gpio_io_type;
  
typedef struct
{
  /* Total number of GPIO pins. */
  uint32_t pin_count;

  /* Number of pins per bank. The last bank may be smaller,
   * depending on the total number of pins. */
  uint32_t pins_per_bank;

  /* List of bsp specific new functions. */
  gpio_io_type* bsp_functions;
} gpio_layout;
  
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
  uint32_t bank_number;
  uint32_t pin_number;
  
  gpio_function pin_type;

  /* Type of event which will trigger an interrupt. */
  gpio_interrupt enabled_interrupt;

  /* If more than one interrupt handler is to be used, a task will be created
   * to call each handler sequentially. */
  rtems_id task_id;

  /* ISR shared flag. */
  gpio_handler_flag handler_flag;
  
  /* Linked list of interrupt handlers. */
  gpio_handler_list *handler_list;

  /* GPIO input (pull resistor) pin mode. */
  gpio_pull_mode input_mode;

  /* Switch-deboucing information. */
  int debouncing_tick_count;
  rtems_interval last_isr_tick;

  /* Struct with bsp specific data.
   * If function == BSP_SPECIFIC this should have a pointer to
   * a gpio_specific_data struct.
   *
   * If not this field may be NULL. This is passed to the bsp function so any bsp specific data
   * can be passed to it through this pointer. */
  void* bsp_specific;
  
} gpio_pin;
 
/** @} */

/**
 * @name gpio Usage
 *
 * @{
 */

/**
 * @brief Initializes the GPIO API.
 */
extern rtems_status_code gpio_initialize(void);

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
extern rtems_status_code gpio_select_pin(int pin, gpio_function type);

extern rtems_status_code gpio_select_pin_group(int* pin_group, int pin_count, gpio_function type);
  
/**
 * @brief Configures a input GPIO pin pull-up resistor.
 */
extern rtems_status_code gpio_input_mode(int pin, gpio_pull_mode mode);

extern rtems_status_code gpio_input_pin_group(int* pin_group, int pin_count, gpio_pull_mode mode);
  
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


extern gpio_layout bsp_gpio_initialize(void);
  
/**
 * @brief Turns on the given pin. This must be implemented by each BSP.
 */
extern rtems_status_code bsp_gpio_set(int bank, int pin);

/**
 * @brief Turns off the given pin. This must be implemented by each BSP.
 */
extern rtems_status_code bsp_gpio_clear(int bank, int pin);

extern int bsp_gpio_get_value(int bank, int pin);

extern rtems_status_code bsp_gpio_select(int bank, int pin, gpio_function type);

extern rtems_status_code bsp_gpio_set_input_mode(int bank, int pin, gpio_pull_mode mode);

  // Should return a 32 bit bitmask, where a bit set indicates an active interrupt on that pin
extern uint32_t bsp_gpio_interrupt_line(rtems_vector_number vector);

  extern void bsp_gpio_clear_interrupt_line(rtems_vector_number vector, uint32_t event_status);

extern rtems_vector_number bsp_gpio_get_vector(int bank);
  
/**
 * @brief Disables the interrupt vector. This must be implemented by each BSP.
 */
extern void interrupt_vector_disable(void);

/**
 * @brief Enables the interrupt vector. This must be implemented by each BSP.
 */
extern void interrupt_vector_enable(void);

/**
 * @brief Enables interrupt on hardware. This must be implemented by each BSP.
 */
extern rtems_status_code bsp_enable_interrupt(int dev_pin, gpio_interrupt interrupt);
  
/**
 * @brief Disables interrupt on hardware. This must be implemented by each BSP.
 */
extern rtems_status_code bsp_disable_interrupt(int dev_pin, gpio_interrupt enabled_interrupt);
  
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_RASPBERRYPI_GPIO_H */
