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
  uint32_t io_function;
 
  void* pin_data;
} gpio_specific_data;
    
typedef struct
{
  /* Total number of GPIO pins. */
  uint32_t pin_count;

  /* Number of pins per bank. The last bank may be smaller,
   * depending on the total number of pins. */
  uint32_t pins_per_bank;
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

typedef struct
{
  gpio_interrupt enabled_interrupt;
 
  gpio_handler_flag handler_flag;

  /* Interrupt handler function. */
  gpio_irq_state (*handler) (void *arg);
  
  /* Interrupt handler function arguments. */
  void *arg;
 
  /* Software switch debounce settings. It should contain the amount of clock
   * ticks that must pass between interrupts to ensure that the interrupt
   * was not caused by a switch bounce. If set to 0 this feature is disabled . */
  uint32_t clock_tick_interval;
} gpio_interrupt_conf;
  
typedef struct
{
  uint32_t pin_number;
  gpio_function function;
 
  /* Pull resistor setting. */
  gpio_pull_mode pull_mode;
 
  /* Pin interrupt configuration. Should be NULL if not used. */
  gpio_interrupt_conf* interrupt;

  /* If digital out pin, set to TRUE to set the pin to logical high,
   * or FALSE for logical low. If not a digital out then this
   * is ignored. */
  bool output_enabled;
  bool invert_output;
 
  /* Struct with bsp specific data, to use during the pin request.
   * If function == BSP_SPECIFIC this should have a pointer to
   * a gpio_specific_data struct.
   *
   * If not this field may be NULL. This is passed to the bsp function so any bsp specific data
   * can be passed to it through this pointer. */
  void* bsp_specific;
} gpio_pin_conf;
  
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
 * @brief Requests/configures a pin based on a gpio_pin_conf struct.
 */
extern rtems_status_code gpio_request_conf(gpio_pin_conf* conf);
  
/**
 * @brief Sets an output GPIO pin with the logical high.
 */
extern rtems_status_code gpio_set(uint32_t pin_number);

/**
 * @brief Sets an output GPIO pin with the logical low.
 */
extern rtems_status_code gpio_clear(uint32_t pin_number);

/**
 * @brief Returns the value (level) of a GPIO input pin.
 */
extern int gpio_get_value(uint32_t pin_number);

/**
 * @brief Assigns a certain function to a GPIO pin.
 */
extern rtems_status_code gpio_request_pin(uint32_t pin_number, gpio_function type, void* bsp_specific);
  
/**
 * @brief Configures a single GPIO pin pull resistor. 
 */
extern rtems_status_code gpio_input_mode(uint32_t pin_number, gpio_pull_mode mode);
  
/**
 * @brief Releases a GPIO pin from the API, making it available to be used 
 *        again.
 */
extern rtems_status_code gpio_release_pin(uint32_t pin_number);

/**
 * @brief Debouces a switch by requiring a number of clock ticks to 
 *        pass between interruts.
 */
extern rtems_status_code gpio_debounce_switch(uint32_t pin_number, int ticks);

/**
 * @brief Connects a new user-defined interrupt handler to a given pin.
 */
extern rtems_status_code gpio_interrupt_handler_install(
uint32_t pin_number,
gpio_irq_state (*handler) (void *arg),
void *arg
);
  
/**
 * @brief Enables interrupts to be generated on a given GPIO pin.
 *        When fired that interrupt will call the given handler.
 */
extern rtems_status_code gpio_enable_interrupt(
uint32_t pin_number, 
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
uint32_t pin_number,
gpio_irq_state (*handler) (void *arg),
void *arg
);
  
/**
 * @brief Disables any interrupt enabled on the given GPIO pin.
 */
extern rtems_status_code gpio_disable_interrupt(uint32_t pin_number);

/**
 * @brief Defines the GPIO pin layout. This must be implemented by each BSP.
 */
extern gpio_layout bsp_gpio_initialize(void);
  
/**
 * @brief Sets an output GPIO pin with the logical high. This must be implemented by each BSP.
 */
extern rtems_status_code bsp_gpio_set(uint32_t bank, uint32_t pin);

/**
 * @brief Sets an output GPIO pin with the logical low. This must be implemented by each BSP.
 */
extern rtems_status_code bsp_gpio_clear(uint32_t bank, uint32_t pin);

/**
 * @brief Returns the value (level) of a GPIO input pin. This must be implemented by each BSP.
 */
extern int bsp_gpio_get_value(uint32_t bank, uint32_t pin);

extern rtems_status_code bsp_gpio_select_input(uint32_t bank, uint32_t pin, void* bsp_specific);

extern rtems_status_code bsp_gpio_select_output(uint32_t bank, uint32_t pin, void* bsp_specific);

extern rtems_status_code bsp_select_specific_io(uint32_t bank, uint32_t pin, uint32_t function, void* pin_data);

// should return UNSTATISFIED IF FAILED
extern rtems_status_code bsp_gpio_set_input_mode(uint32_t bank, uint32_t pin, gpio_pull_mode mode);

/**
 * @brief Returns a bitmask (max 32-bit) representing a GPIO bank, where a bit set indicates an active interrupt on that pin. This must be implemented by each BSP.
 */
extern uint32_t bsp_gpio_interrupt_line(rtems_vector_number vector);

extern void bsp_gpio_clear_interrupt_line(rtems_vector_number vector, uint32_t event_status);

extern rtems_vector_number bsp_gpio_get_vector(uint32_t bank);

/**
 * @brief Enables interrupt on hardware. This must be implemented by each BSP.
 */
extern rtems_status_code bsp_enable_interrupt(uint32_t pin, gpio_interrupt interrupt);
  
/**
 * @brief Disables interrupt on hardware. This must be implemented by each BSP.
 */
extern rtems_status_code bsp_disable_interrupt(uint32_t pin, gpio_interrupt enabled_interrupt);
  
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_RASPBERRYPI_GPIO_H */
