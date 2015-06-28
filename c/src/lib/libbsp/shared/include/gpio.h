/**
 * @file gpio.h
 *
 * @ingroup rtems_gpio
 *
 * @brief RTEMS GPIO API definition.
 */

/*
 *  Copyright (c) 2014-2015 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifndef LIBBSP_SHARED_GPIO_H
#define LIBBSP_SHARED_GPIO_H

#include <bsp.h>
#include <rtems.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if !defined(BSP_GPIO_PIN_COUNT) || !defined(BSP_GPIO_PINS_PER_BANK)
  #error "BSP_GPIO_PIN_COUNT or BSP_GPIO_PINS_PER_BANK is not defined."
#endif

#if BSP_GPIO_PIN_COUNT <= 0 || BSP_GPIO_PINS_PER_BANK <= 0
  #error "Invalid BSP_GPIO_PIN_COUNT or BSP_GPIO_PINS_PER_BANK. Must be greater than zero."
#endif

#if BSP_GPIO_PINS_PER_BANK > 32
  #error "Invalid BSP_GPIO_PINS_PER_BANK. Must be between (and including) 1 and 32."
#endif

#define GPIO_LAST_BANK_PINS BSP_GPIO_PIN_COUNT % BSP_GPIO_PINS_PER_BANK

#if GPIO_LAST_BANK_PINS > 0
  #define GPIO_BANK_COUNT (BSP_GPIO_PIN_COUNT / BSP_GPIO_PINS_PER_BANK) + 1
#else
  #define GPIO_BANK_COUNT BSP_GPIO_PIN_COUNT / BSP_GPIO_PINS_PER_BANK
#endif

/**
 * @name GPIO data structures
 *
 * @{
 */

/**
 * @brief The set of possible configurations for a GPIO pull-up resistor.
 *
 * Enumerated type to define the possible pull-up resistor configuratons
 * for a GPIO pin.
 */
typedef enum
{
  PULL_UP = 1,
  PULL_DOWN,
  NO_PULL_RESISTOR
} rtems_gpio_pull_mode;

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
} rtems_gpio_function;

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
} rtems_gpio_interrupt;

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
} rtems_gpio_irq_state;

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
} rtems_gpio_handler_flag;

/**
 * @brief Object containing relevant information for assigning a BSP specific
 *        function to a pin.
 *
 * Encapsulates relevant data for a BSP specific GPIO function.
 */
typedef struct
{
  /* The bsp defined function code. */
  uint32_t io_function;

  void* pin_data;
} rtems_gpio_specific_data;

/**
 * @brief Object containing configuration information
 *        regarding interrupts.
 */
typedef struct
{
  rtems_gpio_interrupt active_interrupt;

  rtems_gpio_handler_flag handler_flag;

  /* Interrupt handler function. */
  rtems_gpio_irq_state (*handler) (void *arg);

  /* Interrupt handler function arguments. */
  void *arg;

  /* Software switch debounce settings. It should contain the amount of clock
   * ticks that must pass between interrupts to ensure that the interrupt
   * was not caused by a switch bounce. If set to 0 this feature is disabled . */
  uint32_t clock_tick_interval;
} rtems_gpio_interrupt_conf;

/**
 * @brief Object containing configuration information
 *        to request/update a GPIO pin.
 */
typedef struct
{
  uint32_t pin_number;
  rtems_gpio_function function;

  /* Pull resistor setting. */
  rtems_gpio_pull_mode pull_mode;

  /* If digital out pin, set to TRUE to set the pin to logical high,
   * or FALSE for logical low. If not a digital out then this
   * is ignored. */
  bool output_enabled;
  bool logic_invert;

  /* Pin interrupt configuration. Should be NULL if not used. */
  rtems_gpio_interrupt_conf* interrupt;

  /* Struct with bsp specific data, to use during the pin request.
   * If function == BSP_SPECIFIC this should have a pointer to
   * a rtems_gpio_specific_data struct.
   *
   * If not this field may be NULL. This is passed to the bsp function so any bsp specific data
   * can be passed to it through this pointer. */
  void* bsp_specific;
} rtems_gpio_pin_conf;

/** @} */

/**
 * @name gpio Usage
 *
 * @{
 */

/**
 * @brief Initializes the GPIO API.
 *
 * @retval RTEMS_SUCCESSFUL API successfully initialized.
 * @retval * For other error code see @rtems_semaphore_create().
 */
extern rtems_status_code rtems_gpio_initialize(void);

/**
 * @brief Requests a GPIO pin configuration from the API.
 *        It may be used either to request a new pin, or to update the
 *        configuration/functions of a pin currently in use.
 *
 * @param[in] conf rtems_gpio_pin_conf structure filled with the pin information
 *                 and desired configurations.
 *
 * @retval RTEMS_SUCCESSFUL Pin was configured successfully.
 * @retval RTEMS_UNSATISFIED Could not request/update the pin's configuration.
 */
extern rtems_status_code rtems_gpio_request_conf(rtems_gpio_pin_conf* conf);

/**
 * @brief Sets multiple output GPIO pins with the logical high.
 *
 * @param[in] count Number of GPIO pins to set.
 * @param[in] ... Comma-separated list of GPIO pin numbers.
 *
 * @retval RTEMS_SUCCESSFUL All pins were set successfully.
 * @retval RTEMS_INVALID_ID At least one pin number is invalid.
 * @retval RTEMS_NOT_CONFIGURED At least one of the received pins
 *                              is not configured as a digital output.
 * @retval RTEMS_UNSATISFIED Could not set the GPIO pins.
 */
extern rtems_status_code rtems_gpio_multi_set(uint32_t count, ...);

/**
 * @brief Sets multiple output GPIO pins with the logical low.
 *
 * @param[in] count Number of GPIO pins to clear.
 * @param[in] ... Comma-separated list of GPIO pin numbers.
 *
 * @retval RTEMS_SUCCESSFUL All pins were cleared successfully.
 * @retval RTEMS_INVALID_ID At least one pin number is invalid.
 * @retval RTEMS_NOT_CONFIGURED At least one of the received pins
 *                              is not configured as a digital output.
 * @retval RTEMS_UNSATISFIED Could not clear the GPIO pins.
 */
extern rtems_status_code rtems_gpio_multi_clear(uint32_t count, ...);

/**
 * @brief Sets an output GPIO pin with the logical high.
 *
 * @param[in] pin_number GPIO pin number.
 *
 * @retval RTEMS_SUCCESSFUL Pin was set successfully.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_NOT_CONFIGURED The received pin is not configured
 *                              as a digital output.
 * @retval RTEMS_UNSATISFIED Could not set the GPIO pin.
 */
extern rtems_status_code rtems_gpio_set(uint32_t pin_number);

/**
 * @brief Sets an output GPIO pin with the logical low.
 *
 * @param[in] pin_number GPIO pin number.
 *
 * @retval RTEMS_SUCCESSFUL Pin was cleared successfully.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_NOT_CONFIGURED The received pin is not configured
 *                              as a digital output.
 * @retval RTEMS_UNSATISFIED Could not clear the GPIO pin.
 */
extern rtems_status_code rtems_gpio_clear(uint32_t pin_number);

/**
 * @brief Returns the value (level) of a GPIO input pin.
 *
 * @param[in] pin_number GPIO pin number.
 *
 * @retval The function returns 0 or 1 depending on the pin current
 *         logical value.
 * @retval -1 Pin number is invalid, or not a digital input pin.
 */
extern int rtems_gpio_get_value(uint32_t pin_number);

/**
 * @brief Assigns a certain function to a GPIO pin.
 *
 * @param[in] pin_number GPIO pin number.
 * @param[in] function The new function for the pin.
 * @param[in] output_enabled If TRUE and @var function is DIGITAL_OUTPUT,
 *                           then the pin is set with the logical high.
 *                           Otherwise it is set with logical low.
 * @param[in] logic_invert Reverses the digital I/O logic for DIGITAL_INPUT
 *                         and DIGITAL_OUTPUT pins.
 * @param[in] bsp_specific Pointer to a bsp defined structure with bsp-specific
 *                         data. This field is not handled by the API.
 *
 * @retval RTEMS_SUCCESSFUL Pin was configured successfully.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_RESOURCE_IN_USE The received pin is already being used.
 * @retval RTEMS_UNSATISFIED Could not assign the GPIO function.
 * @retval RTEMS_NOT_DEFINED GPIO function not defined, or NOT_USED.
 */
extern rtems_status_code rtems_gpio_request_pin(uint32_t pin_number, rtems_gpio_function function, bool output_enable, bool logic_invert, void* bsp_specific);

/**
 * @brief Configures a single GPIO pin pull resistor.
 *
 * @param[in] pin_number GPIO pin number.
 * @param[in] mode The pull resistor mode.
 *
 * @retval RTEMS_SUCCESSFUL Pull resistor successfully configured.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_UNSATISFIED Could not set the pull mode.
 */
extern rtems_status_code rtems_gpio_resistor_mode(uint32_t pin_number, rtems_gpio_pull_mode mode);

/**
 * @brief Releases a GPIO pin from the API, making it available to be used
 *        again.
 *
 * @param[in] pin_number GPIO pin number.
 *
 * @retval RTEMS_SUCCESSFUL Pin successfully disabled on the API.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval * Could not disable an ative interrupt on this pin,
 *           @see rtems_gpio_disable_interrupt().
 */
extern rtems_status_code rtems_gpio_release_pin(uint32_t pin_number);

/**
 * @brief Attaches a debouncing function to a given pin/switch.
 *        Debouncing is done by requiring a certain number of clock ticks to
 *        pass between interrupts. Any interrupt fired too close to the last
 *        will be ignored as it is probably the result of an involuntary
 *        switch/button bounce after being released.
 *
 * @param[in] pin_number GPIO pin number.
 * @param[in] ticks Minimum number of clock ticks that must pass between
 *                  interrupts so it can be considered a legitimate
 *                  interrupt.
 *
 * @retval RTEMS_SUCCESSFUL De-bounce function successfully attached to the pin.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_NOT_CONFIGURED The current pin is not configured as a digital
 *                              input, hence it can not be connected to a switch.
 */
extern rtems_status_code rtems_gpio_debounce_switch(uint32_t pin_number, int ticks);

/**
 * @brief Connects a new user-defined interrupt handler to a given pin.
 *
 * @param[in] pin_number GPIO pin number.
 * @param[in] handler Pointer to a function that will be called every time
 *                    the enabled interrupt for the given pin is generated.
 *                    This function must return information about its
 *                    handled/unhandled state.
 * @param[in] arg Void pointer to the arguments of the user-defined handler.
 *
 * @retval RTEMS_SUCCESSFUL Handler successfully connected to this pin.
 * @retval RTEMS_NO_MEMORY Could not connect more user-defined handlers to
 *                         the given pin.
 * @retval RTEMS_NOT_CONFIGURED The given pin has no interrupt configured.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_RESOURCE_IN_USE The current user-defined handler for this pin
 *                               is unique.
 */
extern rtems_status_code rtems_gpio_interrupt_handler_install(
uint32_t pin_number,
rtems_gpio_irq_state (*handler) (void *arg),
void *arg
);

/**
 * @brief Enables interrupts to be generated on a given GPIO pin.
 *        When fired that interrupt will call the given handler.
 *
 * @param[in] pin_number GPIO pin number.
 * @param[in] interrupt Type of interrupt to enable for the pin.
 * @param[in] handler Pointer to a function that will be called every time
 *                    @var interrupt is generated. This function must return
 *                    information about its handled/unhandled state.
 * @param[in] arg Void pointer to the arguments of the user-defined handler.
 *
 * @retval RTEMS_SUCCESSFUL Interrupt successfully enabled for this pin.
 * @retval RTEMS_UNSATISFIED Could not install the GPIO ISR, create/start
 *                           the handler task, or enable the interrupt
 *                           on the pin.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_RESOURCE_IN_USE The pin already has an enabled interrupt.
 */
extern rtems_status_code rtems_gpio_enable_interrupt(
uint32_t pin_number,
rtems_gpio_interrupt interrupt,
rtems_gpio_handler_flag flag,
rtems_gpio_irq_state (*handler) (void *arg),
void *arg
);

/**
 * @brief Disconnects an user-defined interrupt handler from the given pin.
 *        If in the end there are no more user-defined handlers connected
 *        to the pin, interrupts are disabled on the given pin.
 *
 * @param[in] pin_number GPIO pin number.
 * @param[in] handler Pointer to the user-defined handler
 * @param[in] arg Void pointer to the arguments of the user-defined handler.
 *
 * @retval RTEMS_SUCCESSFUL Handler successfully disconnected from this pin.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval * @see rtems_gpio_disable_interrupt()
 */
rtems_status_code rtems_gpio_interrupt_handler_remove(
uint32_t pin_number,
rtems_gpio_irq_state (*handler) (void *arg),
void *arg
);

/**
 * @brief Stops interrupts from being generated on a given GPIO pin
 *        and removes the corresponding handler.
 *
 * @param[in] pin_number GPIO pin number.
 *
 * @retval RTEMS_SUCCESSFUL Interrupt successfully disabled for this pin.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_UNSATISFIED Could not remove the current interrupt handler,
 *                           could not recognise the current active interrupt
 *                           on this pin or could not disable interrupts on
 *                           this pin.
 */
extern rtems_status_code rtems_gpio_disable_interrupt(uint32_t pin_number);

/**
 * @brief Sets multiple output GPIO pins with the logical high. This must be implemented
 *        by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] bitmask Bitmask of GPIO pins to set in the given bank.
 *
 * @retval RTEMS_SUCCESSFUL All pins were set successfully.
 * @retval RTEMS_UNSATISFIED Could not set at least one of the pins.
 */
extern rtems_status_code rtems_bsp_gpio_multi_set(uint32_t bank, uint32_t bitmask);

/**
 * @brief Sets multiple output GPIO pins with the logical low. This must be implemented
 *        by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] bitmask Bitmask of GPIO pins to clear in the given bank.
 *
 * @retval RTEMS_SUCCESSFUL All pins were cleared successfully.
 * @retval RTEMS_UNSATISFIED Could not clear at least one of the pins.
 */
extern rtems_status_code rtems_bsp_gpio_multi_clear(uint32_t bank, uint32_t bitmask);

/**
 * @brief Sets an output GPIO pin with the logical high. This must be implemented
 *        by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] pin GPIO pin number within the given bank.
 *
 * @retval RTEMS_SUCCESSFUL Pin was set successfully.
 * @retval RTEMS_UNSATISFIED Could not set the given pin.
 */
extern rtems_status_code rtems_bsp_gpio_set(uint32_t bank, uint32_t pin);

/**
 * @brief Sets an output GPIO pin with the logical low. This must be implemented
 *        by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] pin GPIO pin number within the given bank.
 *
 * @retval RTEMS_SUCCESSFUL Pin was cleared successfully.
 * @retval RTEMS_UNSATISFIED Could not clear the given pin.
 */
extern rtems_status_code rtems_bsp_gpio_clear(uint32_t bank, uint32_t pin);

/**
 * @brief Returns the value (level) of a GPIO input pin. This must be implemented
 *        by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] pin GPIO pin number within the given bank.
 *
 * @retval The function must return 0 or 1 depending on the pin current
 *         logical value.
 * @retval -1 Could not read the pin level.
 */
extern int rtems_bsp_gpio_get_value(uint32_t bank, uint32_t pin);

/**
 * @brief Assigns the digital input function to the given pin.
 *        This must be implemented by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] pin GPIO pin number within the given bank.
 * @param[in] bsp_specific Pointer to a bsp defined structure with bsp-specific
 *                         data.
 *
 * @retval RTEMS_SUCCESSFUL Function was asssigned successfully.
 * @retval RTEMS_UNSATISFIED Could not assign the function to the pin.
 */
extern rtems_status_code rtems_bsp_gpio_select_input(uint32_t bank, uint32_t pin, void* bsp_specific);

/**
 * @brief Assigns the digital output function to the given pin.
 *        This must be implemented by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] pin GPIO pin number within the given bank.
 * @param[in] bsp_specific Pointer to a bsp defined structure with bsp-specific
 *                         data.
 *
 * @retval RTEMS_SUCCESSFUL Function was asssigned successfully.
 * @retval RTEMS_UNSATISFIED Could not assign the function to the pin.
 */
extern rtems_status_code rtems_bsp_gpio_select_output(uint32_t bank, uint32_t pin, void* bsp_specific);

/**
 * @brief Assigns a bsp specific function to the given pin.
 *        This must be implemented by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] pin GPIO pin number within the given bank.
 * @param[in] function Bsp defined GPIO function.
 * @param[in] pin_data Pointer to a bsp defined structure with bsp-specific
 *                     data.
 *
 * @retval RTEMS_SUCCESSFUL Function was asssigned successfully.
 * @retval RTEMS_UNSATISFIED Could not assign the function to the pin.
 */
extern rtems_status_code rtems_bsp_select_specific_io(uint32_t bank, uint32_t pin, uint32_t function, void* pin_data);

/**
 * @brief Configures a single GPIO pin pull resistor.
 *        This must be implemented by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] pin GPIO pin number within the given bank.
 * @param[in] mode The pull resistor mode.
 *
 * @retval RTEMS_SUCCESSFUL Pull resistor successfully configured.
 * @retval RTEMS_UNSATISFIED Could not set the pull mode.
 */
extern rtems_status_code rtems_bsp_gpio_set_resistor_mode(uint32_t bank, uint32_t pin, rtems_gpio_pull_mode mode);

/**
 * @brief Reads and returns a vector/bank interrupt event line.
 *        This must be implemented by each BSP.
 *
 * @param[in] vector GPIO vector/bank.
 *
 * @retval Bitmask (max 32-bit) representing a GPIO bank, where a bit set
 *         indicates an active interrupt on that pin.
 */
extern uint32_t rtems_bsp_gpio_interrupt_line(rtems_vector_number vector);

/**
 * @brief Clears a vector/bank interrupt event line.
 *        This must be implemented by each BSP.
 *
 * @param[in] vector GPIO vector/bank.
 * @param[in] event_status Bitmask with the processed interrupts on the given
 *                         vector. This bitmask is the same as calculated in
 *                         @see rtems_bsp_gpio_interrupt_line().
 */
extern void rtems_bsp_gpio_clear_interrupt_line(rtems_vector_number vector, uint32_t event_status);

/**
 * @brief Calculates a vector number for a given GPIO bank.
 *        This must be implemented by each BSP.
 *
 * @param[in] bank GPIO bank number.
 *
 * @retval The corresponding rtems_vector_number.
 */
extern rtems_vector_number rtems_bsp_gpio_get_vector(uint32_t bank);

/**
 * @brief Enables interrupts to be generated on a given GPIO pin.
 *        This must be implemented by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] pin GPIO pin number within the given bank.
 * @param[in] interrupt Type of interrupt to enable for the pin.
 *
 * @retval RTEMS_SUCCESSFUL Interrupt successfully enabled for this pin.
 * @retval RTEMS_UNSATISFIED Could not enable the interrupt on the pin.
 */
extern rtems_status_code rtems_bsp_enable_interrupt(uint32_t bank, uint32_t pin, rtems_gpio_interrupt interrupt);

/**
 * @brief Disables interrupt on hardware. This must be implemented by each BSP.
 */

/**
 * @brief Stops interrupts from being generated on a given GPIO pin.
 *        This must be implemented by each BSP.
 *
 * @param[in] bank GPIO bank number.
 * @param[in] pin GPIO pin number within the given bank.
 * @param[in] enabled_interrupt Interrupt type currently active on this pin.
 *
 * @retval RTEMS_SUCCESSFUL Interrupt successfully disabled for this pin.
 * @retval RTEMS_UNSATISFIED Could not disable interrupts on this pin.
 */
extern rtems_status_code rtems_bsp_disable_interrupt(uint32_t bank, uint32_t pin, rtems_gpio_interrupt enabled_interrupt);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_SHARED_GPIO_H */
