/**
 * @file gpio.c
 *
 * @ingroup rtems_gpio
 *
 * @brief RTEMS GPIO API implementation.
 */

/*
 *  Copyright (c) 2014-2015 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/raspberrypi.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>
#include <bsp/gpio.h>

#include <assert.h>
#include <stdlib.h>

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

#define AQUIRE_LOCK(m) assert( rtems_semaphore_obtain(m,		\
						      RTEMS_WAIT,	\
						      RTEMS_NO_TIMEOUT	\
						      ) == RTEMS_SUCCESSFUL )

#define RELEASE_LOCK(m) assert( rtems_semaphore_release(m) == RTEMS_SUCCESSFUL )

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

static gpio_pin** gpio_pin_state;
static bool is_initialized = false;
static uint32_t gpio_count;
static uint32_t bank_count;
static uint32_t pins_per_bank;
static uint32_t odd_bank_pins;
static int* interrupt_counter;
static rtems_id int_id = RTEMS_ID_NONE;

static uint32_t get_bank_number(uint32_t pin_number)
{
  return pin_number / pins_per_bank;
}

static uint32_t get_pin_number(uint32_t pin_number)
{
  return pin_number % pins_per_bank;
}

/**
 * @brief De-bounces a switch by requiring a certain time to pass between 
 *        interrupts. Any interrupt fired too close to the last will be 
 *        ignored as it is probably the result of an involuntary switch/button
 *        bounce after being released.
 *
 * @param[in] gpio GPIO pin that is being debounced. 
 *
 * @retval 0 Interrupt is likely provoked by a user press on the switch.
 * @retval -1 Interrupt was generated too close to the last one. 
 *            Probably a switch bounce.
 */
static int debounce_switch(gpio_pin* gpio)
{
  rtems_interval time;
  
  time = rtems_clock_get_ticks_since_boot();
  
  if ( (time - gpio->last_isr_tick) < gpio->debouncing_tick_count ) {
    return -1;
  }

  gpio->last_isr_tick = time;
  
  return 0;
}

static rtems_task generic_handler_task(rtems_task_argument arg)
{
  gpio_handler_list *handler_list;
  rtems_event_set out;
  gpio_pin* gpio;
  int handled_count;
  int rv;

  gpio = (gpio_pin*) arg;

  while ( true ) {
    handled_count = 0;

    rtems_event_receive(RTEMS_EVENT_1, RTEMS_EVENT_ALL | RTEMS_WAIT,
                        RTEMS_NO_TIMEOUT,
                        &out);

    /* If this pin has the debouncing function attached, call it. */
    if ( gpio->debouncing_tick_count > 0 ) {
      rv = debounce_switch(gpio);

      /* If the handler call was caused by a switch bounce, ignores and move on. */
      if ( rv < 0 ) {
	continue;
      }

      /* Record the current clock tick. */
      gpio->last_isr_tick = rtems_clock_get_ticks_since_boot();
    }
  
    handler_list = gpio->handler_list;

    while ( handler_list != NULL ) {
      /* Call the user's ISR. */
      if ( (handler_list->handler)(handler_list->arg) == IRQ_HANDLED ) {
        ++handled_count;
      }

      handler_list = handler_list->next_isr;
    }

    /* If no handler assumed the interrupt, treat it as a spurious interrupt. */
    if ( handled_count == 0 ) {
      bsp_interrupt_handler_default(bsp_gpio_get_vector(gpio->bank_number));
    }
  }
}

static rtems_status_code gpio_check_pin(int pin_number, bool check_used)
{
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  if ( check_used == TRUE ) {
    bank = get_bank_number(pin_number);
    pin = get_pin_number(pin_number);

    /* If the pin is already being used returns with an error. */
    if ( gpio_pin_state[bank][pin].pin_type != NOT_USED ) {
      return RTEMS_RESOURCE_IN_USE;
    }
  }

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Initializes the GPIO API and sets every pin as NOT_USED.
 *
 * @retval RTEMS_SUCCESSFUL API successfully initialized.
 * @retval * @rtems_semaphore_create().
 */
rtems_status_code gpio_initialize(void)
{
  rtems_status_code sc;
  gpio_layout layout;
  int bank_pins;
  int pin;
  int bank;
  int i;

  if ( is_initialized ) {
    return RTEMS_SUCCESSFUL;
  }

  is_initialized = true;

  sc = rtems_semaphore_create(rtems_build_name('G', 'I', 'N', 'T'), 1, MUTEX_ATRIBUTES, 0, &int_id);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  layout = bsp_gpio_initialize();

  gpio_count = layout.pin_count;
  pins_per_bank = layout.pins_per_bank;

  bank_count = gpio_count / pins_per_bank;

  /* Account for remaining pins after filling the last bank. */
  odd_bank_pins = gpio_count % pins_per_bank;
  
  if ( odd_bank_pins > 0 ) {
    ++bank_count;
  }

  gpio_pin_state = (gpio_pin **) malloc(bank_count * sizeof(gpio_pin*));

  for ( i = 0; i < bank_count; ++i ) {
    if ( i == bank_count - 1 ) {
      bank_pins = odd_bank_pins;
    }
    else {
      bank_pins = pins_per_bank;
    }
    
    gpio_pin_state[i] = (gpio_pin *) malloc(bank_pins * sizeof(gpio_pin));
  }

  interrupt_counter = (int *) calloc(bank_count, sizeof(int));
  
  for ( i = 0; i < gpio_count; ++i ) {
    bank = i / pins_per_bank;
    pin = i % pins_per_bank;

    gpio_pin_state[bank][pin].bank_number = bank;
    gpio_pin_state[bank][pin].pin_number = pin;
    gpio_pin_state[bank][pin].pin_type = NOT_USED;
    gpio_pin_state[bank][pin].enabled_interrupt = NONE;
    gpio_pin_state[bank][pin].task_id = RTEMS_ID_NONE;
    gpio_pin_state[bank][pin].handler_list = NULL;
    gpio_pin_state[bank][pin].debouncing_tick_count = 0;
    gpio_pin_state[bank][pin].last_isr_tick = 0;
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code gpio_request_conf(gpio_pin_conf conf)
{
  gpio_interrupt_conf* interrupt_conf;
  rtems_status_code sc;
  bool new_request;
  uint32_t bank;
  uint32_t pin;

  new_request = FALSE;
  
  sc = gpio_check_pin(conf.pin_number, TRUE);

  if ( sc == RTEMS_SUCCESSFUL ) {    
    sc = gpio_request_pin(conf.pin_number, conf.function, conf.bsp_specific);

    if ( sc != RTEMS_SUCCESSFUL ) {
      //TODO: print explanatory messages, and maybe return another code related to thsi particular function (apply to the rest of this function)
      return sc;
    }

    new_request = TRUE;
  }

  else if ( sc == RTEMS_INVALID_ID ) {
    return sc;
  }
  
  sc = gpio_input_mode(conf.pin_number, conf.pull_mode);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  interrupt_conf = (gpio_interrupt_conf*) conf.interrupt;

  if ( interrupt_conf != NULL ) {
    bank = get_bank_number(conf.pin_number);
    pin = get_pin_number(conf.pin_number);

    if ( interrupt_conf->enabled_interrupt != gpio_pin_state[bank][pin].enabled_interrupt ) {
      if ( new_request == FALSE ) {
	sc = gpio_disable_interrupt(conf.pin_number);

	if ( sc != RTEMS_SUCCESSFUL ) {
	  return sc;
	}
      }

      sc = gpio_enable_interrupt(conf.pin_number,
				 interrupt_conf->enabled_interrupt,
				 interrupt_conf->handler_flag,
				 interrupt_conf->handler,
				 interrupt_conf->arg);

      if ( sc != RTEMS_SUCCESSFUL ) {
	return sc;
      }
      
    }

    if ( interrupt_conf->clock_tick_interval != gpio_pin_state[bank][pin].debouncing_tick_count ) {
      gpio_pin_state[bank][pin].debouncing_tick_count = interrupt_conf->clock_tick_interval;
      gpio_pin_state[bank][pin].last_isr_tick = 0;
    }
  }

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Sets an output GPIO pin with the logical high.
 *
 * @param[in] pin_number GPIO pin number.
 *
 * @retval RTEMS_SUCCESSFUL Pin was set successfully.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_NOT_CONFIGURED The received pin is not configured 
 *                              as an digital output.
 * @retval RTEMS_UNSATISFIED Could not set the GPIO pin.
 */
rtems_status_code gpio_set(int pin_number)
{
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);
  
  if ( gpio_pin_state[bank][pin].pin_type != DIGITAL_OUTPUT ) {
    return RTEMS_NOT_CONFIGURED;
  }

  return bsp_gpio_set(bank, pin);
}

/**
 * @brief Sets an output GPIO pin with the logical low.
 *
 * @param[in] pin_number GPIO pin number.
 *
 * @retval RTEMS_SUCCESSFUL Pin was cleared successfully.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_NOT_CONFIGURED The received pin is not configured 
 *                              as an digital output.
 * @retval RTEMS_UNSATISFIED Could not clear the GPIO pin.
 */
rtems_status_code gpio_clear(int pin_number)
{
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }
  
  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);

  if ( gpio_pin_state[bank][pin].pin_type != DIGITAL_OUTPUT ) {
    return RTEMS_NOT_CONFIGURED;
  }

  return bsp_gpio_clear(bank, pin);
}

/**
 * @brief Returns the value (level) of a GPIO input pin.
 *
 * @param[in] pin_number GPIO pin number.
 *
 * @retval The function returns 0 or 1 depending on the pin current 
 *         logical value.
 * @retval -1 Pin number is invalid.
 */
int gpio_get_value(int pin_number)
{
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return -1;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);

  return bsp_gpio_get_value(bank, pin);
}

/**
 * @brief Assigns a certain function to a GPIO pin.
 *
 * @param[in] pin_number GPIO pin number.
 * @param[in] function The new function of the pin.
 *
 * @retval RTEMS_SUCCESSFUL Pin was configured successfully.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_RESOURCE_IN_USE The received pin is already being used.
 * @retval RTEMS_UNSATISFIED Could not assign the GPIO function.
 */
rtems_status_code gpio_request_pin(int pin_number, gpio_function function, void* bsp_specific)
{
  gpio_specific_data* bsp_data;
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);

  /* If the pin is already being used returns with an error. */
  if ( gpio_pin_state[bank][pin].pin_type != NOT_USED ) {
    return RTEMS_RESOURCE_IN_USE;
  }

  switch ( function ) {
    case DIGITAL_INPUT:
      sc = bsp_gpio_select_input(bank, pin, bsp_specific);
      break;
    case DIGITAL_OUTPUT:
      sc = bsp_gpio_select_output(bank, pin, bsp_specific);
      break;
    case BSP_SPECIFIC:
      bsp_data = (gpio_specific_data*) bsp_specific;

      if ( bsp_data == NULL ) {
	return RTEMS_UNSATISFIED;
      }

      sc = bsp_select_specific_io(bank, pin, bsp_data->io_function, bsp_data->pin_data);
      break;
    case NOT_USED:
      return RTEMS_SUCCESSFUL;

      //TODO: add default case?
  }

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  /* If the function was successfuly assigned to the pin,
   * record that information on the gpio_pin_state structure. */
  gpio_pin_state[bank][pin].pin_type = function;

  return RTEMS_SUCCESSFUL;
}

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
rtems_status_code gpio_input_mode(int pin_number, gpio_pull_mode mode)
{
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);
  
  /* If the desired actuation mode is already set, silently exits. 
   * The NO_PULL_RESISTOR is a special case, as some platforms have
   * pull-up resistors enabled on startup, so this state may have to
   * be reinforced in the hardware. */
  if ( gpio_pin_state[bank][pin].input_mode == mode  && mode != NO_PULL_RESISTOR ) {
    return RTEMS_SUCCESSFUL;
  }

  sc = bsp_gpio_set_input_mode(bank, pin, mode);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  gpio_pin_state[bank][pin].input_mode = mode;
  
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Releases a GPIO pin from the API, making it available to be used 
 *        again.
 *
 * @param[in] pin_number GPIO pin number.
 *
 * @retval RTEMS_SUCCESSFUL Pin successfully disabled on the API.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_UNSATISFIED Could not disable an ative interrupt on this pin, 
 *                           @see gpio_disable_interrupt().
 */
rtems_status_code gpio_disable_pin(int pin_number)
{
  rtems_status_code sc;
  gpio_pin* gpio;
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);

  gpio = &gpio_pin_state[bank][pin];
 
  /* If the pin has an enabled interrupt then remove the handler(s),
   * and disable the interrupts on that pin. */
  if ( gpio->enabled_interrupt != NONE ) {
    sc = gpio_disable_interrupt(pin_number);

    if ( sc != RTEMS_SUCCESSFUL ) {
      return sc;
    }
  }

  gpio->pin_type = NOT_USED;
  gpio->task_id = RTEMS_ID_NONE;
  gpio->debouncing_tick_count = 0;
  gpio->last_isr_tick = 0;

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Generic ISR that clears the event register on the Raspberry Pi and 
 *        calls an user defined ISR. 
 *
 * @param[in] arg Void pointer to a handler_arguments structure. 
 */
static void generic_isr(void* arg)
{
  rtems_vector_number vector;
  uint32_t event_status;
  uint32_t bank_pin_count;
  gpio_pin *gpio;
  uint32_t bank_number;
  int i;

  gpio = (gpio_pin *) arg;

  /* Get the bank/vector number of a pin (e.g.: 0) from this bank. */
  bank_number = gpio[0].bank_number;

  vector = bank_number;

  /* If the current bank is an odd last bank (i.e.: not completely filled). */
  if ( odd_bank_pins > 0 &&  bank_number == bank_count - 1 ) {
    bank_pin_count = odd_bank_pins;
  }
  else {
    bank_pin_count = pins_per_bank;
  }

  /* Prevents more interrupts from being generated on GPIO. */
  bsp_interrupt_vector_disable(bsp_gpio_get_vector(bank_number));

  /* TODO: Insert Memory Barrier */

  /* Obtains a 32-bit bitmask, with the pins currently reporting interrupts
   * signaled with 1. */
  event_status = bsp_gpio_interrupt_line(vector);

  /* Iterates through the bitmask and calls the corresponding handler
   * for active interrupts. */
  for ( i = 0; i < bank_pin_count; ++i ) {
    /* If active, wake the corresponding pin's ISR task. */
    if ( event_status & (1 << i) ) {
      rtems_event_send(gpio[i].task_id, RTEMS_EVENT_1);
    }
  }

  /* Clear all active events. */
  bsp_gpio_clear_interrupt_line(vector, event_status);
  
  /* TODO: Insert Memory Barrier to prevent the interrupts being enabled before updating the event register. */

  bsp_interrupt_vector_enable(bsp_gpio_get_vector(bank_number));
}

/**
 * @brief Defines for a GPIO input pin the number of clock ticks that must pass
 *        before an generated interrupt is guaranteed to be generated by the user
 *        and not by a bouncing switch/button.
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
rtems_status_code gpio_debounce_switch(int pin_number, int ticks)
{
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);

  if ( gpio_pin_state[bank][pin].pin_type != DIGITAL_INPUT ) {
    return RTEMS_NOT_CONFIGURED;
  }

  gpio_pin_state[bank][pin].debouncing_tick_count = ticks;
  gpio_pin_state[bank][pin].last_isr_tick = rtems_clock_get_ticks_per_second();

  return RTEMS_SUCCESSFUL;
}

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
rtems_status_code gpio_interrupt_handler_install(
int pin_number,
gpio_irq_state (*handler) (void *arg),
void *arg
)
{
  gpio_handler_list *isr_node;
  gpio_pin* gpio;
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);

  gpio = &gpio_pin_state[bank][pin];

  if ( gpio->enabled_interrupt == NONE ) {
    return RTEMS_NOT_CONFIGURED;
  }
  /* If the pin already has an enabled interrupt but the installed handler
   * is set as unique. */
  else if ( gpio->handler_flag == UNIQUE_HANDLER && gpio->handler_list != NULL ) {
      return RTEMS_RESOURCE_IN_USE;
  }
  
  isr_node = (gpio_handler_list *) malloc(sizeof(gpio_handler_list));

  if ( isr_node == NULL ) {
    return RTEMS_NO_MEMORY;
  }
  
  isr_node->handler = handler;
  isr_node->arg = arg;

  if ( gpio->handler_flag == SHARED_HANDLER ) {
    isr_node->next_isr = gpio->handler_list;
  }
  else {
    isr_node->next_isr = NULL;
  }

  gpio->handler_list = isr_node;

  return RTEMS_SUCCESSFUL;
}

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
 * @retval RTEMS_NOT_CONFIGURED The pin is not configured as a digital input,
 *                              hence interrupts are not taken into account.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval RTEMS_RESOURCE_IN_USE The current user-defined handler for this pin
 *                               is unique.
 */
rtems_status_code gpio_enable_interrupt(
int pin_number, 
gpio_interrupt interrupt,
gpio_handler_flag flag,
gpio_irq_state (*handler) (void *arg),
void *arg
)
{
  rtems_status_code sc;
  gpio_pin* gpio;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);

  gpio = &gpio_pin_state[bank][pin];

  if ( gpio->pin_type != DIGITAL_INPUT ) {
    return RTEMS_NOT_CONFIGURED;
  }

  /* If trying to enable the same type of interrupt on the same pin, or if the pin
   * already has an enabled interrupt, silently exits. */
  if ( interrupt == gpio->enabled_interrupt || gpio->enabled_interrupt != NONE ) {
    return RTEMS_SUCCESSFUL;
  }

  AQUIRE_LOCK(int_id);
  
  /* Creates and starts a new task which will call the corresponding 
   * user-defined handlers. */
  sc = rtems_task_create(rtems_build_name('G', 'P', 'I', 'O'),
			 5,
			 RTEMS_MINIMUM_STACK_SIZE * 2,
			 RTEMS_NO_TIMESLICE,
			 RTEMS_DEFAULT_ATTRIBUTES,
			 &gpio->task_id);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(int_id);
      
    return RTEMS_UNSATISFIED;
  }
  
  sc = rtems_task_start(gpio->task_id, generic_handler_task, (rtems_task_argument) gpio);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(int_id);
      
    sc = rtems_task_delete(gpio->task_id);

    assert( sc == RTEMS_SUCCESSFUL );
      
    return RTEMS_UNSATISFIED;
  } 

  gpio->enabled_interrupt = interrupt;
  gpio->handler_flag = flag;

  /* Installs the interrupt handler. */
  sc = gpio_interrupt_handler_install(pin_number, handler, arg);

  assert( sc == RTEMS_SUCCESSFUL );

  bsp_interrupt_vector_disable(bsp_gpio_get_vector(bank));//TODO: move this up?

  /* If the generic ISR has not been yet installed for this bank, installs it.
   * This ISR will be responsible for calling the handler tasks,
   * which in turn will call the user-defined interrupt handlers.*/
  if ( interrupt_counter[bank] == 0 ) {
    sc = rtems_interrupt_handler_install(bsp_gpio_get_vector(bank), 
                                         "GPIO_HANDLER", 
                                         RTEMS_INTERRUPT_UNIQUE, 
                                         (rtems_interrupt_handler) generic_isr,
                                         gpio_pin_state[bank]);

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(int_id);

      bsp_interrupt_vector_enable(bsp_gpio_get_vector(bank));
      
      return RTEMS_UNSATISFIED;
    }
  }
  
  sc = bsp_enable_interrupt(pin_number, interrupt);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(int_id);

    bsp_interrupt_vector_enable(bsp_gpio_get_vector(bank));
      
    return RTEMS_UNSATISFIED;
  }
  
  ++interrupt_counter[bank];

  RELEASE_LOCK(int_id);

  bsp_interrupt_vector_enable(bsp_gpio_get_vector(bank));
  
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Disconnects a new user-defined interrupt handler from the given pin.
 *        If in the end there are no more user-defined handler connected
 *        to the pin interrupts are disabled on the given pin.
 *
 * @param[in] pin_number GPIO pin number.
 * @param[in] handler Pointer to the user-defined handler
 * @param[in] arg Void pointer to the arguments of the user-defined handler.
 *
 * @retval RTEMS_SUCCESSFUL Handler successfully disconnected from this pin.
 * @retval RTEMS_INVALID_ID Pin number is invalid.
 * @retval * @see gpio_disable_interrupt()
 */
rtems_status_code gpio_interrupt_handler_remove(
int pin_number,
gpio_irq_state (*handler) (void *arg),
void *arg
)
{
  gpio_handler_list *isr_node, *next_node;
  gpio_pin* gpio;
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);
  
  gpio = &gpio_pin_state[bank][pin];

  AQUIRE_LOCK(int_id);

  isr_node = gpio->handler_list;

  if ( isr_node != NULL ) {
    if ( isr_node->handler == handler && isr_node->arg == arg ) {
      gpio->handler_list = isr_node->next_isr;

      free(isr_node);
    }
    else {
      while ( isr_node->next_isr != NULL ) {
        if ( (isr_node->next_isr)->handler == handler && (isr_node->next_isr)->arg == arg ) {
          next_node = (isr_node->next_isr)->next_isr;

          free(isr_node->next_isr);

          isr_node->next_isr = next_node;
        }
      }
    }
  }

  /* If the removed handler was the last for this pin, disables further
   * interrupts on this pin. */
  if ( gpio->handler_list == NULL ) {
    RELEASE_LOCK(int_id);
    
    return gpio_disable_interrupt(pin_number);
  }

  RELEASE_LOCK(int_id);
  
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Stops interrupts from being generated from a given GPIO pin
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
rtems_status_code gpio_disable_interrupt(int pin_number)
{
  gpio_handler_list *isr_node;
  rtems_status_code sc;
  gpio_pin* gpio;
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = get_bank_number(pin_number);
  pin = get_pin_number(pin_number);

  gpio = &gpio_pin_state[bank][pin];
  
  if ( interrupt_counter[bank] == 0 || gpio->enabled_interrupt == NONE ) {
    return RTEMS_SUCCESSFUL;
  }

  bsp_interrupt_vector_disable(bsp_gpio_get_vector(bank));

  AQUIRE_LOCK(int_id);

  sc = bsp_disable_interrupt(pin_number, gpio->enabled_interrupt);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(int_id);

    bsp_interrupt_vector_enable(bsp_gpio_get_vector(bank));
      
    return RTEMS_UNSATISFIED;
  }
  
  gpio->enabled_interrupt = NONE;

  while ( gpio->handler_list != NULL ) {
    isr_node = gpio->handler_list;

    gpio->handler_list = isr_node->next_isr;

    free(isr_node);
  }

  sc = rtems_task_delete(gpio->task_id);

  assert( sc == RTEMS_SUCCESSFUL );
  
  --interrupt_counter[bank];

  /* If no GPIO interrupts are left in this bank, removes the handler. */
  if ( interrupt_counter[bank] == 0 ) {
    sc = rtems_interrupt_handler_remove(bsp_gpio_get_vector(bank), 
                                        (rtems_interrupt_handler) generic_isr, 
                                        gpio_pin_state[bank]);
    
    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(int_id);

      bsp_interrupt_vector_enable(bsp_gpio_get_vector(bank));
      
      return RTEMS_UNSATISFIED;
    }
  }

  RELEASE_LOCK(int_id);

  bsp_interrupt_vector_enable(bsp_gpio_get_vector(bank));
  
  return RTEMS_SUCCESSFUL;
}
