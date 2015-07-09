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

#include <rtems/score/atomic.h>
#include <rtems/chain.h>
#include <bsp/irq-generic.h>
#include <bsp/gpio.h>
#include <assert.h>
#include <stdlib.h>

/**
 * @brief GPIO API mutex attributes.
 */
#define MUTEX_ATTRIBUTES (     \
  RTEMS_LOCAL                  \
  | RTEMS_PRIORITY             \
  | RTEMS_BINARY_SEMAPHORE     \
  | RTEMS_INHERIT_PRIORITY     \
  | RTEMS_NO_PRIORITY_CEILING  \
)

#define CREATE_LOCK(name, lock_id) rtems_semaphore_create(  \
 name,                                                      \
 1,                                                         \
 MUTEX_ATTRIBUTES,                                          \
 0,                                                         \
 lock_id                                                    \
)

#define ACQUIRE_LOCK(m) assert( rtems_semaphore_obtain(m,               \
                                                       RTEMS_WAIT,      \
                                                       RTEMS_NO_TIMEOUT \
                                                       ) == RTEMS_SUCCESSFUL )

#define RELEASE_LOCK(m) assert( rtems_semaphore_release(m) == RTEMS_SUCCESSFUL )

#define GPIO_INTERRUPT_EVENT RTEMS_EVENT_1

/**
 * @brief Object containing relevant information about a GPIO group.
 *
 * Encapsulates relevant data for a GPIO pin group.
 */
struct rtems_gpio_group
{
  rtems_chain_node node;

  uint32_t *digital_inputs;
  uint32_t digital_input_bank;
  uint32_t input_count;

  uint32_t *digital_outputs;
  uint32_t digital_output_bank;
  uint32_t output_count;

  uint32_t *bsp_speficifc_pins;
  uint32_t bsp_specific_bank;
  uint32_t bsp_specific_pin_count;

  rtems_id group_lock;
};

/**
 * @brief Object containing relevant information to a list of user-defined
 *        interrupt handlers.
 *
 * Encapsulates relevant data for a GPIO interrupt handler.
 */
typedef struct
{
  rtems_chain_node node;

  /* User-defined ISR routine. */
  rtems_gpio_irq_state (*handler) (void *arg);

  /* User-defined arguments for the ISR routine. */
  void *arg;
} gpio_handler_node;

/**
 * @brief Object containing relevant information of a pin's interrupt
 *        configuration/state.
 *
 * Encapsulates relevant data of a GPIO pin interrupt state.
 */
typedef struct
{
  /* GPIO pin's bank. */
  uint32_t bank_number;

  /* Currently active interrupt. */
  rtems_gpio_interrupt active_interrupt;

  /* Id of the task that will be calling the user-defined ISR handlers
   * for this pin. */
  rtems_id handler_task_id;

  /* ISR shared flag. */
  rtems_gpio_handler_flag handler_flag;

  /* Linked list of interrupt handlers. */
  rtems_chain_control handler_chain;

  /* Switch-deboucing information. */
  uint32_t debouncing_tick_count;
  rtems_interval last_isr_tick;
} gpio_pin_interrupt_state;

/**
 * @brief Object containing information on a GPIO pin.
 *
 * Encapsulates relevant data about a GPIO pin.
 */
typedef struct
{
  rtems_gpio_function pin_function;

  /* GPIO pull resistor configuration. */
  rtems_gpio_pull_mode resistor_mode;

  /* If true inverts digital in/out applicational logic. */
  bool logic_invert;

  /* Interrupt data for a pin. This field is NULL if no interrupt is enabled
   * on the pin. */
  gpio_pin_interrupt_state *interrupt_state;
} gpio_pin;

/**
 * @brief Object containing relevant information regarding a GPIO bank state.
 *
 * Encapsulates relevant data for a GPIO bank.
 */
typedef struct
{
  uint32_t bank_number;
  uint32_t interrupt_counter;
  rtems_id lock;
} gpio_bank;

static gpio_pin **gpio_pin_state;
static Atomic_Flag init_flag = ATOMIC_INITIALIZER_FLAG;
static gpio_bank gpio_bank_state[GPIO_BANK_COUNT];
static rtems_chain_control gpio_group;

#define BANK_NUMBER(pin_number) pin_number / BSP_GPIO_PINS_PER_BANK
#define PIN_NUMBER(pin_number) pin_number % BSP_GPIO_PINS_PER_BANK

static int debounce_switch(gpio_pin_interrupt_state *interrupt_state)
{
  rtems_interval time;

  time = rtems_clock_get_ticks_since_boot();

  /* If not enough time has elapsed since last interrupt. */
  if (
      (time - interrupt_state->last_isr_tick) <
      interrupt_state->debouncing_tick_count
  ) {
    return -1;
  }

  interrupt_state->last_isr_tick = time;

  return 0;
}

static rtems_task generic_handler_task(rtems_task_argument arg)
{
  gpio_pin_interrupt_state *interrupt_state;
  rtems_chain_control *handler_list;
  rtems_chain_node *node;
  rtems_chain_node *next_node;
  gpio_handler_node *isr_node;
  rtems_event_set out;
  uint32_t bank;
  int handled_count;
  uint8_t rv;

  interrupt_state = (gpio_pin_interrupt_state *) arg;

  assert ( interrupt_state != NULL );

  bank = interrupt_state->bank_number;

  while ( true ) {
    handled_count = 0;

    /* Wait for interrupt event.
     * This is sent by the bank's generic_isr handler. */
    rtems_event_receive(
      GPIO_INTERRUPT_EVENT,
      RTEMS_EVENT_ALL | RTEMS_WAIT,
      RTEMS_NO_TIMEOUT,
      &out
    );

    ACQUIRE_LOCK(gpio_bank_state[bank].lock);

    /* If this pin has the debouncing function attached, call it. */
    if ( interrupt_state->debouncing_tick_count > 0 ) {
      rv = debounce_switch(interrupt_state);

      /* If the handler call was caused by a switch bounce,
       * ignores and move on. */
      if ( rv < 0 ) {
        RELEASE_LOCK(gpio_bank_state[bank].lock);

        continue;
      }
    }

    handler_list = &interrupt_state->handler_chain;

    node = rtems_chain_first(handler_list);

    /* Iterate the ISR list. */
    while ( !rtems_chain_is_tail(handler_list, node) ) {
      isr_node = (gpio_handler_node *) node;

      next_node = node->next;

      if ( (isr_node->handler)(isr_node->arg) == IRQ_HANDLED ) {
        ++handled_count;
      }

      node = next_node;
    }

    /* If no handler assumed the interrupt, treat it as a spurious interrupt. */
    if ( handled_count == 0 ) {
      bsp_interrupt_handler_default(rtems_gpio_bsp_get_vector(bank));
    }

    RELEASE_LOCK(gpio_bank_state[bank].lock);
  }
}

static void generic_isr(void *arg)
{
  rtems_vector_number vector;
  uint32_t event_status;
  uint32_t bank_pin_count;
  uint32_t bank_number;
  uint8_t i;

  bank_number = *((uint32_t*) arg);

  assert ( bank_number >= 0 && bank_number < GPIO_BANK_COUNT );

  vector = rtems_gpio_bsp_get_vector(bank_number);

  /* If the current bank is the last bank, which may not be completely filled. */
  if ( bank_number == GPIO_BANK_COUNT - 1 ) {
    bank_pin_count = GPIO_LAST_BANK_PINS;
  }
  else {
    bank_pin_count = BSP_GPIO_PINS_PER_BANK;
  }

  /* Prevents more interrupts from being generated on GPIO. */
  bsp_interrupt_vector_disable(vector);

  /* Ensure that interrupts are disabled in this vector, before checking
   * the interrupt line. */
  RTEMS_COMPILER_MEMORY_BARRIER(); // TODO: move to bsp_interrupt_vector_disable?

  /* Obtains a 32-bit bitmask, with the pins currently reporting interrupts
   * signaled with 1. */
  event_status = rtems_gpio_bsp_interrupt_line(vector);

  /* Iterates through the bitmask and calls the corresponding handler
   * for active interrupts. */
  for ( i = 0; i < bank_pin_count; ++i ) {
    /* If active, wake the corresponding pin's ISR task. */
    if ( event_status & (1 << i) ) {
      rtems_event_send(
        gpio_pin_state[bank_number][i].interrupt_state->handler_task_id,
        GPIO_INTERRUPT_EVENT
      );
    }
  }

  /* Clear all active events. */
  rtems_gpio_bsp_clear_interrupt_line(vector, event_status);

  /* Ensure that the interrupt line is cleared before re-activating
   * the interrupts on this vector. */
  RTEMS_COMPILER_MEMORY_BARRIER(); // TODO: move to bsp_interrupt_vector_enable?

  bsp_interrupt_vector_enable(vector);
}

/* Verifies if all pins in the received pin array are from the same bank and
 * are configured as digital outputs. Produces bitmask of the received pins. */
static rtems_status_code get_pin_bitmask(
  uint32_t *pins,
  uint32_t pin_count,
  uint32_t *bank_number,
  uint32_t *bitmask
) {
  uint32_t pin_number;
  uint32_t bank;
  uint32_t pin;
  uint8_t i;

  if ( pin_count < 1 ) {
    return RTEMS_UNSATISFIED;
  }

  *bitmask = 0;

  for ( i = 0; i < pin_count; ++i ) {
    pin_number = pins[i];

    if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
      return RTEMS_INVALID_ID;
    }

    bank = BANK_NUMBER(pin_number);
    pin = PIN_NUMBER(pin_number);

    if ( i == 0 ) {
      *bank_number = bank;

      ACQUIRE_LOCK(gpio_bank_state[bank].lock);
    }
    else if ( bank != *bank_number ) {
      RELEASE_LOCK(gpio_bank_state[*bank_number].lock);

      return RTEMS_UNSATISFIED;
    }

    if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_OUTPUT ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return RTEMS_NOT_CONFIGURED;
    }

    *bitmask |= (1 << pin);
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

static rtems_status_code check_same_bank_and_availability(
  const rtems_gpio_pin_conf *pin_confs,
  uint32_t pin_count,
  uint32_t *bank_number,
  uint32_t *pins
) {
  uint32_t bank;
  uint32_t pin;
  uint8_t i;

  for ( i = 0; i < pin_count; ++i ) {
    bank = BANK_NUMBER(pin_confs[i].pin_number);
    pin = PIN_NUMBER(pin_confs[i].pin_number);

    if ( i == 0 ) {
      *bank_number = bank;

      ACQUIRE_LOCK(gpio_bank_state[bank].lock);
    }
    else if ( bank != *bank_number ) {
      RELEASE_LOCK(gpio_bank_state[*bank_number].lock);

      return RTEMS_UNSATISFIED;
    }

    if ( gpio_pin_state[bank][pin].pin_function != NOT_USED ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return RTEMS_NOT_CONFIGURED;
    }

    pins[i] = PIN_NUMBER(pin_confs[i].pin_number);
  }

  RELEASE_LOCK(gpio_bank_state[*bank_number].lock);

  return RTEMS_SUCCESSFUL;
}

static rtems_status_code setup_resistor_and_interrupt_configuration(
  uint32_t pin_number,
  rtems_gpio_pull_mode pull_mode,
  rtems_gpio_interrupt_configuration *interrupt_conf
) {
  gpio_pin_interrupt_state *interrupt_state;
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  sc = rtems_gpio_resistor_mode(pin_number, pull_mode);

  if ( sc != RTEMS_SUCCESSFUL ) {
#if defined(DEBUG)
    printk("rtems_gpio_resistor_mode failed with status code %d\n", sc);
#endif

    return RTEMS_UNSATISFIED;
  }

  if ( interrupt_conf != NULL ) {
    bank = BANK_NUMBER(pin_number);
    pin = PIN_NUMBER(pin_number);

    ACQUIRE_LOCK(gpio_bank_state[bank].lock);

    sc = rtems_gpio_enable_interrupt(
           pin_number,
           interrupt_conf->active_interrupt,
           interrupt_conf->handler_flag,
           interrupt_conf->handler,
           interrupt_conf->arg
         );

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

#if defined(DEBUG)
      printk(
        "rtems_gpio_enable_interrupt failed with status code %d\n",
        sc
      );
#endif

      return RTEMS_UNSATISFIED;
    }

    interrupt_state = gpio_pin_state[bank][pin].interrupt_state;

    interrupt_state->debouncing_tick_count =
      interrupt_conf->debounce_clock_tick_interval;

    interrupt_state->last_isr_tick = 0;

    RELEASE_LOCK(gpio_bank_state[bank].lock);
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_initialize(void)
{
  rtems_status_code sc;
  uint32_t bank_pins;
  uint32_t bank;
  uint32_t pin;
  uint32_t i;

  if ( _Atomic_Flag_test_and_set(&init_flag, ATOMIC_ORDER_RELAXED) == true ) {
    return RTEMS_SUCCESSFUL;
  }

  for ( i = 0; i < GPIO_BANK_COUNT; ++i ) {
    sc = CREATE_LOCK(
           rtems_build_name('G', 'I', 'N', 'T'),
           &gpio_bank_state[i].lock
         );

    if ( sc != RTEMS_SUCCESSFUL ) {
      return sc;
    }

    gpio_bank_state[i].bank_number = i;
    gpio_bank_state[i].interrupt_counter = 0;
  }

  /* Create GPIO pin state matrix. */
  gpio_pin_state = (gpio_pin **) malloc(GPIO_BANK_COUNT * sizeof(gpio_pin *));

  for ( i = 0; i < GPIO_BANK_COUNT; ++i ) {
    if ( i == GPIO_BANK_COUNT - 1 ) {
      bank_pins = GPIO_LAST_BANK_PINS;
    }
    else {
      bank_pins = BSP_GPIO_PINS_PER_BANK;
    }

    gpio_pin_state[i] = (gpio_pin *) malloc(bank_pins * sizeof(gpio_pin));
  }

  for ( i = 0; i < BSP_GPIO_PIN_COUNT; ++i ) {
    bank = BANK_NUMBER(i);
    pin = PIN_NUMBER(i);

    gpio_pin_state[bank][pin].pin_function = NOT_USED;
    gpio_pin_state[bank][pin].resistor_mode = NO_PULL_RESISTOR;
    gpio_pin_state[bank][pin].logic_invert = false;
    gpio_pin_state[bank][pin].interrupt_state = NULL;
  }

  /* Initialize GPIO groups chain. */
  rtems_chain_initialize_empty(&gpio_group);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_pin_group(
  const rtems_gpio_group_definition *group_definition,
  rtems_gpio_group *group
) {
  rtems_status_code sc;

  if ( group_definition == NULL || group == NULL ) {
    return RTEMS_UNSATISFIED;
  }

  group->digital_inputs =
    (uint32_t *) malloc(group_definition->input_count * sizeof(uint32_t));

  group->input_count = group_definition->input_count;

  /* Evaluate if the pins that will constitute the group are available and
   * that pins with the same function within the group all belong
   * to the same pin group. */
  sc = check_same_bank_and_availability(
         group_definition->digital_inputs,
         group->input_count,
         &group->digital_input_bank,
         group->digital_inputs
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  group->digital_outputs =
    (uint32_t *) malloc(group_definition->output_count * sizeof(uint32_t));

  group->output_count = group_definition->output_count;

  sc = check_same_bank_and_availability(
         group_definition->digital_outputs,
         group->output_count,
         &group->digital_output_bank,
         group->digital_outputs
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  group->bsp_speficifc_pins =
    (uint32_t *) malloc(
                   group_definition->bsp_specific_pin_count *
                   sizeof(uint32_t)
                 );

  group->bsp_specific_pin_count = group_definition->bsp_specific_pin_count;

  sc = check_same_bank_and_availability(
         group_definition->bsp_specifics,
         group->bsp_specific_pin_count,
         &group->bsp_specific_bank,
         group->bsp_speficifc_pins
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  /* Request the pins. */
  sc = rtems_gpio_multi_select(
         group_definition->digital_inputs,
         group_definition->input_count
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_multi_select(
         group_definition->digital_outputs,
         group_definition->output_count
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  sc = rtems_gpio_multi_select(
         group_definition->bsp_specifics,
         group_definition->bsp_specific_pin_count
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  /* Create group lock. */
  sc = CREATE_LOCK(rtems_build_name('G', 'R', 'P', 'L'), &group->group_lock);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  rtems_chain_append(&gpio_group, &group->node);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_write_group(uint32_t data, rtems_gpio_group group)
{
  rtems_status_code sc;
  uint32_t set_bitmask;
  uint32_t clear_bitmask;
  uint32_t bank;
  uint32_t pin;
  uint8_t i;

  bank = group.digital_output_bank;

  /* Acquire bank lock for the digital output pins. */
  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  /* Acquire group lock. */
  ACQUIRE_LOCK(group.group_lock);

  set_bitmask = 0;
  clear_bitmask = 0;

  for ( i = 0; i < group.output_count; ++i ) {
    pin = group.digital_outputs[i];

    if ( (data & (1 << i)) == 0 ) {
      clear_bitmask |= (1 << pin);
    }
    else {
      set_bitmask |= (1 << pin);
    }
  }

  /* Set the logical highs. */
  sc = rtems_gpio_bsp_multi_set(bank, set_bitmask);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);
    RELEASE_LOCK(group.group_lock);
  }

  /* Set the logical lows. */
  sc = rtems_gpio_bsp_multi_clear(bank, clear_bitmask);

  RELEASE_LOCK(gpio_bank_state[bank].lock);
  RELEASE_LOCK(group.group_lock);

  return sc;
}

uint32_t rtems_gpio_read_group(rtems_gpio_group group)
{
  uint32_t read_bitmask;
  uint32_t bank;
  uint32_t pin;
  uint32_t rv;
  uint8_t i;

  bank = group.digital_input_bank;

  /* Acquire bank lock for the digital output pins. */
  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  /* Acquire group lock. */
  ACQUIRE_LOCK(group.group_lock);

  read_bitmask = 0;

  for ( i = 0; i < group.input_count; ++i ) {
    pin = group.digital_inputs[i];

    read_bitmask |= (1 << pin);
  }

  rv = rtems_gpio_bsp_multi_read(bank, read_bitmask);

  RELEASE_LOCK(gpio_bank_state[bank].lock);
  RELEASE_LOCK(group.group_lock);

  return rv;
}

rtems_status_code rtems_gpio_multi_select(
  const rtems_gpio_pin_conf *pins,
  uint32_t pin_count
) {
  rtems_status_code sc;
  uint32_t pin_number;
  uint32_t bank;
  uint32_t pin;
  uint8_t i;

  /* If the BSP has multi select capabilities. */
#ifdef BSP_GPIO_PINS_PER_SELECT_BANK
  rtems_gpio_multiple_pin_select
    pin_data[GPIO_SELECT_BANK_COUNT][BSP_GPIO_PINS_PER_SELECT_BANK];
  rtems_gpio_specific_data *bsp_data;

  /* Since each platform may have more than two functions to assign to a pin,
   * each pin requires more than one bit in the selection register to
   * properly assign a function to it.
   * Therefore a selection bank (pin selection register) will support fewer pins
   * than a regular bank, meaning that there will be more selection banks than
   * regular banks, which have to be handled separately.
   *
   * This field records the select bank number relative to the GPIO bank. */
  uint32_t select_bank;
  uint32_t bank_number;
  uint32_t select_bank_counter[GPIO_SELECT_BANK_COUNT];
  uint32_t select_count;

  for ( i = 0; i < GPIO_SELECT_BANK_COUNT; ++i ) {
    select_bank_counter[i] = 0;
  }

  for ( i = 0; i < pin_count; ++i ) {
    pin_number = pins[i].pin_number;

    if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
      return RTEMS_INVALID_ID;
    }

    bank = BANK_NUMBER(pin_number);
    pin = PIN_NUMBER(pin_number);

    if ( i == 0 ) {
      bank_number = bank;

      ACQUIRE_LOCK(gpio_bank_state[bank].lock);
    }
    else if ( bank != bank_number ) {
      RELEASE_LOCK(gpio_bank_state[bank_number].lock);

      return RTEMS_UNSATISFIED;
    }

    /* If the pin is already being used returns with an error. */
    if ( gpio_pin_state[bank][pin].pin_function != NOT_USED ) {
      RELEASE_LOCK(gpio_bank_state[bank_number].lock);

      return RTEMS_RESOURCE_IN_USE;
    }

    select_bank = (pins[i].pin_number / BSP_GPIO_PINS_PER_SELECT_BANK) -
                  (bank * GPIO_SELECT_BANK_COUNT);

    select_count = select_bank_counter[select_bank];

    pin_data[select_bank][select_count].pin_number = pin_number;
    pin_data[select_bank][select_count].function = pins[i].function;

    if ( pins[i].function == BSP_SPECIFIC ) {
      bsp_data = (rtems_gpio_specific_data *) pins[i].bsp_specific;

      if ( bsp_data == NULL ) {
        RELEASE_LOCK(gpio_bank_state[bank_number].lock);

        return RTEMS_UNSATISFIED;
      }

      pin_data[select_bank][select_count].io_function = bsp_data->io_function;
      pin_data[select_bank][select_count].bsp_specific = bsp_data->pin_data;
    }
    else {
      /* io_function takes a dummy value, as it will not be used. */
      pin_data[select_bank][select_count].io_function = 0;
      pin_data[select_bank][select_count].bsp_specific = pins[i].bsp_specific;
    }

    ++select_bank_counter[select_bank];
  }

  for ( i = 0; i < GPIO_SELECT_BANK_COUNT; ++i ) {
    if ( select_bank_counter[i] == 0 ) {
      continue;
    }

    sc = rtems_gpio_bsp_multi_select(
           pin_data[i], select_bank_counter[i], i +
           (bank_number * GPIO_SELECT_BANK_COUNT)
         );

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank_number].lock);

      return sc;
    }
  }

  for ( i = 0; i < pin_count; ++i ) {
    sc = setup_resistor_and_interrupt_configuration(
           pins[i].pin_number,
           pins[i].pull_mode,
           pins[i].interrupt
         );

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank_number].lock);

      return sc;
    }

    bank = BANK_NUMBER(pins[i].pin_number);
    pin = PIN_NUMBER(pins[i].pin_number);

    /* Fill other pin state information. */
    gpio_pin_state[bank][pin].pin_function = pins[i].function;
    gpio_pin_state[bank][pin].logic_invert = pins[i].logic_invert;

    if ( pins[i].function == DIGITAL_OUTPUT ) {
      if ( pins[i].output_enabled == true ) {
        sc = rtems_gpio_bsp_set(bank, pin);
      }
      else {
        sc = rtems_gpio_bsp_clear(bank, pin);
      }
    }
  }

  RELEASE_LOCK(gpio_bank_state[bank_number].lock);

  return sc;

  /* If the BSP does not provide pin multi-selection,
   * configures each pin sequentially. */
#else
  for ( i = 0; i < pin_count; ++i ) {
    pin_number = pins[i].pin_number;

    if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
      return RTEMS_INVALID_ID;
    }

    bank = BANK_NUMBER(pin_number);
    pin = PIN_NUMBER(pin_number);

    ACQUIRE_LOCK(gpio_bank_state[bank].lock);

    /* If the pin is already being used returns with an error. */
    if ( gpio_pin_state[bank][pin].pin_function != NOT_USED ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return RTEMS_RESOURCE_IN_USE;
    }
  }

  for ( i = 0; i < pin_count; ++i ) {
    sc = rtems_gpio_request_configuration(&pins[i]);

    if ( sc != RTEMS_SUCCESSFUL ) {
      return sc;
    }
  }

  return RTEMS_SUCCESSFUL;
#endif
}

rtems_status_code rtems_gpio_request_configuration(
  const rtems_gpio_pin_conf *conf
) {
  rtems_status_code sc;

  sc = rtems_gpio_request_pin(
         conf->pin_number,
         conf->function,
         conf->output_enabled,
         conf->logic_invert,
         conf->bsp_specific
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
#if defined(DEBUG)
    printk("rtems_gpio_request_pin failed with status code %d\n",sc);
#endif

    return RTEMS_UNSATISFIED;
  }

  return setup_resistor_and_interrupt_configuration(
           conf->pin_number,
           conf->pull_mode,
           conf->interrupt
         );
}

rtems_status_code rtems_gpio_update_configuration(
  const rtems_gpio_pin_conf *conf
) {
  rtems_gpio_interrupt_configuration *interrupt_conf;
  gpio_pin_interrupt_state *interrupt_state;
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( conf->pin_number < 0 || conf->pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(conf->pin_number);
  pin = PIN_NUMBER(conf->pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  /* If the pin is not being used returns with an error. */
  if ( gpio_pin_state[bank][pin].pin_function == NOT_USED ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }

  sc = rtems_gpio_resistor_mode(conf->pin_number, conf->pull_mode);

  if ( sc != RTEMS_SUCCESSFUL ) {
#if defined(DEBUG)
    printk("rtems_gpio_resistor_mode failed with status code %d\n", sc);
#endif

    return RTEMS_UNSATISFIED;
  }

  interrupt_conf = (rtems_gpio_interrupt_configuration *) conf->interrupt;

  interrupt_state = gpio_pin_state[bank][pin].interrupt_state;

  if ( interrupt_state != NULL ) {
    sc = rtems_gpio_disable_interrupt(conf->pin_number);

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

#if defined(DEBUG)
      printk(
        "rtems_gpio_disable_interrupt failed with status code %d\n",
        sc
      );
#endif

      return RTEMS_UNSATISFIED;
    }
  }

  if ( interrupt_conf != NULL ) {
    sc = rtems_gpio_enable_interrupt(
           conf->pin_number,
           interrupt_conf->active_interrupt,
           interrupt_conf->handler_flag,
           interrupt_conf->handler,
           interrupt_conf->arg
         );

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

#if defined(DEBUG)
      printk(
        "rtems_gpio_enable_interrupt failed with status code %d\n",
        sc
      );
#endif

      return RTEMS_UNSATISFIED;
    }
  }

  if ( interrupt_conf != NULL && interrupt_state != NULL ) {
    if (
        interrupt_conf->debounce_clock_tick_interval !=
        interrupt_state->debouncing_tick_count
    ) {
      interrupt_state->debouncing_tick_count =
        interrupt_conf->debounce_clock_tick_interval;

      interrupt_state->last_isr_tick = 0;
    }
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_multi_set(
 uint32_t *pin_numbers,
 uint32_t pin_count
) {
  rtems_status_code sc;
  uint32_t bitmask;
  uint32_t bank;

  sc = get_pin_bitmask(pin_numbers, pin_count, &bank, &bitmask);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  sc = rtems_gpio_bsp_multi_set(bank, bitmask);

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return sc;
}

rtems_status_code rtems_gpio_multi_clear(
  uint32_t *pin_numbers,
  uint32_t pin_count
) {
  rtems_status_code sc;
  uint32_t bitmask;
  uint32_t bank;

  sc = get_pin_bitmask(pin_numbers, pin_count, &bank, &bitmask);

  if ( sc != RTEMS_SUCCESSFUL ) {
    return sc;
  }

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  sc = rtems_gpio_bsp_multi_clear(bank, bitmask);

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return sc;
}

rtems_status_code rtems_gpio_set(uint32_t pin_number)
{
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_OUTPUT ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

#if defined(DEBUG)
    printk("Can only set digital output pins\n");
#endif

    return RTEMS_NOT_CONFIGURED;
  }

  if ( gpio_pin_state[bank][pin].logic_invert ) {
    sc = rtems_gpio_bsp_clear(bank, pin);
  }
  else {
    sc = rtems_gpio_bsp_set(bank, pin);
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return sc;
}

rtems_status_code rtems_gpio_clear(uint32_t pin_number)
{
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_OUTPUT ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

#if defined(DEBUG)
    printk("Can only clear digital output pins\n");
#endif

    return RTEMS_NOT_CONFIGURED;
  }

  if ( gpio_pin_state[bank][pin].logic_invert ) {
    sc = rtems_gpio_bsp_set(bank, pin);
  }
  else {
    sc = rtems_gpio_bsp_clear(bank, pin);
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return sc;
}

int rtems_gpio_get_value(uint32_t pin_number)
{
  uint32_t bank;
  uint32_t pin;
  int rv;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return -1;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_INPUT) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

#if defined(DEBUG)
    printk("Can only read digital input pins\n");
#endif

    return -1;
  }

  rv = rtems_gpio_bsp_get_value(bank, pin);

  if ( gpio_pin_state[bank][pin].logic_invert && rv > 0 ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return !rv;
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return ( rv > 0 ) ? 1 : rv;
}

rtems_status_code rtems_gpio_request_pin(
  uint32_t pin_number,
  rtems_gpio_function function,
  bool output_enabled,
  bool logic_invert,
  void *bsp_specific
) {
  rtems_gpio_specific_data *bsp_data;
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  /* If the pin is already being used returns with an error. */
  if ( gpio_pin_state[bank][pin].pin_function != NOT_USED ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_RESOURCE_IN_USE;
  }

  switch ( function ) {
    case DIGITAL_INPUT:
      sc = rtems_gpio_bsp_select_input(bank, pin, bsp_specific);
      break;
    case DIGITAL_OUTPUT:
      sc = rtems_gpio_bsp_select_output(bank, pin, bsp_specific);
      break;
    case BSP_SPECIFIC:
      bsp_data = (rtems_gpio_specific_data *) bsp_specific;

      if ( bsp_data == NULL ) {
        RELEASE_LOCK(gpio_bank_state[bank].lock);

        return RTEMS_UNSATISFIED;
      }

      sc = rtems_bsp_select_specific_io(
             bank,
             pin,
             bsp_data->io_function,
             bsp_data->pin_data
           );
      break;
    case NOT_USED:
    default:
      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return RTEMS_NOT_DEFINED;
  }

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return sc;
  }

  /* If the function was successfully assigned to the pin,
   * record that information on the gpio_pin_state structure. */
  gpio_pin_state[bank][pin].pin_function = function;
  gpio_pin_state[bank][pin].logic_invert = logic_invert;

  if ( function == DIGITAL_OUTPUT ) {
    if ( output_enabled == true ) {
      sc = rtems_gpio_bsp_set(bank, pin);
    }
    else {
      sc = rtems_gpio_bsp_clear(bank, pin);
    }
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return sc;
}

rtems_status_code rtems_gpio_resistor_mode(
  uint32_t pin_number,
  rtems_gpio_pull_mode mode
) {
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  /* If the desired actuation mode is already set, silently exits.
   * The NO_PULL_RESISTOR is a special case, as some platforms have
   * pull-up resistors enabled on startup, so this state may have to
   * be reinforced in the hardware. */
  if (
      gpio_pin_state[bank][pin].resistor_mode == mode &&
      mode != NO_PULL_RESISTOR
  ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_SUCCESSFUL;
  }

  sc = rtems_gpio_bsp_set_resistor_mode(bank, pin, mode);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return sc;
  }

  gpio_pin_state[bank][pin].resistor_mode = mode;

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_release_pin(uint32_t pin_number)
{
  gpio_pin_interrupt_state *interrupt_state;
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  interrupt_state = gpio_pin_state[bank][pin].interrupt_state;

  /* If the pin has an enabled interrupt then remove the handler(s)
   * and disable interrupts on that pin. */
  if ( interrupt_state != NULL ) {
    sc = rtems_gpio_disable_interrupt(pin_number);

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return sc;
    }
  }

  gpio_pin_state[bank][pin].pin_function = NOT_USED;
  gpio_pin_state[bank][pin].resistor_mode = NO_PULL_RESISTOR;
  gpio_pin_state[bank][pin].logic_invert = false;

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_debounce_switch(uint32_t pin_number, int ticks)
{
  gpio_pin_interrupt_state *interrupt_state;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  interrupt_state = gpio_pin_state[bank][pin].interrupt_state;

  /* If no interrupt configuration is set for this pin, or if the pin is
   * not set as a digital input. */
  if (
      interrupt_state == NULL ||
      gpio_pin_state[bank][pin].pin_function != DIGITAL_INPUT
  ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }

  interrupt_state->debouncing_tick_count = ticks;
  interrupt_state->last_isr_tick = 0;

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_interrupt_handler_install(
  uint32_t pin_number,
  rtems_gpio_irq_state (*handler) (void *arg),
  void *arg
) {
  gpio_pin_interrupt_state *interrupt_state;
  gpio_handler_node *isr_node;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  interrupt_state = gpio_pin_state[bank][pin].interrupt_state;

  /* If no interrupt configuration is set for this pin. */
   if ( interrupt_state == NULL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }

  /* If the current pin has no interrupt enabled
   * then it does not need an handler. */
  if ( interrupt_state->active_interrupt == NONE ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }
  /* If the pin already has an enabled interrupt but the installed handler
   * is set as unique. */
  else if (
           interrupt_state->handler_flag == UNIQUE_HANDLER &&
           !rtems_chain_is_empty(&interrupt_state->handler_chain)
  ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_TOO_MANY;
  }

  /* Update the pin's ISR list. */
  isr_node = (gpio_handler_node *) malloc(sizeof(gpio_handler_node));

  if ( isr_node == NULL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NO_MEMORY;
  }

  isr_node->handler = handler;
  isr_node->arg = arg;

  rtems_chain_append(&interrupt_state->handler_chain, &isr_node->node);

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_enable_interrupt(
  uint32_t pin_number,
  rtems_gpio_interrupt interrupt,
  rtems_gpio_handler_flag flag,
  rtems_gpio_irq_state (*handler) (void *arg),
  void *arg
) {
  gpio_pin_interrupt_state *interrupt_state;
  rtems_vector_number vector;
  rtems_status_code sc;
  gpio_pin *gpio;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  vector = rtems_gpio_bsp_get_vector(bank);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  gpio = &gpio_pin_state[bank][pin];

  if ( gpio->pin_function != DIGITAL_INPUT ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }

  interrupt_state = gpio_pin_state[bank][pin].interrupt_state;

  /* If an interrupt configuration is already in place for this pin. */
  if ( interrupt_state != NULL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_RESOURCE_IN_USE;
  }

  gpio_pin_state[bank][pin].interrupt_state =
    (gpio_pin_interrupt_state *) malloc(sizeof(gpio_pin_interrupt_state));

  if ( gpio_pin_state[bank][pin].interrupt_state == NULL ) {
    return RTEMS_NO_MEMORY;
  }

  gpio_pin_state[bank][pin].interrupt_state->bank_number = bank;
  gpio_pin_state[bank][pin].interrupt_state->active_interrupt = NONE;
  gpio_pin_state[bank][pin].interrupt_state->handler_task_id = RTEMS_ID_NONE;
  gpio_pin_state[bank][pin].interrupt_state->debouncing_tick_count = 0;
  gpio_pin_state[bank][pin].interrupt_state->last_isr_tick = 0;

  rtems_chain_initialize_empty(
    &gpio_pin_state[bank][pin].interrupt_state->handler_chain
  );

  interrupt_state = gpio->interrupt_state;

  /* Creates and starts a new task which will call the corresponding
   * user-defined handlers for this pin. */
  sc = rtems_task_create(
         rtems_build_name('G', 'P', 'I', 'O'),
         5,
         RTEMS_MINIMUM_STACK_SIZE * 2,
         RTEMS_NO_TIMESLICE,
         RTEMS_DEFAULT_ATTRIBUTES,
         &interrupt_state->handler_task_id
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_UNSATISFIED;
  }

  sc = rtems_task_start(
         interrupt_state->handler_task_id,
         generic_handler_task,
         (rtems_task_argument) interrupt_state
       );

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    sc = rtems_task_delete(interrupt_state->handler_task_id);

    assert( sc == RTEMS_SUCCESSFUL );

    return RTEMS_UNSATISFIED;
  }

  interrupt_state->active_interrupt = interrupt;
  interrupt_state->handler_flag = flag;

  /* Installs the interrupt handler. */
  sc = rtems_gpio_interrupt_handler_install(pin_number, handler, arg);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    sc = rtems_task_delete(interrupt_state->handler_task_id);

    assert( sc == RTEMS_SUCCESSFUL );

    return RTEMS_UNSATISFIED;
  }

  /* If the generic ISR has not been yet installed for this bank, installs it.
   * This ISR will be responsible for calling the handler tasks,
   * which in turn will call the user-defined interrupt handlers.*/
  if ( gpio_bank_state[bank].interrupt_counter == 0 ) {
    sc = rtems_interrupt_handler_install(
           vector,
           "GPIO_HANDLER",
           RTEMS_INTERRUPT_UNIQUE,
           (rtems_interrupt_handler) generic_isr,
           &gpio_bank_state[bank].bank_number
         );

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return RTEMS_UNSATISFIED;
    }
  }

  sc = rtems_bsp_enable_interrupt(bank, pin, interrupt);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_UNSATISFIED;
  }

  ++gpio_bank_state[bank].interrupt_counter;

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_interrupt_handler_remove(
  uint32_t pin_number,
  rtems_gpio_irq_state (*handler) (void *arg),
  void *arg
) {
  gpio_pin_interrupt_state *interrupt_state;
  rtems_chain_control *handler_list;
  rtems_chain_node *node;
  rtems_chain_node *next_node;
  gpio_handler_node *isr_node;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  interrupt_state = gpio_pin_state[bank][pin].interrupt_state;

  /* If no interrupt configuration is set for this pin. */
  if ( interrupt_state == NULL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }

  handler_list = &interrupt_state->handler_chain;

  node = rtems_chain_first(handler_list);

  /* If the first node is also the last handler for this pin, disables
   * interrupts on this pin as there will be no handler to handle it.
   * This also removes the remaining handler. */
  if ( rtems_chain_is_last(node) ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return rtems_gpio_disable_interrupt(pin_number);
  }

  /* Iterate the ISR list. */
  while ( !rtems_chain_is_tail(handler_list, node) ) {
    isr_node = (gpio_handler_node *) node;

    next_node = node->next;

    if ( isr_node->handler == handler && isr_node->arg == arg ) {
      rtems_chain_extract(node);

      break;
    }

    node = next_node;
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_disable_interrupt(uint32_t pin_number)
{
  gpio_pin_interrupt_state *interrupt_state;
  rtems_chain_control *handler_list;
  rtems_chain_node *node;
  rtems_chain_node *next_node;
  rtems_vector_number vector;
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  vector = rtems_gpio_bsp_get_vector(bank);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  interrupt_state = gpio_pin_state[bank][pin].interrupt_state;

  /* If no interrupt configuration is set for this pin. */
  if ( interrupt_state == NULL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }

  sc = rtems_bsp_disable_interrupt(bank, pin, interrupt_state->active_interrupt);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_UNSATISFIED;
  }

  interrupt_state->active_interrupt = NONE;

  handler_list = &interrupt_state->handler_chain;

  node = rtems_chain_first(handler_list);

  /* Iterate the ISR list. */
  while ( !rtems_chain_is_tail(handler_list, node) ) {
    next_node = node->next;

    rtems_chain_extract(node);

    node = next_node;
  }

  sc = rtems_task_delete(interrupt_state->handler_task_id);

  assert( sc == RTEMS_SUCCESSFUL );

  /* Free the pin's interrupt state structure. */
  free(interrupt_state);

  --gpio_bank_state[bank].interrupt_counter;

  /* If no GPIO interrupts are left in this bank, removes the handler. */
  if ( gpio_bank_state[bank].interrupt_counter == 0 ) {
    sc = rtems_interrupt_handler_remove(
           vector,
           (rtems_interrupt_handler) generic_isr,
           &gpio_bank_state[bank].bank_number
         );

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return RTEMS_UNSATISFIED;
    }
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}
