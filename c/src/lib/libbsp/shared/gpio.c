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
#include <rtems/status-checks.h>
#include <bsp/irq-generic.h>
#include <bsp/gpio.h>

#include <assert.h>
#include <stdlib.h>

/**
 * @brief GPIO API mutex attributes.
 */
#define MUTEX_ATTRIBUTES                        \
  ( RTEMS_LOCAL                                 \
    | RTEMS_PRIORITY                            \
    | RTEMS_BINARY_SEMAPHORE                    \
    | RTEMS_INHERIT_PRIORITY                    \
    | RTEMS_NO_PRIORITY_CEILING                 \
    )

#define ACQUIRE_LOCK(m) assert( rtems_semaphore_obtain(m,               \
                                                       RTEMS_WAIT,      \
                                                       RTEMS_NO_TIMEOUT \
                                                       ) == RTEMS_SUCCESSFUL )

#define RELEASE_LOCK(m) assert( rtems_semaphore_release(m) == RTEMS_SUCCESSFUL )

#define GPIO_INTERRUPT_EVENT RTEMS_EVENT_1

/**
 * @brief Object containing relevant information to a list of user-defined
 *        interrupt handlers.
 *
 * Encapsulates relevant data for a GPIO interrupt handler.
 */
typedef struct gpio_handler_node
{
  rtems_chain_node node;

  /* User-defined ISR routine. */
  rtems_gpio_irq_state (*handler) (void *arg);

  /* User-defined arguments for the ISR routine. */
  void *arg;
} gpio_handler_node;

/**
 * @brief Object containing information on a GPIO pin.
 *
 * Encapsulates relevant data about a GPIO pin.
 */
typedef struct
{
  //uint32_t bank_number;
  //uint32_t pin_number;

  rtems_gpio_function pin_function;

  /* Currently active interrupt. */
  rtems_gpio_interrupt active_interrupt;

  /* Id of the task that will be calling the user-defined ISR handlers
   * for this pin. */
  rtems_id handler_task_id;

  /* ISR shared flag. */
  rtems_gpio_handler_flag handler_flag;

  /* Linked list of interrupt handlers. */
  rtems_chain_control handler_chain;

  rtems_chain_control temp_fix; //FIXME: this field is not need. Its here as a temp fix to an unknown bug (with rpi?) which causes a crash when interrupts are enabled.

  /* GPIO input (pull resistor) pin mode. */
  rtems_gpio_pull_mode resistor_mode;

  /* If true inverts digital in/out logic. */
  bool logic_invert;

  /* Switch-deboucing information. */
  uint32_t debouncing_tick_count;
  rtems_interval last_isr_tick;
} gpio_pin;

typedef struct
{
  uint32_t bank_number;
  uint32_t interrupt_counter;
  rtems_id lock;
} gpio_bank;

static gpio_pin **gpio_pin_state;
static Atomic_Flag init_flag = ATOMIC_INITIALIZER_FLAG;
static gpio_bank gpio_bank_state[GPIO_BANK_COUNT];

#define BANK_NUMBER(pin_number) pin_number / BSP_GPIO_PINS_PER_BANK
#define PIN_NUMBER(pin_number) pin_number % BSP_GPIO_PINS_PER_BANK

static int debounce_switch(gpio_pin *gpio)
{
  rtems_interval time;

  time = rtems_clock_get_ticks_since_boot();

  /* If not enough time has elapsed since last interrupt. */
  if ( (time - gpio->last_isr_tick) < gpio->debouncing_tick_count ) {
    return -1;
  }

  gpio->last_isr_tick = time;

  return 0;
}

static rtems_task generic_handler_task(rtems_task_argument arg)
{
  rtems_chain_control *handler_list;
  rtems_chain_node *node;
  rtems_chain_node *next_node;
  gpio_handler_node *isr_node;
  rtems_event_set out;
  gpio_pin *gpio;
  uint32_t bank;
  int handled_count;
  int rv;

  gpio = (gpio_pin *) arg;

  bank = 0;//TODO gpio->bank_number;

  // TODO asserts

  while ( true ) {
    handled_count = 0;

    /* Wait for interrupt event. This is sent by the bank's generic_isr handler. */
    rtems_event_receive(GPIO_INTERRUPT_EVENT,
                        RTEMS_EVENT_ALL | RTEMS_WAIT,
                        RTEMS_NO_TIMEOUT,
                        &out);

    ACQUIRE_LOCK(gpio_bank_state[bank].lock);

    /* If this pin has the debouncing function attached, call it. */
    if ( gpio->debouncing_tick_count > 0 ) {
      rv = debounce_switch(gpio);

      /* If the handler call was caused by a switch bounce, ignores and move on. */
      if ( rv < 0 ) {
        RELEASE_LOCK(gpio_bank_state[bank].lock);

        continue;
      }
    }

    handler_list = &gpio->handler_chain;

    node = rtems_chain_first(handler_list);

    /* Iterate the ISR list. */
    while ( !rtems_chain_is_tail( handler_list, node ) ) {
      isr_node = (gpio_handler_node *) node;

      next_node = node->next;

      if ( (isr_node->handler)(isr_node->arg) == IRQ_HANDLED ) {
        ++handled_count;
      }
      node = next_node;
    }

    /* If no handler assumed the interrupt, treat it as a spurious interrupt. */
    if ( handled_count == 0 ) {
      bsp_interrupt_handler_default(rtems_bsp_gpio_get_vector(bank));
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
  int i;

  bank_number = *((uint32_t*) arg);

  assert ( bank_number >= 0 && bank_number < GPIO_BANK_COUNT );

  vector = rtems_bsp_gpio_get_vector(bank_number);

  /* If the current bank is an odd last bank (i.e.: not completely filled). */
  if ( GPIO_LAST_BANK_PINS > 0 && bank_number == GPIO_BANK_COUNT - 1 ) {
    bank_pin_count = GPIO_LAST_BANK_PINS;
  }
  else {
    bank_pin_count = BSP_GPIO_PINS_PER_BANK;
  }

  /* Prevents more interrupts from being generated on GPIO. */
  bsp_interrupt_vector_disable(vector);

  /* Ensure that interrupts are disabled in this vector, before checking
   * the interrupt line. */
  RTEMS_COMPILER_MEMORY_BARRIER(); // TODO: move to bsp_interrupt_vector_

  /* Obtains a 32-bit bitmask, with the pins currently reporting interrupts
   * signaled with 1. */
  event_status = rtems_bsp_gpio_interrupt_line(vector);

  /* Iterates through the bitmask and calls the corresponding handler
   * for active interrupts. */
  for ( i = 0; i < bank_pin_count; ++i ) {
    /* If active, wake the corresponding pin's ISR task. */
    if ( event_status & (1 << i) ) {
      rtems_event_send(gpio_pin_state[bank_number][i].handler_task_id, GPIO_INTERRUPT_EVENT);
    }
  }

  /* Clear all active events. */
  rtems_bsp_gpio_clear_interrupt_line(vector, event_status);

  /* Ensure that the interrupt line is cleared before re-activating
   * the interrupts on this vector. */
  RTEMS_COMPILER_MEMORY_BARRIER(); // TODO: move to bsp_interrupt_vector_

  bsp_interrupt_vector_enable(vector);
}

static uint32_t get_pin_bitmask(uint32_t count, va_list args, uint32_t *bank_number, rtems_status_code *sc)
{
  uint32_t bank;
  uint32_t pin;
  uint32_t bitmask;
  uint32_t pin_number;
  int i;

  if ( count < 1 ) {
    *sc = RTEMS_UNSATISFIED;

    return 0;
  }

  bitmask = 0;

  for ( i = 0; i < count; ++i ) {
    pin_number = va_arg(args, uint32_t);

    if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
      *sc = RTEMS_INVALID_ID;

      return 0;
    }

    bank = BANK_NUMBER(pin_number);
    pin = PIN_NUMBER(pin_number);

    if ( i == 0 ) {
      *bank_number = bank;

      ACQUIRE_LOCK(gpio_bank_state[bank].lock);
    }
    else if ( bank != *bank_number ) {
      *sc = RTEMS_UNSATISFIED;

      RELEASE_LOCK(gpio_bank_state[*bank_number].lock);

      return 0;
    }

    if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_OUTPUT ) {
      *sc = RTEMS_NOT_CONFIGURED;

      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return 0;
    }

    bitmask |= (1 << PIN_NUMBER(pin_number));
  }

  *sc = RTEMS_SUCCESSFUL;

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return bitmask;
}

rtems_status_code rtems_gpio_initialize(void)
{
  rtems_status_code sc;
  int bank_pins;
  int pin;
  int bank;
  int i;

  if ( _Atomic_Flag_test_and_set(&init_flag, ATOMIC_ORDER_RELAXED) == true ) {
    return RTEMS_SUCCESSFUL;
  }

  for ( i = 0; i < GPIO_BANK_COUNT; ++i ) {
    /* Create GPIO bank mutex. */
    sc = rtems_semaphore_create(rtems_build_name('G', 'I', 'N', 'T'), 1, MUTEX_ATTRIBUTES, 0, &gpio_bank_state[i].lock);

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

    //gpio_pin_state[bank][pin].bank_number = bank;
    //gpio_pin_state[bank][pin].pin_number = pin;
    gpio_pin_state[bank][pin].pin_function = NOT_USED;
    gpio_pin_state[bank][pin].active_interrupt = NONE;
    gpio_pin_state[bank][pin].handler_task_id = RTEMS_ID_NONE;
    gpio_pin_state[bank][pin].debouncing_tick_count = 0;
    gpio_pin_state[bank][pin].last_isr_tick = 0;

    rtems_chain_initialize_empty(&gpio_pin_state[bank][pin].handler_chain);
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_request_conf(rtems_gpio_pin_conf *conf)
{
  rtems_gpio_interrupt_conf *interrupt_conf;
  rtems_status_code sc;
  bool new_request;
  uint32_t bank;
  uint32_t pin;

  new_request = false;

  sc = rtems_gpio_request_pin(conf->pin_number, conf->function, conf->output_enabled, conf->logic_invert, conf->bsp_specific);

  if ( sc == RTEMS_SUCCESSFUL ) {
    new_request = true;
  }
  /* If the pin is being used, then this function call is an update call.
   * If not, an error occurred. */
  else if ( sc != RTEMS_RESOURCE_IN_USE ) {
    RTEMS_SYSLOG_ERROR("rtems_gpio_request_pin failed with status code %d\n", sc);

    return RTEMS_UNSATISFIED;
  }

  sc = rtems_gpio_resistor_mode(conf->pin_number, conf->pull_mode);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RTEMS_SYSLOG_ERROR("rtems_gpio_resistor_mode failed with status code %d\n", sc);

    return RTEMS_UNSATISFIED;
  }

  interrupt_conf = (rtems_gpio_interrupt_conf *) conf->interrupt;

  if ( interrupt_conf != NULL ) {
    bank = BANK_NUMBER(conf->pin_number);
    pin = PIN_NUMBER(conf->pin_number);

    ACQUIRE_LOCK(gpio_bank_state[bank].lock);

    if ( interrupt_conf->active_interrupt != gpio_pin_state[bank][pin].active_interrupt ) {
      if ( new_request == false ) {
        sc = rtems_gpio_disable_interrupt(conf->pin_number);

        if ( sc != RTEMS_SUCCESSFUL ) {
          RELEASE_LOCK(gpio_bank_state[bank].lock);

          RTEMS_SYSLOG_ERROR("rtems_gpio_disable_interrupt failed with status code %d\n", sc);

          return RTEMS_UNSATISFIED;
        }
      }

      sc = rtems_gpio_enable_interrupt(conf->pin_number,
                                       interrupt_conf->active_interrupt,
                                       interrupt_conf->handler_flag,
                                       interrupt_conf->handler,
                                       interrupt_conf->arg);

      if ( sc != RTEMS_SUCCESSFUL ) {
        RELEASE_LOCK(gpio_bank_state[bank].lock);

        RTEMS_SYSLOG_ERROR("rtems_gpio_enable_interrupt failed with status code %d\n", sc);

        return RTEMS_UNSATISFIED;
      }

    }

    if ( interrupt_conf->clock_tick_interval != gpio_pin_state[bank][pin].debouncing_tick_count ) {
      gpio_pin_state[bank][pin].debouncing_tick_count = interrupt_conf->clock_tick_interval;
      gpio_pin_state[bank][pin].last_isr_tick = 0;
    }

    RELEASE_LOCK(gpio_bank_state[bank].lock);
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_multi_set(uint32_t count, ...)
{
  rtems_status_code sc;
  uint32_t bank;
  uint32_t bitmask;
  va_list ap;

  va_start(ap, count);

  bitmask = get_pin_bitmask(count, ap, &bank, &sc);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RTEMS_SYSLOG_ERROR("error parsing function arguments\n");

    return sc;
  }

  va_end(ap);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  sc = rtems_bsp_gpio_multi_set(bank, bitmask);

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return sc;
}

rtems_status_code rtems_gpio_multi_clear(uint32_t count, ...)
{
  rtems_status_code sc;
  uint32_t bank;
  uint32_t bitmask;
  va_list ap;

  va_start(ap, count);

  bitmask = get_pin_bitmask(count, ap, &bank, &sc);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RTEMS_SYSLOG_ERROR("error parsing function arguments\n");

    return sc;
  }

  va_end(ap);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  sc = rtems_bsp_gpio_multi_clear(bank, bitmask);

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

    RTEMS_SYSLOG_ERROR("Can only set digital output pins\n");

    return RTEMS_NOT_CONFIGURED;
  }

  if ( gpio_pin_state[bank][pin].logic_invert ) {
    sc = rtems_bsp_gpio_clear(bank, pin);
  }
  else {
    sc = rtems_bsp_gpio_set(bank, pin);
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

    RTEMS_SYSLOG_ERROR("Can only clear digital output pins\n");

    return RTEMS_NOT_CONFIGURED;
  }

  if ( gpio_pin_state[bank][pin].logic_invert ) {
    sc = rtems_bsp_gpio_set(bank, pin);
  }
  else {
    sc = rtems_bsp_gpio_clear(bank, pin);
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

    RTEMS_SYSLOG_ERROR("Can only read digital input pins\n");

    return -1;
  }

  rv = rtems_bsp_gpio_get_value(bank, pin);

  if ( gpio_pin_state[bank][pin].logic_invert && rv > 0 ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return !rv;
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return ( rv > 0 ) ? 1 : rv;
}

rtems_status_code rtems_gpio_request_pin(uint32_t pin_number, rtems_gpio_function function, bool output_enabled, bool logic_invert, void *bsp_specific)
{
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
      sc = rtems_bsp_gpio_select_input(bank, pin, bsp_specific);
      break;
    case DIGITAL_OUTPUT:
      sc = rtems_bsp_gpio_select_output(bank, pin, bsp_specific);
      break;
    case BSP_SPECIFIC:
      bsp_data = (rtems_gpio_specific_data *) bsp_specific;

      if ( bsp_data == NULL ) {
        RELEASE_LOCK(gpio_bank_state[bank].lock);

        return RTEMS_UNSATISFIED;
      }

      sc = rtems_bsp_select_specific_io(bank, pin, bsp_data->io_function, bsp_data->pin_data);
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

  /* If the function was successfuly assigned to the pin,
   * record that information on the gpio_pin_state structure. */
  gpio_pin_state[bank][pin].pin_function = function;
  gpio_pin_state[bank][pin].logic_invert = logic_invert;

  if ( function == DIGITAL_OUTPUT ) {
    if ( output_enabled == true ) {
      sc = rtems_bsp_gpio_set(bank, pin);
    }
    else {
      sc = rtems_bsp_gpio_clear(bank, pin);
    }
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return sc;
}

rtems_status_code rtems_gpio_resistor_mode(uint32_t pin_number, rtems_gpio_pull_mode mode)
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

  /* If the desired actuation mode is already set, silently exits.
   * The NO_PULL_RESISTOR is a special case, as some platforms have
   * pull-up resistors enabled on startup, so this state may have to
   * be reinforced in the hardware. */
  if ( gpio_pin_state[bank][pin].resistor_mode == mode  && mode != NO_PULL_RESISTOR ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_SUCCESSFUL;
  }

  sc = rtems_bsp_gpio_set_resistor_mode(bank, pin, mode);

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
  rtems_status_code sc;
  gpio_pin *gpio;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  gpio = &gpio_pin_state[bank][pin];

  /* If the pin has an enabled interrupt then remove the handler(s),
   * and disable the interrupts on that pin. */
  if ( gpio->active_interrupt != NONE ) {
    sc = rtems_gpio_disable_interrupt(pin_number);

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return sc;
    }
  }

  gpio->pin_function = NOT_USED;
  gpio->handler_task_id = RTEMS_ID_NONE;
  gpio->debouncing_tick_count = 0;
  gpio->last_isr_tick = 0;

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_debounce_switch(uint32_t pin_number, int ticks)
{
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_INPUT ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }

  gpio_pin_state[bank][pin].debouncing_tick_count = ticks;
  gpio_pin_state[bank][pin].last_isr_tick = rtems_clock_get_ticks_per_second();

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_interrupt_handler_install(
uint32_t pin_number,
rtems_gpio_irq_state (*handler) (void *arg),
void *arg
)
{
  gpio_handler_node *isr_node;
  gpio_pin *gpio;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  gpio = &gpio_pin_state[bank][pin];

  /* If the current pin has no interrupt enabled
   * then it does not need an handler. */
  if ( gpio->active_interrupt == NONE ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }
  /* If the pin already has an enabled interrupt but the installed handler
   * is set as unique. */
  else if ( gpio->handler_flag == UNIQUE_HANDLER && !rtems_chain_is_empty(&gpio->handler_chain) ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_RESOURCE_IN_USE; // TODO change to too many
  }

  /* Update the pin's ISR list. */
  isr_node = (gpio_handler_node *) malloc(sizeof(gpio_handler_node));

  if ( isr_node == NULL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NO_MEMORY;
  }

  isr_node->handler = handler;
  isr_node->arg = arg;

  rtems_chain_append(&gpio->handler_chain, &isr_node->node);

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_enable_interrupt(
uint32_t pin_number,
rtems_gpio_interrupt interrupt,
rtems_gpio_handler_flag flag,
rtems_gpio_irq_state (*handler) (void *arg),
void *arg
)
{
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

  vector = rtems_bsp_gpio_get_vector(bank);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  gpio = &gpio_pin_state[bank][pin];

  if ( gpio->pin_function != DIGITAL_INPUT ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_NOT_CONFIGURED;
  }

  /* If trying to enable the same type of interrupt on the same pin, or if the pin
   * already has an enabled interrupt. */
  if ( interrupt == gpio->active_interrupt || gpio->active_interrupt != NONE ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_RESOURCE_IN_USE;
  }

  /* Creates and starts a new task which will call the corresponding
   * user-defined handlers for this pin. */
  sc = rtems_task_create(rtems_build_name('G', 'P', 'I', 'O'),
                         5,
                         RTEMS_MINIMUM_STACK_SIZE * 2,
                         RTEMS_NO_TIMESLICE,
                         RTEMS_DEFAULT_ATTRIBUTES,
                         &gpio->handler_task_id);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_UNSATISFIED;
  }

  sc = rtems_task_start(gpio->handler_task_id, generic_handler_task, (rtems_task_argument) gpio);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    sc = rtems_task_delete(gpio->handler_task_id);

    assert( sc == RTEMS_SUCCESSFUL );

    return RTEMS_UNSATISFIED;
  }

  gpio->active_interrupt = interrupt;
  gpio->handler_flag = flag;

  /* Installs the interrupt handler. */
  sc = rtems_gpio_interrupt_handler_install(pin_number, handler, arg);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    sc = rtems_task_delete(gpio->handler_task_id);

    assert( sc == RTEMS_SUCCESSFUL );

    return RTEMS_UNSATISFIED;
  }

  /* If the generic ISR has not been yet installed for this bank, installs it.
   * This ISR will be responsible for calling the handler tasks,
   * which in turn will call the user-defined interrupt handlers.*/
  if ( gpio_bank_state[bank].interrupt_counter == 0 ) {
    sc = rtems_interrupt_handler_install(vector,
                                         "GPIO_HANDLER",
                                         RTEMS_INTERRUPT_UNIQUE,
                                         (rtems_interrupt_handler) generic_isr,
                                         &gpio_bank_state[bank].bank_number);

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
)
{
  rtems_chain_control *handler_list;
  rtems_chain_node *node;
  rtems_chain_node *next_node;
  gpio_handler_node *isr_node;
  gpio_pin *gpio;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= BSP_GPIO_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  gpio = &gpio_pin_state[bank][pin];

  handler_list = &gpio->handler_chain;

  node = rtems_chain_first(handler_list);

  /* Iterate the ISR list. */
  while ( !rtems_chain_is_tail( handler_list, node ) ) {
    isr_node = (gpio_handler_node *) node;

    next_node = node->next;

    if ( isr_node->handler == handler && isr_node->arg == arg ) {
      rtems_chain_extract(node);

      break;
    }

    node = next_node;
  }

  /* If the removed handler was the last for this pin, disables further
   * interrupts on this pin. */
  if ( rtems_chain_is_empty(handler_list) ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return rtems_gpio_disable_interrupt(pin_number);
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_disable_interrupt(uint32_t pin_number)
{
  rtems_chain_control *handler_list;
  rtems_chain_node *node;
  rtems_chain_node *next_node;
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

  vector = rtems_bsp_gpio_get_vector(bank);

  ACQUIRE_LOCK(gpio_bank_state[bank].lock);

  gpio = &gpio_pin_state[bank][pin];

  if ( gpio_bank_state[bank].interrupt_counter == 0 || gpio->active_interrupt == NONE ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_SUCCESSFUL;
  }

  sc = rtems_bsp_disable_interrupt(bank, pin, gpio->active_interrupt);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(gpio_bank_state[bank].lock);

    return RTEMS_UNSATISFIED;
  }

  gpio->active_interrupt = NONE;

  handler_list = &gpio->handler_chain;

  node = rtems_chain_first(handler_list);

  /* Iterate the ISR list. */
  while ( !rtems_chain_is_tail( handler_list, node ) ) {
    next_node = node->next;

    rtems_chain_extract(node);

    node = next_node;
  }

  sc = rtems_task_delete(gpio->handler_task_id);

  assert( sc == RTEMS_SUCCESSFUL );

  --gpio_bank_state[bank].interrupt_counter;

  /* If no GPIO interrupts are left in this bank, removes the handler. */
  if ( gpio_bank_state[bank].interrupt_counter == 0 ) {
    sc = rtems_interrupt_handler_remove(vector,
                                        (rtems_interrupt_handler) generic_isr,
                                        &gpio_bank_state[bank].bank_number);

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(gpio_bank_state[bank].lock);

      return RTEMS_UNSATISFIED;
    }
  }

  RELEASE_LOCK(gpio_bank_state[bank].lock);

  return RTEMS_SUCCESSFUL;
}
