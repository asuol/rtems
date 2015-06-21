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
#include <rtems/status-checks.h>
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

#define AQUIRE_LOCK(m) assert( rtems_semaphore_obtain(m,                \
                                                      RTEMS_WAIT,       \
                                                      RTEMS_NO_TIMEOUT  \
                                                      ) == RTEMS_SUCCESSFUL )

#define RELEASE_LOCK(m) assert( rtems_semaphore_release(m) == RTEMS_SUCCESSFUL )

/*Encapsulates relevant data for a GPIO interrupt handler. */
typedef struct _gpio_handler_list
{
  struct _gpio_handler_list *next_isr;

  /* User-defined ISR routine. */
  rtems_gpio_irq_state (*handler) (void *arg);

  /* User-defined arguments for the ISR routine. */
  void *arg;
} gpio_handler_list;

/* Encapsulates relevant data about a GPIO pin. */
typedef struct
{
  uint32_t bank_number;
  uint32_t pin_number;
  
  rtems_gpio_function pin_function;

  /* Type of event which will trigger an interrupt. */
  rtems_gpio_interrupt enabled_interrupt;

  /* Id of the task that will be calling the user-defined ISR handlers
   * for this pin. */
  rtems_id task_id;

  /* ISR shared flag. */
  rtems_gpio_handler_flag handler_flag;
  
  /* Linked list of interrupt handlers. */
  gpio_handler_list *handler_list;

  /* GPIO input (pull resistor) pin mode. */
  rtems_gpio_pull_mode resistor_mode;

  /* If true inverts digital in/out logic. */
  int logic_invert;
  
  /* Switch-deboucing information. */
  int debouncing_tick_count;
  rtems_interval last_isr_tick;  
} gpio_pin;

static gpio_pin** gpio_pin_state;
static Atomic_Flag init_flag = ATOMIC_INITIALIZER_FLAG;
static uint32_t gpio_count;
static uint32_t bank_count;
static uint32_t pins_per_bank;
static uint32_t odd_bank_pins;
static uint32_t* interrupt_counter;
static rtems_id* bank_lock;

#define BANK_NUMBER(pin_number) pin_number / pins_per_bank
#define PIN_NUMBER(pin_number) pin_number % pins_per_bank

static int debounce_switch(gpio_pin* gpio)
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
  gpio_handler_list *handler_list;
  rtems_event_set out;
  gpio_pin* gpio;
  uint32_t bank;
  int handled_count;
  int rv;

  gpio = (gpio_pin*) arg;

  bank = gpio->bank_number;

  while ( true ) {
    handled_count = 0;

    /* Wait for interrupt event. This is sent by the bank's generic_isr handler. */
    rtems_event_receive(RTEMS_EVENT_1, RTEMS_EVENT_ALL | RTEMS_WAIT,
                        RTEMS_NO_TIMEOUT,
                        &out);

    AQUIRE_LOCK(bank_lock[bank]);
    
    /* If this pin has the debouncing function attached, call it. */
    if ( gpio->debouncing_tick_count > 0 ) {
      rv = debounce_switch(gpio);

      /* If the handler call was caused by a switch bounce, ignores and move on. */
      if ( rv < 0 ) {
        RELEASE_LOCK(bank_lock[bank]);

        continue;
      }

      /* Record the current clock tick. */
      gpio->last_isr_tick = rtems_clock_get_ticks_since_boot();
    }
  
    handler_list = gpio->handler_list;

    /* Iterate the ISR list. */
    while ( handler_list != NULL ) {
      if ( (handler_list->handler)(handler_list->arg) == IRQ_HANDLED ) {
        ++handled_count;
      }

      handler_list = handler_list->next_isr;
    }

    /* If no handler assumed the interrupt, treat it as a spurious interrupt. */
    if ( handled_count == 0 ) {
      bsp_interrupt_handler_default(rtems_bsp_gpio_get_vector(bank));
    }

    RELEASE_LOCK(bank_lock[bank]);
  }
}

rtems_status_code rtems_gpio_initialize(void)
{
  rtems_status_code sc;
  rtems_gpio_layout layout;
  int bank_pins;
  int pin;
  int bank;
  int i;

  if ( _Atomic_Flag_test_and_set(&init_flag, ATOMIC_ORDER_RELAXED) == true ) {
    return RTEMS_SUCCESSFUL;
  }

  layout = rtems_bsp_gpio_initialize();

  gpio_count = layout.pin_count;
  pins_per_bank = layout.pins_per_bank;

  bank_count = gpio_count / pins_per_bank;

  /* Account for remaining pins after filling the last bank. */
  odd_bank_pins = gpio_count % pins_per_bank;
  
  if ( odd_bank_pins > 0 ) {
    ++bank_count;
  }

  /* Create GPIO bank mutexes. */
  bank_lock = (rtems_id*) malloc(bank_count * sizeof(rtems_id));

  for ( i = 0; i < bank_count; ++i ) {    
    sc = rtems_semaphore_create(rtems_build_name('G', 'I', 'N', 'T'), 1, MUTEX_ATRIBUTES, 0, &bank_lock[i]);

    if ( sc != RTEMS_SUCCESSFUL ) {
      return sc;
    }
  }

  /* Create GPIO pin state matrix. */
  gpio_pin_state = (gpio_pin**) malloc(bank_count * sizeof(gpio_pin*));

  for ( i = 0; i < bank_count; ++i ) {
    if ( i == bank_count - 1 ) {
      bank_pins = odd_bank_pins;
    }
    else {
      bank_pins = pins_per_bank;
    }
    
    gpio_pin_state[i] = (gpio_pin*) malloc(bank_pins * sizeof(gpio_pin));
  }

  /* Creates an interrupt counter per pin bank. */
  interrupt_counter = (uint32_t*) calloc(bank_count, sizeof(uint32_t));
  
  for ( i = 0; i < gpio_count; ++i ) {    
    bank = BANK_NUMBER(i);
    pin = PIN_NUMBER(i);

    gpio_pin_state[bank][pin].bank_number = bank;
    gpio_pin_state[bank][pin].pin_number = pin;
    gpio_pin_state[bank][pin].pin_function = NOT_USED;
    gpio_pin_state[bank][pin].enabled_interrupt = NONE;
    gpio_pin_state[bank][pin].task_id = RTEMS_ID_NONE;
    gpio_pin_state[bank][pin].handler_list = NULL;
    gpio_pin_state[bank][pin].debouncing_tick_count = 0;
    gpio_pin_state[bank][pin].last_isr_tick = 0;
  }
  
  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_request_conf(rtems_gpio_pin_conf* conf)
{
  rtems_gpio_interrupt_conf* interrupt_conf;
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

  interrupt_conf = (rtems_gpio_interrupt_conf*) conf->interrupt;

  if ( interrupt_conf != NULL ) {
    bank = BANK_NUMBER(conf->pin_number);
    pin = PIN_NUMBER(conf->pin_number);

    AQUIRE_LOCK(bank_lock[bank]);

    if ( interrupt_conf->enabled_interrupt != gpio_pin_state[bank][pin].enabled_interrupt ) {
      if ( new_request == false ) {
        sc = rtems_gpio_disable_interrupt(conf->pin_number);

        if ( sc != RTEMS_SUCCESSFUL ) {
          RELEASE_LOCK(bank_lock[bank]);
          
          RTEMS_SYSLOG_ERROR("rtems_gpio_disable_interrupt failed with status code %d\n", sc);
    
          return RTEMS_UNSATISFIED;
        }
      }

      sc = rtems_gpio_enable_interrupt(conf->pin_number,
				       interrupt_conf->enabled_interrupt,
				       interrupt_conf->handler_flag,
				       interrupt_conf->handler,
				       interrupt_conf->arg);

      if ( sc != RTEMS_SUCCESSFUL ) {
        RELEASE_LOCK(bank_lock[bank]);
        
        RTEMS_SYSLOG_ERROR("rtems_gpio_enable_interrupt failed with status code %d\n", sc);
    
        return RTEMS_UNSATISFIED;
      }
      
    }

    if ( interrupt_conf->clock_tick_interval != gpio_pin_state[bank][pin].debouncing_tick_count ) {
      gpio_pin_state[bank][pin].debouncing_tick_count = interrupt_conf->clock_tick_interval;
      gpio_pin_state[bank][pin].last_isr_tick = 0;
    }

    RELEASE_LOCK(bank_lock[bank]);
  }

  return RTEMS_SUCCESSFUL;
}

static uint32_t get_pin_bitmask(uint32_t count, va_list args, uint32_t* bank_number, rtems_status_code* sc)
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

    if ( pin_number < 0 || pin_number >= gpio_count ) {
      *sc = RTEMS_INVALID_ID;

      return 0;
    }

    bank = BANK_NUMBER(pin_number);
    pin = PIN_NUMBER(pin_number);

    if ( i == 0 ) {
      *bank_number = bank;

      AQUIRE_LOCK(bank_lock[bank]);
    }
    else if ( bank != *bank_number ) {
      *sc = RTEMS_UNSATISFIED;

      RELEASE_LOCK(bank_lock[*bank_number]);
      
      return 0;
    }

    if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_OUTPUT ) {
      *sc = RTEMS_NOT_CONFIGURED;

      RELEASE_LOCK(bank_lock[bank]);
      
      return 0;
    }
    
    bitmask |= (1 << PIN_NUMBER(pin_number));
  }
  
  *sc = RTEMS_SUCCESSFUL;

  RELEASE_LOCK(bank_lock[bank]);

  return bitmask;
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

  AQUIRE_LOCK(bank_lock[bank]);
  
  sc = rtems_bsp_gpio_multi_set(bank, bitmask);

  RELEASE_LOCK(bank_lock[bank]);

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

  AQUIRE_LOCK(bank_lock[bank]);
  
  sc = rtems_bsp_gpio_multi_clear(bank, bitmask);

  RELEASE_LOCK(bank_lock[bank]);

  return sc;
}

rtems_status_code rtems_gpio_set(uint32_t pin_number)
{
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  AQUIRE_LOCK(bank_lock[bank]);
  
  if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_OUTPUT ) {
    RELEASE_LOCK(bank_lock[bank]);

    RTEMS_SYSLOG_ERROR("Can only set digital output pins\n");
    
    return RTEMS_NOT_CONFIGURED;
  }

  if ( gpio_pin_state[bank][pin].logic_invert ) {
    sc = rtems_bsp_gpio_clear(bank, pin);
  }
  else {
    sc = rtems_bsp_gpio_set(bank, pin);
  }
  
  RELEASE_LOCK(bank_lock[bank]);

  return sc;
}

rtems_status_code rtems_gpio_clear(uint32_t pin_number)
{
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }
  
  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  AQUIRE_LOCK(bank_lock[bank]);
  
  if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_OUTPUT ) {
    RELEASE_LOCK(bank_lock[bank]);

    RTEMS_SYSLOG_ERROR("Can only clear digital output pins\n");
    
    return RTEMS_NOT_CONFIGURED;
  }

  if ( gpio_pin_state[bank][pin].logic_invert ) {
    sc = rtems_bsp_gpio_set(bank, pin);
  }
  else {
    sc = rtems_bsp_gpio_clear(bank, pin);
  }

  RELEASE_LOCK(bank_lock[bank]);
  
  return sc;
}

int rtems_gpio_get_value(uint32_t pin_number)
{
  uint32_t bank;
  uint32_t pin;
  int rv;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return -1;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  AQUIRE_LOCK(bank_lock[bank]);

  if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_INPUT) {
    RELEASE_LOCK(bank_lock[bank]);

    RTEMS_SYSLOG_ERROR("Can only read digital input pins\n");
    
    return -1;
  }

  rv = rtems_bsp_gpio_get_value(bank, pin);

  if ( gpio_pin_state[bank][pin].logic_invert ) {
    RELEASE_LOCK(bank_lock[bank]);
    
    return !rv;
  }

  RELEASE_LOCK(bank_lock[bank]);

  return ( rv > 0 ) ? 1 : 0;
}

rtems_status_code rtems_gpio_request_pin(uint32_t pin_number, rtems_gpio_function function, bool output_enabled, bool logic_invert, void* bsp_specific)
{
  rtems_gpio_specific_data* bsp_data;
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  AQUIRE_LOCK(bank_lock[bank]);

  /* If the pin is already being used returns with an error. */
  if ( gpio_pin_state[bank][pin].pin_function != NOT_USED ) {
    RELEASE_LOCK(bank_lock[bank]);
    
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
      bsp_data = (rtems_gpio_specific_data*) bsp_specific;

      if ( bsp_data == NULL ) {
        RELEASE_LOCK(bank_lock[bank]);
        
        return RTEMS_UNSATISFIED;
      }

      sc = rtems_bsp_select_specific_io(bank, pin, bsp_data->io_function, bsp_data->pin_data);
      break;
    case NOT_USED:
    default:
      RELEASE_LOCK(bank_lock[bank]);
      
      return RTEMS_NOT_DEFINED;
  }

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(bank_lock[bank]);
    
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

  RELEASE_LOCK(bank_lock[bank]);
  
  return sc;
}

rtems_status_code rtems_gpio_resistor_mode(uint32_t pin_number, rtems_gpio_pull_mode mode)
{
  rtems_status_code sc;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);
  
  AQUIRE_LOCK(bank_lock[bank]);
  
  /* If the desired actuation mode is already set, silently exits. 
   * The NO_PULL_RESISTOR is a special case, as some platforms have
   * pull-up resistors enabled on startup, so this state may have to
   * be reinforced in the hardware. */
  if ( gpio_pin_state[bank][pin].resistor_mode == mode  && mode != NO_PULL_RESISTOR ) {
    RELEASE_LOCK(bank_lock[bank]);
    
    return RTEMS_SUCCESSFUL;
  }

  sc = rtems_bsp_gpio_set_resistor_mode(bank, pin, mode);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(bank_lock[bank]);
    
    return sc;
  }

  gpio_pin_state[bank][pin].resistor_mode = mode;

  RELEASE_LOCK(bank_lock[bank]);
  
  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_release_pin(uint32_t pin_number)
{
  rtems_status_code sc;
  gpio_pin* gpio;
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  AQUIRE_LOCK(bank_lock[bank]);

  gpio = &gpio_pin_state[bank][pin];
 
  /* If the pin has an enabled interrupt then remove the handler(s),
   * and disable the interrupts on that pin. */
  if ( gpio->enabled_interrupt != NONE ) {
    sc = rtems_gpio_disable_interrupt(pin_number);

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(bank_lock[bank]);
      
      return sc;
    }
  }

  gpio->pin_function = NOT_USED;
  gpio->task_id = RTEMS_ID_NONE;
  gpio->debouncing_tick_count = 0;
  gpio->last_isr_tick = 0;

  RELEASE_LOCK(bank_lock[bank]);

  return RTEMS_SUCCESSFUL;
}

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

  vector = rtems_bsp_gpio_get_vector(bank_number);

  /* If the current bank is an odd last bank (i.e.: not completely filled). */
  if ( odd_bank_pins > 0 &&  bank_number == bank_count - 1 ) {
    bank_pin_count = odd_bank_pins;
  }
  else {
    bank_pin_count = pins_per_bank;
  }

  /* Prevents more interrupts from being generated on GPIO. */
  bsp_interrupt_vector_disable(vector);

  /* Ensure that interrupts are disabled in this vector, before checking
   * the interrupt line. */
  RTEMS_COMPILER_MEMORY_BARRIER();

  /* Obtains a 32-bit bitmask, with the pins currently reporting interrupts
   * signaled with 1. */
  event_status = rtems_bsp_gpio_interrupt_line(vector);

  /* Iterates through the bitmask and calls the corresponding handler
   * for active interrupts. */
  for ( i = 0; i < bank_pin_count; ++i ) {
    /* If active, wake the corresponding pin's ISR task. */
    if ( event_status & (1 << i) ) {
      rtems_event_send(gpio[i].task_id, RTEMS_EVENT_1);
    }
  }

  /* Clear all active events. */
  rtems_bsp_gpio_clear_interrupt_line(vector, event_status);

  /* Ensure that the interrupt line is cleared before re-activating
   * the interrupts on this vector. */
  RTEMS_COMPILER_MEMORY_BARRIER();
  
  bsp_interrupt_vector_enable(vector);
}

rtems_status_code rtems_gpio_debounce_switch(uint32_t pin_number, int ticks)
{
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);
  
  AQUIRE_LOCK(bank_lock[bank]);
  
  if ( gpio_pin_state[bank][pin].pin_function != DIGITAL_INPUT ) {
    RELEASE_LOCK(bank_lock[bank]);
    
    return RTEMS_NOT_CONFIGURED;
  }

  gpio_pin_state[bank][pin].debouncing_tick_count = ticks;
  gpio_pin_state[bank][pin].last_isr_tick = rtems_clock_get_ticks_per_second();

  RELEASE_LOCK(bank_lock[bank]);
  
  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_interrupt_handler_install(
uint32_t pin_number,
rtems_gpio_irq_state (*handler) (void *arg),
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

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  AQUIRE_LOCK(bank_lock[bank]);

  gpio = &gpio_pin_state[bank][pin];

  /* If the current pin has no interrupt enabled 
   * then it does not need an handler. */
  if ( gpio->enabled_interrupt == NONE ) {
    RELEASE_LOCK(bank_lock[bank]);
    
    return RTEMS_NOT_CONFIGURED;
  }
  /* If the pin already has an enabled interrupt but the installed handler
   * is set as unique. */
  else if ( gpio->handler_flag == UNIQUE_HANDLER && gpio->handler_list != NULL ) {
    RELEASE_LOCK(bank_lock[bank]);
    
    return RTEMS_RESOURCE_IN_USE;
  }

  /* Update the pin's ISR list. */
  isr_node = (gpio_handler_list *) malloc(sizeof(gpio_handler_list));

  if ( isr_node == NULL ) {
    RELEASE_LOCK(bank_lock[bank]);
    
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

  RELEASE_LOCK(bank_lock[bank]);
  
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
  gpio_pin* gpio;
  uint32_t bank;
  uint32_t pin;

  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  vector = rtems_bsp_gpio_get_vector(bank);

  AQUIRE_LOCK(bank_lock[bank]);
  
  gpio = &gpio_pin_state[bank][pin];

  if ( gpio->pin_function != DIGITAL_INPUT ) {
    RELEASE_LOCK(bank_lock[bank]);

    return RTEMS_NOT_CONFIGURED;
  }

  /* If trying to enable the same type of interrupt on the same pin, or if the pin
   * already has an enabled interrupt. */
  if ( interrupt == gpio->enabled_interrupt || gpio->enabled_interrupt != NONE ) {
    RELEASE_LOCK(bank_lock[bank]);

    return RTEMS_RESOURCE_IN_USE;
  }
  
  /* Creates and starts a new task which will call the corresponding 
   * user-defined handlers for this pin. */
  sc = rtems_task_create(rtems_build_name('G', 'P', 'I', 'O'),
                         5,
                         RTEMS_MINIMUM_STACK_SIZE * 2,
                         RTEMS_NO_TIMESLICE,
                         RTEMS_DEFAULT_ATTRIBUTES,
                         &gpio->task_id);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(bank_lock[bank]);

    return RTEMS_UNSATISFIED;
  }
  
  sc = rtems_task_start(gpio->task_id, generic_handler_task, (rtems_task_argument) gpio);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(bank_lock[bank]);
      
    sc = rtems_task_delete(gpio->task_id);

    assert( sc == RTEMS_SUCCESSFUL );
      
    return RTEMS_UNSATISFIED;
  } 

  gpio->enabled_interrupt = interrupt;
  gpio->handler_flag = flag;

  /* Installs the interrupt handler. */
  sc = rtems_gpio_interrupt_handler_install(pin_number, handler, arg);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(bank_lock[bank]);

    sc = rtems_task_delete(gpio->task_id);

    assert( sc == RTEMS_SUCCESSFUL );
      
    return RTEMS_UNSATISFIED;
  }

  /* If the generic ISR has not been yet installed for this bank, installs it.
   * This ISR will be responsible for calling the handler tasks,
   * which in turn will call the user-defined interrupt handlers.*/
  if ( interrupt_counter[bank] == 0 ) {
    sc = rtems_interrupt_handler_install(vector, 
                                         "GPIO_HANDLER", 
                                         RTEMS_INTERRUPT_UNIQUE, 
                                         (rtems_interrupt_handler) generic_isr,
                                         gpio_pin_state[bank]);

    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(bank_lock[bank]);

      return RTEMS_UNSATISFIED;
    }
  }
  
  sc = rtems_bsp_enable_interrupt(bank, pin, interrupt);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(bank_lock[bank]);

    return RTEMS_UNSATISFIED;
  }
  
  ++interrupt_counter[bank];

  RELEASE_LOCK(bank_lock[bank]);

  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_interrupt_handler_remove(
uint32_t pin_number,
rtems_gpio_irq_state (*handler) (void *arg),
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

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  AQUIRE_LOCK(bank_lock[bank]);
  
  gpio = &gpio_pin_state[bank][pin];

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
    RELEASE_LOCK(bank_lock[bank]);
    
    return rtems_gpio_disable_interrupt(pin_number);
  }

  RELEASE_LOCK(bank_lock[bank]);
  
  return RTEMS_SUCCESSFUL;
}

rtems_status_code rtems_gpio_disable_interrupt(uint32_t pin_number)
{
  gpio_handler_list *isr_node;
  rtems_vector_number vector;
  rtems_status_code sc;
  gpio_pin* gpio;
  uint32_t bank;
  uint32_t pin;
  
  if ( pin_number < 0 || pin_number >= gpio_count ) {
    return RTEMS_INVALID_ID;
  }

  bank = BANK_NUMBER(pin_number);
  pin = PIN_NUMBER(pin_number);

  vector = rtems_bsp_gpio_get_vector(bank);

  AQUIRE_LOCK(bank_lock[bank]);
  
  gpio = &gpio_pin_state[bank][pin];
  
  if ( interrupt_counter[bank] == 0 || gpio->enabled_interrupt == NONE ) {
    RELEASE_LOCK(bank_lock[bank]);
    
    return RTEMS_SUCCESSFUL;
  }

  sc = rtems_bsp_disable_interrupt(bank, pin, gpio->enabled_interrupt);

  if ( sc != RTEMS_SUCCESSFUL ) {
    RELEASE_LOCK(bank_lock[bank]);

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
    sc = rtems_interrupt_handler_remove(vector, 
                                        (rtems_interrupt_handler) generic_isr, 
                                        gpio_pin_state[bank]);
    
    if ( sc != RTEMS_SUCCESSFUL ) {
      RELEASE_LOCK(bank_lock[bank]);

      return RTEMS_UNSATISFIED;
    }
  }

  RELEASE_LOCK(bank_lock[bank]);

  return RTEMS_SUCCESSFUL;
}
