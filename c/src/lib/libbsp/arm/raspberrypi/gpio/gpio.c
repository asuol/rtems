/**
 * @file gpio.c
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Support for the Raspberry PI GPIO.
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

/* Calculates a bitmask to assign an alternate function to a given pin. */
#define SELECT_PIN_FUNCTION(fn, pn) (fn << ((pn % 10) * 3))

#define API_LOCK(m) assert(rtems_semaphore_obtain(m,                    \
                                                  RTEMS_WAIT,           \
                                                  RTEMS_NO_TIMEOUT      \
                                                  ) == RTEMS_SUCCESSFUL)

#define API_UNLOCK(m) rtems_semaphore_release(m)

static bool is_initialized = false;

static int interrupt_counter = 0;

static rpi_gpio_pin gpio_pin[GPIO_COUNT];

static rtems_id int_id = RTEMS_ID_NONE;

/**
 * @brief De-bounces a switch by requiring a certain time to pass between 
 *        interrupts. Any interrupt fired too close to the last will be 
 *        ignored as it is probably the result of an involuntary switch/button
 *        bounce after being released.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval 0 Interrupt is likely provoked by a user press on the switch.
 * @retval -1 Interrupt was generated too close to the last one. 
 *            Probably a switch bounce.
 */
static int debounce_switch(int dev_pin)
{
  rtems_interval time;
  rpi_gpio_pin *pin;

  assert( dev_pin >= 1 && dev_pin <= GPIO_COUNT );
  
  pin = &gpio_pin[dev_pin];
  
  time = rtems_clock_get_ticks_since_boot();
  
  if ( (time - pin->last_isr_tick) < pin->debouncing_tick_count ) {
    return -1;
  }

  pin->last_isr_tick = time;
  
  return 0;
}

static rtems_task generic_handler_task(rtems_task_argument arg)
{
  gpio_handler_list *handler_list;
  rtems_event_set out;
  int handled_count;
  int pin;
  int rv;

  pin = (int) arg;

  /* If this pin has the debouncing function attached, call it. */
  if ( gpio_pin[pin].debouncing_tick_count > 0 ) {
    rv = debounce_switch(pin);

    /* If the handler call was caused by a switch bounce, simply returns. */
    if ( rv < 0 ) {
      return;
    }

    /* Record the current clock tick. */
    gpio_pin[pin].last_isr_tick = rtems_clock_get_ticks_since_boot();
  }
  
  while ( true ) {
    handled_count = 0;

    rtems_event_receive(RTEMS_EVENT_1, RTEMS_EVENT_ALL | RTEMS_WAIT,
                        RTEMS_NO_TIMEOUT,
                        &out);
    
    handler_list = gpio_pin[pin].handler_list;

    while ( handler_list != NULL ) {
      /* Call the user's ISR. */
      if ( (handler_list->handler)(handler_list->arg) == IRQ_HANDLED ) {
        ++handled_count;
      }

      handler_list = handler_list->next_isr;
    }

    /* If no handler assumed the interrupt, treat it as a spurious interrupt. */
    if ( handled_count == 0 ) {
      bsp_interrupt_handler_default(BCM2835_IRQ_ID_GPIO_0);
    }
  }
}

/**
 * @brief Waits a number of CPU cycles.
 *
 * @param[in] cycles The number of CPU cycles to wait.
 */
static void arm_delay (int cycles)
{
  int i;

  for ( i = 0; i < cycles; ++i ) {
    asm volatile ("nop");
  }
}

/**
 * @brief Initializes the GPIO API and sets every pin as NOT_USED.
 */
void gpio_initialize(void)
{
  rtems_status_code sc;
  rtems_interval tick;
  int i;

  if ( is_initialized ) {
    return;
  }

  is_initialized = true;

  sc = rtems_semaphore_create(rtems_build_name('G', 'I', 'N', 'T'), 1, MUTEX_ATRIBUTES, 0, &int_id);

  assert( sc == RTEMS_SUCCESSFUL );

  tick = rtems_clock_get_ticks_since_boot();

  for ( i = 0; i < GPIO_COUNT; ++i ) {
    gpio_pin[i].pin_type = NOT_USED;
    gpio_pin[i].enabled_interrupt = NONE;
    gpio_pin[i].task_id = RTEMS_ID_NONE;
    gpio_pin[i].handler_list = NULL;
    gpio_pin[i].debouncing_tick_count = 0;
    gpio_pin[i].last_isr_tick = tick;
  }
}

/**
 * @brief Gives an output GPIO pin the logical value of 1.
 *
 * @param[in] pin The Raspberry Pi GPIO pin label number (not is position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL Pin was set successfully.
 * @retval RTEMS_NOT_CONFIGURED The received pin is not configured 
 *                              as an digital output.
 */
rtems_status_code gpio_set(int pin)
{
  assert( pin >= 0 && pin < GPIO_COUNT );

  if ( gpio_pin[pin].pin_type != DIGITAL_OUTPUT ) {
    return RTEMS_NOT_CONFIGURED;
  }

  BCM2835_REG(BCM2835_GPIO_GPSET0) = (1 << pin);

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Gives an output GPIO pin the logical value of 0.
 *
 * @param[in] pin The Raspberry Pi GPIO pin label number (not is position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL Pin was cleared successfully.
 * @retval RTEMS_NOT_CONFIGURED The received pin is not configured 
 *                              as an digital output.
 */
rtems_status_code gpio_clear(int pin)
{
  assert( pin >= 0 && pin < GPIO_COUNT );

  if ( gpio_pin[pin].pin_type != DIGITAL_OUTPUT ) {
    return RTEMS_NOT_CONFIGURED;
  }

  BCM2835_REG(BCM2835_GPIO_GPCLR0) = (1 << pin);

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Gets the value (level) of a GPIO input pin.
 *
 * @param[in] pin The Raspberry Pi GPIO pin label number (not is position 
 *            on the header). 
 *
 * @retval The function returns 0 or 1 depending on the pin current 
 *         logical value.
 */
int gpio_get_value(int pin)
{
  assert( pin >= 0 && pin < GPIO_COUNT );

  return (BCM2835_REG(BCM2835_GPIO_GPLEV0) & (1 << pin));
}

/**
 * @brief Configures a GPIO pin to perform a certain function.
 *
 * @param[in] pin The Raspberry Pi GPIO pin label number (not is position 
 *            on the header). 
 * @param[in] type The new function of the pin.
 *
 * @retval RTEMS_SUCCESSFUL Pin was configured successfully.
 * @retval RTEMS_RESOURCE_IN_USE The received pin is already being used.
 */
rtems_status_code gpio_select_pin(int pin, rpi_pin type)
{
  assert( pin >= 0 && pin < GPIO_COUNT );

  /* Calculate the pin function select register address. */
  volatile unsigned int *pin_addr = (unsigned int *)BCM2835_GPIO_REGS_BASE +
                                    (pin / 10);
  
  /* If the pin is already being used returns with an error. */
  if ( gpio_pin[pin].pin_type != NOT_USED ) {
    return RTEMS_RESOURCE_IN_USE;
  }

  /* Sets pin function select bits.*/
  if ( type == DIGITAL_INPUT ) {
    *(pin_addr) &= ~SELECT_PIN_FUNCTION(7, pin);
  }
  else {
    *(pin_addr) |= SELECT_PIN_FUNCTION(type, pin);
  }

  /* If the alternate function was successfuly assigned to the pin,
   * record that information on the gpio_pin structure. */
  gpio_pin[pin].pin_type = type;

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Configures the pull resistor setting of an array of GPIO pins. 
 *
 * @param[in] pins Array of Raspberry Pi GPIO pin label numbers (not their 
 *            position on the header). 
 * @param[in] pin_count Number of pins on the @var pins array.
 * @param[in] mode The pull resistor mode.
 *
 * @retval RTEMS_SUCCESSFUL Pull resistor successfully configured.
 * @retval RTEMS_NOT_DEFINED Unknown pull resistor mode.
 */
static rtems_status_code 
set_input_mode(int *pins, int pin_count, int pin_mask, rpi_gpio_input_mode mode)
{
  int i;

  /* Set control signal. */
  switch ( mode ) {
    case PULL_UP:
      BCM2835_REG(BCM2835_GPIO_GPPUD) = (1 << 1);
      break;

    case PULL_DOWN:
      BCM2835_REG(BCM2835_GPIO_GPPUD) = (1 << 0);
      break;

    case NO_PULL_RESISTOR:
      BCM2835_REG(BCM2835_GPIO_GPPUD) = 0;
      break;

    default:
      return RTEMS_NOT_DEFINED;
  }

  /* Wait 150 cyles, as per BCM2835 documentation. */
  arm_delay(150);

  /* Setup clock for the control signal. */
  BCM2835_REG(BCM2835_GPIO_GPPUDCLK0) = pin_mask;

  arm_delay(150);

  /* Remove the control signal. */
  BCM2835_REG(BCM2835_GPIO_GPPUD) = 0;

  /* Remove the clock. */
  BCM2835_REG(BCM2835_GPIO_GPPUDCLK0) = 0;

  /* If the operation was successful, record that information
   * on the gpio_pin structure so it can be recalled later. */
  for ( i = 0; i < pin_count; ++i ) {
    gpio_pin[pins[i]].input_mode = mode;
  }

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Configures a single GPIO pin pull resistor. 
 *
 * @param[in] pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 * @param[in] mode The pull resistor mode.
 *
 * @retval RTEMS_SUCCESSFUL Pull resistor successfully configured.
 * @retval RTEMS_NOT_DEFINED @see set_input_mode().
 */
rtems_status_code gpio_input_mode(int pin, rpi_gpio_input_mode mode)
{
  assert( pin >= 0 && pin < GPIO_COUNT );

  int pin_mask = (1 << pin);
  int pins[1];

  /* If the desired actuation mode is already set, silently exits. */
  if ( gpio_pin[pin].input_mode == mode ) {
    return RTEMS_SUCCESSFUL;
  }

  pins[0] = pin;

  return set_input_mode(pins, 1, pin_mask, mode);
}

/**
 * @brief Sets the same pull-up/down resistors actuation mode to multiple GPIO 
 *        input pins. There is a maximum number of 32 pins per call, 
 *        which is enough for Raspberry Pi models A and B (17 GPIOs on
 *        P1 GPIO header) and also model B+ (28 GPIOs on J8 GPIO header). 
 *
 * @param[in] pins Array of Raspberry Pi GPIO pin label numbers 
 *                 (not their position on the header). 
 * @param[in] pin_count Number of pins on the @var pins array.
 * @param[in] mode The pull resistor mode.
 *
 * @retval RTEMS_SUCCESSFUL Pull resistor successfully configured.
 * @retval RTEMS_INVALID_ID Unknown pull resistor mode.
 */
rtems_status_code // TODO: pin_count may be replaced with sizeof
gpio_setup_input_mode(int *pins, int pin_count, rpi_gpio_input_mode mode)
{
  uint32_t pin_mask = 0;
  int diff_mode_counter = 0;
  int i;

  if ( pin_count > GPIO_PHYSICAL_PIN_COUNT ) {
    return RTEMS_INVALID_ID;
  }

  /* Cycle through the given pins to check if this operation will have an effect
   * on the resistor actuation mode of any one of the pins.
   * Every pin that currently uses a different pull resistor mode sets a bit
   * in its corresponding place on a bitmask. If the mode for a pin will not 
   * change then the diff_mode_counter variable is increased. */
  for ( i = 0; i < pin_count; ++i ) {
    assert( pins[i] >= 0 && pins[i] < GPIO_COUNT );

    if ( gpio_pin[pins[i]].input_mode != mode ) {
      pin_mask |= (1 << pins[i]);
    }
    else {
      ++diff_mode_counter;
    }
  }

  /* If no pin will have its resistor mode changed silently exits, avoiding an
   * unnecessary access to the Rasberry Pi memory registers. */
  if ( diff_mode_counter == 0 ) {
    return RTEMS_SUCCESSFUL;
  }

  return set_input_mode(pins, pin_count, pin_mask, mode);
}

/**
 * @brief Disables a GPIO pin on the API, making it available to be used 
 *        by anyone on the system.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL Pin successfully disabled on the API.
 * @retval RTEMS_UNSATISFIED Could not disable an ative interrupt on this pin, 
 *                           @see gpio_disable_interrupt(),
 */
rtems_status_code gpio_disable_pin(int dev_pin)
{
  rtems_status_code sc = RTEMS_SUCCESSFUL;
  rpi_gpio_pin *pin;

  assert( dev_pin >= 1 && dev_pin <= GPIO_COUNT );

  pin = &gpio_pin[dev_pin];

  pin->pin_type = NOT_USED;
 
  /* If the pin has an enabled interrupt then remove the handler. */
  if ( pin->enabled_interrupt != NONE ) {
    sc = gpio_disable_interrupt(dev_pin);
  }
    
  return sc;
}

/**
 * @brief Setups a JTAG interface using the P1 GPIO pin header
 *        for the models A/B and J8 header on the B+. 
 *        The following pins should be unused before calling this function:
 *        GPIO 4, 22, 24, 25 and 27.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL JTAG interface successfully configured.
 * @retval RTEMS_RESOURCE_IN_USE At least one of the required pins is currently
 *                               occupied, @see gpio_select_pin().
 */
rtems_status_code gpio_select_jtag(void)
{
  rtems_status_code sc;

  int jtag_pins[] = {4, 22, 24, 25, 27};
  
  if ( (sc = gpio_setup_input_mode(jtag_pins, 5, NO_PULL_RESISTOR)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }
  
  /* Setup gpio 4 alt5 ARM_TDI. */
  if ( (sc = gpio_select_pin(4, ALT_FUNC_5)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* Setup gpio 22 alt4 ARM_TRST. */
  if ( (sc = gpio_select_pin(22, ALT_FUNC_4)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* Setup gpio 24 alt4 ARM_TDO. */
  if ( (sc = gpio_select_pin(24, ALT_FUNC_4)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* Setup gpio 25 alt4 ARM_TCK. */
  if ( (sc = gpio_select_pin(25, ALT_FUNC_4)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }
  
  /* Setup gpio 27 alt4 ARM_TMS. */
  if ( (sc = gpio_select_pin(27, ALT_FUNC_4)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }
    
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Setups a SPI interface using the P1 GPIO pin header
 *        for the models A/B and J8 header on the B+. 
 *        The following pins should be unused before calling this function:
 *        GPIO 7, 8, 9, 10 and 11.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL SPI interface successfully configured.
 * @retval RTEMS_RESOURCE_IN_USE At least one of the required pins is currently
 *                               occupied, @see gpio_select_pin().
 */
rtems_status_code gpio_select_spi_p1(void)
{
  rtems_status_code sc;

  /* SPI master 0 MISO data line. */
  if ( (sc = gpio_select_pin(9, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* SPI master 0 MOSI data line. */
  if ( (sc = gpio_select_pin(10, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* SPI master 0 SCLK clock line. */
  if ( (sc = gpio_select_pin(11, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* SPI master 0 CE_0 chip enable line. */
  if ( (sc = gpio_select_pin(8, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* SPI master 0 CE_1 chip enable line. */
  if ( (sc = gpio_select_pin(7, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }
    
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Setups a I2C interface using the P1 GPIO pin header
 *        for the models A/B and J8 header on the B+. 
 *        The following pins should be unused before calling this function:
 *        GPIO 2 and 3.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL JTAG interface successfully configured.
 * @retval RTEMS_RESOURCE_IN_USE At least one of the required pins is currently
 *                               occupied, @see gpio_select_pin().
 */
rtems_status_code gpio_select_i2c_p1_rev2(void)
{
  rtems_status_code sc;
  int pins[] = {2,3};

  /* I2C BSC1 SDA data line. */
  if ( (sc = gpio_select_pin(2, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* I2C BSC1 SCL clock line. */
  if ( (sc = gpio_select_pin(3, ALT_FUNC_0)) != RTEMS_SUCCESSFUL ) {
      return sc;
  }

  /* Enable pins 2 and 3 pull-up resistors. */
  if ( (sc = gpio_setup_input_mode(pins, 2, PULL_UP)) != RTEMS_SUCCESSFUL ) {
    return sc;
  }
    
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
  uint32_t event_status;
  rpi_gpio_pin *gpio;
  int i;

  gpio = (rpi_gpio_pin *) arg;
  
  /* Prevents more interrupts from being generated on GPIO. */
  interrupt_vector_disable();

  /* TODO: Insert Memory Barrier */

  /* Reads the GPIO Event register. */
  event_status = BCM2835_REG(BCM2835_GPIO_GPEDS0);

  /* Iterates through the register and calls the corresponding handler
   * for active events. */
  for ( i = 0; i < 32; ++i ) {
    if ( event_status & (1 << i) ) {
      rtems_event_send(gpio[i].task_id, RTEMS_EVENT_1);
    }
  }

  /* Clear all active events. */
  BCM2835_REG(BCM2835_GPIO_GPEDS0) = event_status;
  
  /* TODO: Insert Memory Barrier to prevent the interrupts being enabled before updating the event register. */

  interrupt_vector_enable();
}

/**
 * @brief Defines for a GPIO input pin the number of clock ticks that must pass
 *        before an generated interrupt is guaranteed to be generated by the user
 *        and not by a bouncing switch/button.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *                    on the header). 
 * @param[in] ticks Minimum number of clock ticks that must pass between 
 *                  interrupts so it can be considered a legitimate
 *                  interrupt. 
 *
 * @retval RTEMS_SUCCESSFUL De-bounce function successfully attached to the pin.
 * @retval RTEMS_NOT_CONFIGURED The current pin is not configured as a digital 
 *                              input, hence it can not be connected to a switch.
 */
rtems_status_code gpio_debounce_switch(int dev_pin, int ticks)
{
  assert( dev_pin >= 1 && dev_pin <= GPIO_COUNT );

  if ( gpio_pin[dev_pin].pin_type != DIGITAL_INPUT ) {
    return RTEMS_NOT_CONFIGURED;
  }

  gpio_pin[dev_pin].debouncing_tick_count = ticks;
  gpio_pin[dev_pin].last_isr_tick = rtems_clock_get_ticks_per_second();

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Connects a new user-defined interrupt handler to a given pin.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *                    on the header). 
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
 * @retval RTEMS_RESOURCE_IN_USE The current user-defined handler for this pin
 *                               is unique.
 */
rtems_status_code gpio_interrupt_handler_install(
int dev_pin,
gpio_irq_state (*handler) (void *arg),
void *arg
)
{
  gpio_handler_list *isr_node;
  rpi_gpio_pin *pin;

  assert( dev_pin >= 1 && dev_pin <= GPIO_PHYSICAL_PIN_COUNT );

  pin = &gpio_pin[dev_pin];

  if ( pin->enabled_interrupt == NONE ) {
    return RTEMS_NOT_CONFIGURED;
  }
  /* If the pin already has an enabled interrupt but the installed handler
   * is set as unique. */
  else if ( pin->handler_flag == UNIQUE_HANDLER && pin->handler_list != NULL ) {
      return RTEMS_RESOURCE_IN_USE;
  }
  
  isr_node = (gpio_handler_list *) malloc(sizeof(gpio_handler_list));

  if ( isr_node == NULL ) {
    return RTEMS_NO_MEMORY;
  }
  
  isr_node->handler = handler;
  isr_node->arg = arg;

  if ( pin->handler_flag == SHARED_HANDLER ) {
    isr_node->next_isr = pin->handler_list;
  }
  else {
    isr_node->next_isr = NULL;
  }

  pin->handler_list = isr_node;

  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Enables interrupts to be generated on a given GPIO pin.
 *        When fired that interrupt will call the given handler.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *                    on the header). 
 * @param[in] interrupt Type of interrupt to enable for the pin.
 * @param[in] handler Pointer to a function that will be called every time 
 *                    @var interrupt is generated. This function must return
 *                    information about its handled/unhandled state.
 * @param[in] arg Void pointer to the arguments of the user-defined handler.
 *
 * @retval RTEMS_SUCCESSFUL Interrupt successfully enabled for this pin.
 * @retval RTEMS_UNSATISFIED Could not install the GPIO ISR or create/start
 *                           the handler task.
 * @retval RTEMS_NOT_CONFIGURED The pin is not configured as a digital input,
 *                              hence interrupts are not taken into account.
 * @retval RTEMS_RESOURCE_IN_USE The current user-defined handler for this pin
 *                               is unique.
 */
rtems_status_code gpio_enable_interrupt(
int dev_pin, 
gpio_interrupt interrupt,
gpio_handler_flag flag,
gpio_irq_state (*handler) (void *arg),
void *arg
)
{
  rtems_status_code sc;
  rpi_gpio_pin *pin;

  assert( dev_pin >= 1 && dev_pin <= GPIO_PHYSICAL_PIN_COUNT && interrupt != NONE );

  pin = &gpio_pin[dev_pin];

  if ( pin->pin_type != DIGITAL_INPUT ) {
    return RTEMS_NOT_CONFIGURED;
  }

  API_LOCK(int_id);
  
  /* If the pin has no currently enabled interrupt, creates and starts
   * a new task which will call the corresponding user-defined handlers. */
  if ( pin->enabled_interrupt == NONE ) {
    sc = rtems_task_create(rtems_build_name('G', 'P', 'I', 'O'),
                           5,
                           RTEMS_MINIMUM_STACK_SIZE * 2,
                           RTEMS_NO_TIMESLICE,
                           RTEMS_DEFAULT_ATTRIBUTES,
                           &pin->task_id);

    if ( sc != RTEMS_SUCCESSFUL ) {
      API_UNLOCK(int_id);
      
      return RTEMS_UNSATISFIED;
    }

    sc = rtems_task_start(pin->task_id, generic_handler_task, (rtems_task_argument) dev_pin);

    if ( sc != RTEMS_SUCCESSFUL ) {
      API_UNLOCK(int_id);
      
      sc = rtems_task_delete(pin->task_id);

      assert( sc == RTEMS_SUCCESSFUL );
      
      return RTEMS_UNSATISFIED;
    } 

    pin->enabled_interrupt = interrupt;
    pin->handler_flag = flag;
  }

  /* Installs the interrupt handler and reactivates the GPIO interrupt vector. */ //TODO: update this comment?
  sc = gpio_interrupt_handler_install(dev_pin, handler, arg);

  assert( sc == RTEMS_SUCCESSFUL );
  
  interrupt_vector_disable(); //TODO: move this up?

  /* If the generic ISR has not been yet installed, installs it.
   * This ISR will be responsible for calling the handler tasks,
   * which in turn will call the user-defined interrupt handlers.*/
  if ( interrupt_counter == 0 ) {
    sc = rtems_interrupt_handler_install(BCM2835_IRQ_ID_GPIO_0, 
                                         "RPI_GPIO", 
                                         RTEMS_INTERRUPT_UNIQUE, 
                                         (rtems_interrupt_handler) generic_isr,
                                         gpio_pin);

    if ( sc != RTEMS_SUCCESSFUL ) {
      API_UNLOCK(int_id);
      
      return RTEMS_UNSATISFIED;
    }
  }
  
  switch ( interrupt ) {
    case FALLING_EDGE:
      /* Enables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) |= (1 << dev_pin);
      break;

    case RISING_EDGE:
      /* Enables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) |= (1 << dev_pin);
      break;

    case BOTH_EDGES:
      /* Enables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) |= (1 << dev_pin);

      /* Enables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) |= (1 << dev_pin);
      break;

    case LOW_LEVEL:
      /* Enables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) |= (1 << dev_pin);
      break;

    case HIGH_LEVEL:
      /* Enables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) |= (1 << dev_pin);
      break;

    case BOTH_LEVELS:
      /* Enables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) |= (1 << dev_pin);

      /* Enables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) |= (1 << dev_pin);
      break;
      
    case NONE:
      break;
  }
  
  ++interrupt_counter;

  API_UNLOCK(int_id);
  
  interrupt_vector_enable();
  
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Disconnects a new user-defined interrupt handler from the given pin.
 *        If in the end there are no more user-defined handler connected
 *        to the pin interrupts are disabled on the given pin.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 * @param[in] handler Pointer to the user-defined handler
 * @param[in] arg Void pointer to the arguments of the user-defined handler.
 *
 * @retval RTEMS_SUCCESSFUL Handler successfully disconnected from this pin.
 * @retval * @see gpio_disable_interrupt()
 */
rtems_status_code gpio_interrupt_handler_remove(
int dev_pin,
gpio_irq_state (*handler) (void *arg),
void *arg
)
{
  gpio_handler_list *isr_node, *next_node;
  rpi_gpio_pin *pin;

  assert( dev_pin >= 1 && dev_pin <= GPIO_PHYSICAL_PIN_COUNT );
  
  pin = &gpio_pin[dev_pin];

  API_LOCK(int_id);

  isr_node = pin->handler_list;

  if ( isr_node != NULL ) {
    if ( isr_node->handler == handler && isr_node->arg == arg ) {
      pin->handler_list = isr_node->next_isr;

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
  if ( pin->handler_list == NULL ) {
    API_UNLOCK(int_id);
    
    return gpio_disable_interrupt(dev_pin);
  }

  API_UNLOCK(int_id);
  
  return RTEMS_SUCCESSFUL;
}

/**
 * @brief Stops interrupts from being generated from a given GPIO pin
 *        and removes the corresponding handler.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval RTEMS_SUCCESSFUL Interrupt successfully disabled for this pin.
 * @retval RTEMS_UNSATISFIED Could not remove the current interrupt handler or 
 *                           could not recognise the current active interrupt 
 *                           on this pin.
 */
rtems_status_code gpio_disable_interrupt(int dev_pin)
{
  gpio_handler_list *isr_node;
  rtems_status_code sc;
  rpi_gpio_pin *pin;

  assert( dev_pin >= 1 && dev_pin <= GPIO_PHYSICAL_PIN_COUNT );
  
  if ( interrupt_counter == 0 ) {
    return RTEMS_SUCCESSFUL;
  }

  pin = &gpio_pin[dev_pin];

  interrupt_vector_disable();

  API_LOCK(int_id);

  if ( pin->enabled_interrupt == NONE ) {
    API_UNLOCK(int_id);

    interrupt_vector_enable();
      
    return RTEMS_SUCCESSFUL; 
  }

  else {
    bsp_disable_interrupt(dev_pin, pin->enabled_interrupt);
  }
  
  pin->enabled_interrupt = NONE;

  while ( pin->handler_list != NULL ) {
    isr_node = pin->handler_list;

    pin->handler_list = isr_node->next_isr;

    free(isr_node);
  }

  sc = rtems_task_delete(pin->task_id);

  assert( sc == RTEMS_SUCCESSFUL );
  
  --interrupt_counter;

  /* If no GPIO interrupts are left, removes the handler. */
  if ( interrupt_counter == 0 ) {
    sc = rtems_interrupt_handler_remove(BCM2835_IRQ_ID_GPIO_0, 
                                        (rtems_interrupt_handler) generic_isr, 
                                        gpio_pin);
    
    if ( sc != RTEMS_SUCCESSFUL ) {
      API_UNLOCK(int_id);

      interrupt_vector_enable();
      
      return RTEMS_UNSATISFIED;
    }
  }

  API_UNLOCK(int_id);
  
  interrupt_vector_enable();
  
  return RTEMS_SUCCESSFUL;
}

/*
 *
 * Raspberry PI specific functions, which any BSP would need to implement.
 *
 */
void interrupt_vector_disable(void)
{
  bsp_interrupt_vector_disable(BCM2835_IRQ_ID_GPIO_0);
}

void interrupt_vector_enable(void)
{
  bsp_interrupt_vector_enable(BCM2835_IRQ_ID_GPIO_0);
}

void bsp_disable_interrupt(int dev_pin, gpio_interrupt enabled_interrupt)
{
  switch ( enabled_interrupt ) {
    case FALLING_EDGE:
      /* Disables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) &= ~(1 << dev_pin);
      break;

    case RISING_EDGE:
      /* Disables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) &= ~(1 << dev_pin);
      break;

    case BOTH_EDGES:
      /* Disables asynchronous falling edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) &= ~(1 << dev_pin);

      /* Disables asynchronous rising edge detection. */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) &= ~(1 << dev_pin);
      break;

    case LOW_LEVEL:
      /* Disables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) &= ~(1 << dev_pin);
      break;

    case HIGH_LEVEL:
      /* Disables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) &= ~(1 << dev_pin);
      break;

    case BOTH_LEVELS:
      /* Disables pin low level detection. */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) &= ~(1 << dev_pin);

      /* Disables pin high level detection. */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) &= ~(1 << dev_pin);
      break;
  }
}


