/**
 * @file gpio.c
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Support for the Raspberry PI GPIO.
 *
 */

/*
 *  COPYRIGHT (c) 2014 Andre Marques <andre.lousa.marques at gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp/raspberrypi.h>
#include <bsp/irq.h>
#include <bsp/gpio.h>

#include <stdlib.h>

/* Calculates a bitmask to assign an alternate function to a given pin. */
#define SELECT_PIN_FUNCTION(fn, pn) (fn << ((pn % 10) * 3))

static bool is_initialized = false;

rpi_gpio_pin *gpio_pin;

/**
 * @brief Waits a number of CPU cycles.
 *
 * @param[in] cycles The number of CPU cycles to wait.
 *
 */
static void arm_delay (int cycles)
{
  int i;

  for (i = 0; i < cycles; i++)
    asm volatile ("nop");
}

/**
 * @brief Initializes the GPIO API. 
 *        Allocates space to the gpio_pin array and sets every pin as NOT_USED.
 *        If the API has already been initialized silently exits.
 */
void gpio_initialize(void)
{
  int i;

  if ( is_initialized )
    return;

  is_initialized = true;

  gpio_pin = (rpi_gpio_pin *) malloc(GPIO_PIN_COUNT * sizeof(rpi_gpio_pin));

  for ( i = 0; i < GPIO_PIN_COUNT; i++ ) {
    gpio_pin[i].pin_type = NOT_USED;
    gpio_pin[i].enabled_interrupt = NONE;

    gpio_pin[i].h_args.debouncing_tick_count = 0;
  }
}

/**
 * @brief Gives an output GPIO pin the logical value of 1.
 *
 * @param[in] pin The Raspberry Pi GPIO pin label number (not is position 
 *            on the header). 
 *
 * @retval 0 Pin was set successfully.
 * @retval -1 The received pin is not configured as an digital output.
 */
int gpio_set(int pin)
{
  if (gpio_pin[pin-1].pin_type != DIGITAL_OUTPUT)
    return -1;

  BCM2835_REG(BCM2835_GPIO_GPSET0) = (1 << pin);

  return 0;
}

/**
 * @brief Gives an output GPIO pin the logical value of 0.
 *
 * @param[in] pin The Raspberry Pi GPIO pin label number (not is position 
 *            on the header). 
 *
 * @retval 0 Pin was cleared successfully.
 * @retval -1 The received pin is not configured as an digital output.
 */
int gpio_clear(int pin)
{
  if (gpio_pin[pin-1].pin_type != DIGITAL_OUTPUT)
    return -1;

  BCM2835_REG(BCM2835_GPIO_GPCLR0) = (1 << pin);

  return 0;
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
int gpio_get_val(int pin)
{
  return BCM2835_REG(BCM2835_GPIO_GPLEV0) &= (1 << (pin));
}

/**
 * @brief Configures a GPIO pin to perform a certain function.
 *
 * @param[in] pin The Raspberry Pi GPIO pin label number (not is position 
 *            on the header). 
 * @param[in] type The new function of the pin.
 *
 * @retval 0 Pin was configured successfully.
 * @retval -1 The received pin is already being used, or unknown function.
 */
int gpio_select_pin(int pin, rpi_pin type)
{
  /* Calculate the pin function select register address. */
  volatile unsigned int *pin_addr = (unsigned int *)BCM2835_GPIO_REGS_BASE + (pin / 10);
  
  /* If the pin is already being used returns with an error. */
  if ( gpio_pin[pin-1].pin_type != NOT_USED )
    return -1;

  /* Sets pin function select bits as zero (DIGITAL_INPUT).*/
  *(pin_addr) &= ~SELECT_PIN_FUNCTION(7, pin);

  switch ( type ) {
    case DIGITAL_INPUT:

      /* Digital input is set by default before this switch. */

      break;

    case DIGITAL_OUTPUT:

      *(pin_addr) |= SELECT_PIN_FUNCTION(1, pin);

      break;

    case ALT_FUNC_0:

      *(pin_addr) |= SELECT_PIN_FUNCTION(4, pin);

      break;

    case ALT_FUNC_1:

      *(pin_addr) |= SELECT_PIN_FUNCTION(5, pin);

      break;

    case ALT_FUNC_2:

      *(pin_addr) |= SELECT_PIN_FUNCTION(6, pin);

      break;

    case ALT_FUNC_3:

      *(pin_addr) |= SELECT_PIN_FUNCTION(7, pin);

      break;

    case ALT_FUNC_4:

      *(pin_addr) |= SELECT_PIN_FUNCTION(3, pin);

      break;

    case ALT_FUNC_5:

      *(pin_addr) |= SELECT_PIN_FUNCTION(2, pin);

      break;

    default:
      return -1;
  }

  /* If the alternate function was successfuly assigned to the pin,
   * record that information on the gpio_pin structure. */
  gpio_pin[pin-1].pin_type = type;

  return 0;
}

/**
 * @brief Configures the pull resistor setting of an array of GPIO pins. 
 *
 * @param[in] pins Array of Raspberry Pi GPIO pin label numbers (not their position 
 *            on the header). 
 * @param[in] pin_count Number of pins on the @var pins array.
 * @param[in] mode The pull resistor mode.
 *
 * @retval 0 Pull resistor successfully configured.
 * @retval -1 Unknown pull resistor mode.
 */
static int 
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
      return -1;
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
  for ( i = 0; i < pin_count; i++ )
    gpio_pin[pins[i]-1].input_mode = mode;

  return 0;
}

/**
 * @brief Configures a single GPIO pin pull resistor. 
 *
 * @param[in] pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 * @param[in] mode The pull resistor mode.
 *
 * @retval 0 Pull resistor successfully configured.
 * @retval -1 @see set_input_mode().
 */
int gpio_input_mode(int pin, rpi_gpio_input_mode mode)
{
  int pin_mask = (1 << pin);
  int pins[1];

  /* If the desired actuation mode is already set, silently exits. */
  if ( gpio_pin[pin-1].input_mode == mode )
    return 0;

  pins[0] = pin;

  return set_input_mode(pins, 1, pin_mask, mode);
}

/**
 * @brief Sets the same pull-up/down resistors actuation mode to multiple GPIO input pins.
 *        There is a maximum number of 32 pins per call, which is enough for 
 *        Raspberry Pi models A and B (17 GPIOs on P1 GPIO header) 
 *        and also model B+ (28 GPIOs on J8 GPIO header). 
 *
 * @param[in] pins Array of Raspberry Pi GPIO pin label numbers (not their position 
 *            on the header). 
 * @param[in] pin_count Number of pins on the @var pins array.
 * @param[in] mode The pull resistor mode.
 *
 * @retval 0 Pull resistor successfully configured.
 * @retval -1 Unknown pull resistor mode.
 */
int gpio_setup_input_mode(int *pins, int pin_count, rpi_gpio_input_mode mode)
{
  uint32_t pin_mask = 0;
  int diff_mode_counter = 0;
  int i;

  if ( pin_count > 32 )
    return -1;

  /* Cycle through the given pins to check if this operation will have an effect
   * on the resistor actuation mode of any one of the pins.
   * Every pin that currently uses a different pull resistor mode sets a bit
   * in its corresponding place on a bitmask. If the mode for a pin will not change 
   * then the diff_mode_counter variable is increased. */
  for ( i = 0; i < pin_count; i++ ) {
    if ( gpio_pin[pins[i] - 1].input_mode != mode )
      pin_mask |= (1 << pins[i]);
    
    else
      diff_mode_counter++;
  }

  /* If no pin will have its resistor mode changed silently exits, avoiding an
   * unnecessary access to the Rasberry Pi memory registers. */
  if ( diff_mode_counter == 0 )
    return 0;

  return set_input_mode(pins, pin_count, pin_mask, mode);
}

/**
 * @brief Disables a GPIO pin on the APiI, making it available to be used 
 *        by anyone on the system.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval 0 Pin successfully disabled on the API.
 * @retval -1 Could not disable an ative interrupt on this pin.
 */
int gpio_disable_pin(int dev_pin)
{
  rtems_status_code sc;
  rpi_gpio_pin *pin;

  pin = &gpio_pin[dev_pin-1];

  pin->pin_type = NOT_USED;
 
  /* If the pin has an enabled interrupt then remove the handler. */
  if ( pin->enabled_interrupt != NONE ) {
    sc = gpio_disable_interrupt(dev_pin);
    
    if ( sc != RTEMS_SUCCESSFUL )
      return -1;
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
 * @retval 0 JTAG interface successfully configured.
 * @retval -1 At least one of the required pins is currently occupied.
 */
int gpio_select_jtag(void)
{
  /* Setup gpio 4 alt5 ARM_TDI. */
  if ( gpio_select_pin(4, ALT_FUNC_5) < 0 )
      return -1;

  /* Setup gpio 22 alt4 ARM_TRST. */
  if ( gpio_select_pin(22, ALT_FUNC_4) < 0 )
      return -1;

  /* Setup gpio 24 alt4 ARM_TDO. */
  if ( gpio_select_pin(24, ALT_FUNC_4) < 0 )
      return -1;

  /* Setup gpio 25 alt4 ARM_TCK. */
  if ( gpio_select_pin(25, ALT_FUNC_4) < 0 )
      return -1;

  /* Setup gpio 27 alt4 ARM_TMS. */
  if ( gpio_select_pin(27, ALT_FUNC_4) < 0 )
      return -1;
    
  return 0;
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
 * @retval 0 SPI interface successfully configured.
 * @retval -1 At least one of the required pins is currently occupied.
 */
int gpio_select_spi_p1(void)
{
  /* SPI master 0 MISO data line. */
  if ( gpio_select_pin(9, ALT_FUNC_0) < 0 )
      return -1;

  /* SPI master 0 MOSI data line. */
  if ( gpio_select_pin(10, ALT_FUNC_0) < 0 )
      return -1;

  /* SPI master 0 SCLK clock line. */
  if ( gpio_select_pin(11, ALT_FUNC_0) < 0 )
      return -1;

  /* SPI master 0 CE_0 chip enable line. */
  if ( gpio_select_pin(8, ALT_FUNC_0) < 0 )
      return -1;

  /* SPI master 0 CE_1 chip enable line. */
  if ( gpio_select_pin(7, ALT_FUNC_0) < 0 )
      return -1;
    
  return 0;
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
 * @retval 0 JTAG interface successfully configured.
 * @retval -1 At least one of the required pins is currently occupied.
 */
int gpio_select_i2c_p1_rev2(void)
{
  int pins[] = {2,3};

  /* I2C BSC1 SDA data line. */
  if ( gpio_select_pin(2, ALT_FUNC_0) < 0 )
      return -1;

  /* I2C BSC1 SCL clock line. */
  if ( gpio_select_pin(3, ALT_FUNC_0) < 0 )
      return -1;

  /* Enable pins 2 and 3 pull-up resistors. */
  if ( gpio_setup_input_mode(pins, 2, PULL_UP) < 0 )
    return -1;
    
  return 0;
}

/**
 * @brief De-bounces a switch by requiring a certain time to pass between interrupts.
 *        Any interrupt fired too close to the last will be ignored as it is probably
 *        the result of a involuntary switch/button bounce after being released.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval 0 Interrupt is likely provoked by a user press on the switch.
 * @retval -1 Interrupt was generated too close to the last one. Probable switch bounce.
 */
static int debounce_switch(int dev_pin)
{
  rtems_interval time;
  rpi_gpio_pin *pin;

  pin = &gpio_pin[dev_pin-1];

  time = rtems_clock_get_ticks_since_boot();

  if ( (time - pin->h_args.last_isr_tick) < pin->h_args.debouncing_tick_count )
    return -1;

  pin->h_args.last_isr_tick = time;

  return 0;
}

/**
 * @brief Generic ISR that clears the event register on the Raspberry Pi and calls 
 *        an user defined ISR. 
 *
 * @param[in] arg Void pointer to a handler_arguments structure. 
 */
static void generic_handler(void* arg)
{
  handler_arguments* handler_args;
  int rv = 0;
  int pin = 0;

  handler_args = (handler_arguments*) arg;

  pin = handler_args->pin_number;

  /*  If the interrupt was generated by the pin attached to this ISR clears it. */
  if ( BCM2835_REG(BCM2835_GPIO_GPEDS0) & (1 << pin) )
    BCM2835_REG(BCM2835_GPIO_GPEDS0) &= (1 << pin);

  /* If not lets the next ISR process the interrupt. */
  else
    return;
  
  /* If this pin has the deboucing function attached, call it. */
  if ( handler_args->debouncing_tick_count > 0 ) {
    rv = debounce_switch(pin);
   
    if ( rv < 0 )
      return;
  }

  /* Call the user's ISR. */  
  (handler_args->handler) ();
}

/**
 * @brief Defines for a GPIO input pin the number of clock ticks that must pass before
 *        an generated interrupt is garanteed to be generated by the user and not by
 *        a bouncing switch/button.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval 0 De-bounce function successfully attached to the pin.
 * @retval -1 The current pin is not configured as a digital input, hence
 *            it can not be connected to a switch.
 */
int gpio_debounce_switch(int dev_pin, int ticks)
{
  if ( gpio_pin[dev_pin-1].pin_type != DIGITAL_INPUT )
    return -1;

  gpio_pin[dev_pin-1].h_args.debouncing_tick_count = ticks;

  return 0;
}

/**
 * @brief Enables interrupts to be generated on a given GPIO pin.
 *        When fired that interrupt will call the given handler.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 * @param[in] interrupt Type of interrupt to enable for the pin.
 * @param[in] handler Pointer to a function that will be called every time 
 *                    @var interrupt is generated. This function must have 
 *                    no receiving parameters and return void.
 *
 * @retval 0 Interrupt successfully enabled for this pin.
 * @retval -1 Could not replace the currently active interrupt on this pin, or
 *            unknown @var interrupt.
 */
int gpio_enable_interrupt(int dev_pin, gpio_interrupt interrupt, void (*handler)(void))
{
  rtems_status_code sc; 
  rpi_gpio_pin *pin;

  /* Only consider GPIO pins up to 31. */
  if ( dev_pin > 31 )
    return -1;

  pin = &gpio_pin[dev_pin-1];

  /* If the pin already has an enabled interrupt removes it first,
   * as well as its handler. */
  if ( pin->enabled_interrupt != NONE ) {
    sc = gpio_disable_interrupt(dev_pin);
    
    if ( sc != RTEMS_SUCCESSFUL )
      return -1;
  }

  pin->h_args.pin_number = dev_pin;
  pin->h_args.handler = handler;

  pin->h_args.last_isr_tick = rtems_clock_get_ticks_since_boot();

  /* Installs the generic_handler, which will call the user handler received 
   * a parameter. */
  sc = rtems_interrupt_handler_install(BCM2835_IRQ_ID_GPIO_0, 
                                       NULL, 
                                       RTEMS_INTERRUPT_SHARED, 
                                       (rtems_interrupt_handler) generic_handler, 
                                       &(pin->h_args));

  if ( sc != RTEMS_SUCCESSFUL )
    return -1;

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
      return 0;

    default:
      return -1; 
  }

  pin->enabled_interrupt = interrupt;
  
  return 0;
}

/**
 * @brief Stops interrupts from being generated from a given GPIO pin
 *        and removes the corresponding handler.
 *
 * @param[in] dev_pin Raspberry Pi GPIO pin label number (not its position 
 *            on the header). 
 *
 * @retval 0 Interrupt successfully disabled for this pin.
 * @retval -1 Could not remove the current interrupt handler or could not
 *            recognise the current active interrupt on this pin.
 */
int gpio_disable_interrupt(int dev_pin)
{
  rtems_status_code sc;
  rpi_gpio_pin *pin;

  pin = &gpio_pin[dev_pin-1];

  switch ( pin->enabled_interrupt ) {
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

    case NONE:
      return 0;

    default:
      return -1;  
  }

  /* Removes the handler. */
  sc = rtems_interrupt_handler_remove(BCM2835_IRQ_ID_GPIO_0, 
                                      (rtems_interrupt_handler) generic_handler, 
                                      &(pin->h_args));

  if ( sc != RTEMS_SUCCESSFUL )
    return -1;

  pin->enabled_interrupt = NONE;

  return 0;
}
