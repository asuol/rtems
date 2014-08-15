/**
 * @file gpio.c
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Raspberry Pi gpio API implementation.
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

/* Wait a number of CPU cycles. */
static void arm_delay (int cycles)
{
  int i;

  for (i = 0; i < cycles; i++)
    asm volatile ("nop");
}

/**
 * @brief Copies from a source to a destination memory area.
 *
 * The source and destination areas may not overlap.
 * 
 * @param[out] dest The destination memory area to copy to.
 * @param[in] src The source memory area to copy from.
 * @param[in] n The number of bytes to copy.
 */

/* Initializes the GPIO API. 
 * Allocates space to the gpio_pin array and sets every pin as NOT_USED.
 * If the API has already been initialized silently exits. */
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

/* Gives an output GPIO pin the logical value of 1 */
int gpio_set(int pin)
{
  if (gpio_pin[pin-1].pin_type != DIGITAL_OUTPUT)
    return -1;

  BCM2835_REG(BCM2835_GPIO_GPSET0) = (1 << pin);

  return 0;
}

/* Gives an output GPIO pin the logical value of 0 */
int gpio_clear(int pin)
{
  if (gpio_pin[pin-1].pin_type != DIGITAL_OUTPUT)
    return -1;

  BCM2835_REG(BCM2835_GPIO_GPCLR0) = (1 << pin);

  return 0;
}

/* Gets the level, or value, of a GPIO input pin */
int gpio_get_val(int pin)
{
  return BCM2835_REG(BCM2835_GPIO_GPLEV0) &= (1 << (pin));
}

/* Selects a GPIO pin operation or function */
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

/* Sets the operating mode of one or more GPIO input pins, 
 * namely its pull-up/down resistor status. */
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
    gpio_pin[pins[i]-1].mode.input = mode;

  return 0;
}

/* Sets the pull-up/down resistors actuation mode for only one GPIO input pin. */
int gpio_input_mode(int pin, rpi_gpio_input_mode mode)
{
  int pin_mask = (1 << pin);
  int pins[1];

  /* If the desired actuation mode is already set, silently exits. */
  if ( gpio_pin[pin-1].mode.input == mode )
    return 0;

  pins[0] = pin;

  return set_input_mode(pins, 1, pin_mask, mode);
}

/* Sets the same pull-up/down resistors actuation mode to multiple GPIO input pins.
 * There is a maximum number of 32 pins per call, which is enough for 
 * Raspberry Pi models A and B (17 GPIOs on P1 GPIO header) 
 * and also model B+ (28 GPIOs on J8 GPIO header). */
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
    if ( gpio_pin[pins[i] - 1].mode.input != mode )
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

/* Disables a GPIO pin on the APiI, making it available to be used by anyone on the system. */
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

/* Allows to setup a JTAG interface using the main (P1) GPIO pin header. */
int gpio_select_jtag(void)
{
  /* setup gpio 4 alt5 ARM_TDI */
  if ( gpio_select_pin(4, ALT_FUNC_5) < 0 )
      return -1;

  /* setup gpio 22 alt4 ARM_TRST */
  if ( gpio_select_pin(22, ALT_FUNC_4) < 0 )
      return -1;

  /* setup gpio 24 alt4 ARM_TDO */
  if ( gpio_select_pin(24, ALT_FUNC_4) < 0 )
      return -1;

  /* setup gpio 25 alt4 ARM_TCK */
  if ( gpio_select_pin(25, ALT_FUNC_4) < 0 )
      return -1;

  /* setup gpio 27 alt4 ARM_TMS */
  if ( gpio_select_pin(27, ALT_FUNC_4) < 0 )
      return -1;
    
  return 0;
}

/* Allows to setup the SPI interface on the main (P1) GPIO pin header */
int gpio_select_spi_p1(void)
{
  /* SPI master 0 MISO data line */
  if ( gpio_select_pin(9, ALT_FUNC_0) < 0 )
      return -1;

  /* SPI master 0 MOSI data line */
  if ( gpio_select_pin(10, ALT_FUNC_0) < 0 )
      return -1;

  /* SPI master 0 SCLK clock line */
  if ( gpio_select_pin(11, ALT_FUNC_0) < 0 )
      return -1;

  /* SPI master 0 CE_0 chip enable line */
  if ( gpio_select_pin(8, ALT_FUNC_0) < 0 )
      return -1;

  /* SPI master 0 CE_1 chip enable line */
  if ( gpio_select_pin(7, ALT_FUNC_0) < 0 )
      return -1;
    
  return 0;
}

/* Allows to setup the I2C interface on the main (P1) GPIO pin header 
 * (model B rev2 and B+) */
int gpio_select_i2c_p1_rev2(void)
{
  int pins[] = {2,3};

  /* I2C BSC1 SDA data line */
  if ( gpio_select_pin(2, ALT_FUNC_0) < 0 )
      return -1;

  /* I2C BSC1 SCL clock line */
  if ( gpio_select_pin(3, ALT_FUNC_0) < 0 )
      return -1;

  /* Enable pins 2 and 3 pull-up resistors */
  if ( gpio_setup_input_mode(pins, 2, PULL_UP) < 0 )
    return -1;
    
  return 0;
}

/* De-bounces a switch by requiring a certain time to pass between interrupts.
 * Any interrupt fired too close to the last will be ignored as it is should
 * be the result of the involuntary hardware switch/button bouncing after its
 * being released. */
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

/* Generic ISR that clears the event register on the Raspberry Pi and calls 
 * an user defined ISR. */
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

/* Defines for a GPIO input pin the number of clock ticks that must pass before
 * an generated interrupt is garanteed to be generated by the user and not by
 * a bouncing switch/button. */
int gpio_debounce_switch(int dev_pin, int ticks)
{
  if ( gpio_pin[dev_pin-1].pin_type != DIGITAL_INPUT )
    return -1;

  gpio_pin[dev_pin-1].h_args.debouncing_tick_count = ticks;

  return 0;
}

/* Enables interrupts to be generated on a given GPIO pin.
 * When fired that interrupt will call the given handler. */
int gpio_enable_interrupt(int dev_pin, gpio_interrupt interrupt, void (*handler)(void))
{
  rtems_status_code sc; 
  rpi_gpio_pin *pin;

  /* Only consider GPIO pins up to 31 */
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

      /* Enables asynchronous falling edge detection */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) |= (1 << dev_pin);

      break;

    case RISING_EDGE:
    
      /* Enables asynchronous rising edge detection */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) |= (1 << dev_pin);
      
      break;

    case BOTH_EDGES:

      /* Enables asynchronous falling edge detection */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) |= (1 << dev_pin);

      /* Enables asynchronous rising edge detection */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) |= (1 << dev_pin);
      
      break;

    case LOW_LEVEL:
    
      /* Enables pin low level detection */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) |= (1 << dev_pin);
      
      break;

    case HIGH_LEVEL:
    
      /* Enables pin high level detection */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) |= (1 << dev_pin);
      
      break;

    case BOTH_LEVELS:
    
      /* Enables pin low level detection */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) |= (1 << dev_pin);

      /* Enables pin high level detection */
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

/* Stops interrupts from being generated from a given GPIO pin
 * and removes the corresponding handler. */
int gpio_disable_interrupt(int dev_pin)
{
  rtems_status_code sc;
  rpi_gpio_pin *pin;

  pin = &gpio_pin[dev_pin-1];

  switch ( pin->enabled_interrupt ) {
    case FALLING_EDGE:

      /* Disables asynchronous falling edge detection */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) &= ~(1 << dev_pin);

      break;

    case RISING_EDGE:
    
      /* Disables asynchronous rising edge detection */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) &= ~(1 << dev_pin);
      
      break;

    case BOTH_EDGES:

      /* Disables asynchronous falling edge detection */
      BCM2835_REG(BCM2835_GPIO_GPAFEN0) &= ~(1 << dev_pin);

      /* Disables asynchronous rising edge detection */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) &= ~(1 << dev_pin);
      
      break;

    case LOW_LEVEL:
    
      /* Disables pin low level detection */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) &= ~(1 << dev_pin);
      
      break;

    case HIGH_LEVEL:
    
      /* Disables pin high level detection */
      BCM2835_REG(BCM2835_GPIO_GPHEN0) &= ~(1 << dev_pin);
      
      break;

    case BOTH_LEVELS:
    
      /* Disables pin low level detection */
      BCM2835_REG(BCM2835_GPIO_GPLEN0) &= ~(1 << dev_pin);

      /* Disables pin high level detection */
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
