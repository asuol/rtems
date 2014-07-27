/**
 * @file
 *
 * @ingroup raspberrypi_gpio
 *
 * @brief Raspberry Pi gpio API implementation.
 *
 */

/*
 * Copyright (c) 2014 Andre Marques.
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <bsp/raspberrypi.h>
#include <bsp/irq.h>
#include <bsp/gpio.h>

#include <stdlib.h>

#define select_pin_function(fn, pn) (fn<<(((pn)%10)*3))

static bool is_initialized = false;

void generic_handler(void *arg);

rpi_gpio_pin *gpio_pin;

/* Waits a number of CPU cycles */
static void arm_delay (int cycles)
{
  int i;

  for (i = 0; i < cycles; i++)
    asm volatile ("nop");
}

/* 
 * Initializes the GPIO API. 
 * Allocates space to the gpio_pin array and sets every pin as NOT_USED.
 * If the API has already been initialized silently exits.
 */
void gpio_initialize(void)
{
  int i;

  if ( is_initialized )
    return;

  is_initialized = true;

  gpio_pin = (rpi_gpio_pin *) malloc(GPIO_PIN_COUNT * sizeof(rpi_gpio_pin));

  for ( i = 0; i < GPIO_PIN_COUNT; i++ )
  {
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
  volatile unsigned int *pin_addr = (unsigned int *)BCM2835_GPIO_REGS_BASE + (pin / 10);
  
  if ( gpio_pin[pin-1].pin_type != NOT_USED )
    return -1;

  /* Sets pin function select bits as zero (DIGITAL_INPUT)*/
  *(pin_addr) &= ~select_pin_function(7, pin);

  switch (type)
  {
    case DIGITAL_INPUT:

      /* Digital input is set by default before this switch */

      break;

    case DIGITAL_OUTPUT:

      *(pin_addr) |= select_pin_function(1, pin);

      break;

    case ALT_FUNC_0:

      *(pin_addr) |= select_pin_function(4, pin);

      break;

    case ALT_FUNC_1:

      *(pin_addr) |= select_pin_function(5, pin);

      break;

    case ALT_FUNC_2:

      *(pin_addr) |= select_pin_function(6, pin);

      break;

    case ALT_FUNC_3:

      *(pin_addr) |= select_pin_function(7, pin);

      break;

    case ALT_FUNC_4:

      *(pin_addr) |= select_pin_function(3, pin);

      break;

    case ALT_FUNC_5:

      *(pin_addr) |= select_pin_function(2, pin);

      break;

    default:
      return -1;
  }

  gpio_pin[pin-1].pin_type = type;

  return 0;
}

/* Sets the operating mode of one or more GPIO input pins */
static int set_input_mode(int *pins, int pin_count, int pin_mask, rpi_gpio_input_mode mode)
{
  int i;

  /* Set control signal */
  switch(mode)
  {
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

  /* Wait 150 cyles, as per BCM2835 documentation */
  arm_delay(150);

  /* Setup clock for the control signal */
  BCM2835_REG(BCM2835_GPIO_GPPUDCLK0) = pin_mask;

  arm_delay(150);

  /* Remove the control signal */
  BCM2835_REG(BCM2835_GPIO_GPPUD) = 0;

  /* Remove the clock */
  BCM2835_REG(BCM2835_GPIO_GPPUDCLK0) = 0;

  for ( i = 0; i < pin_count; i++ )
    gpio_pin[pins[i]-1].mode.input = mode;

  return 0;
}

/* Sets the operating mode for only one GPIO input pin */
int gpio_input_mode(int pin, rpi_gpio_input_mode mode)
{
  int pin_mask = (1 << pin);
  int pins[1];

  /* If trying to use the same mode, silently exits */
  if ( gpio_pin[pin-1].mode.input == mode )
    return 0;

  pins[0] = pin;

  return set_input_mode(pins, 1, pin_mask, mode);
}

/* Sets the operating mode of multiple GPIO input pins (max of 32 pins at a time) */
int gpio_setup_input_mode(int *pins, int pin_count, rpi_gpio_input_mode mode)
{
  uint32_t pin_mask = 0;
  int diff_mode_counter = 0;
  int i;

  if ( pin_count > 32 )
    return -1;

  for ( i = 0; i < pin_count; i++ )
  {
    if ( gpio_pin[pins[i]-1].mode.input != mode )
      pin_mask |= (1 << pins[i]);
    
    else
      diff_mode_counter++;
  }

  /* If trying to set the same mode each pin already has, silently exits.
   * If at least one pin is to be set a different mode than it currently has continues, 
   * as it will take almost the same time to set 1 or 32 pins.
   */
  if ( diff_mode_counter == 0 )
    return 0;

  return set_input_mode(pins, pin_count, pin_mask, mode);
}

/* Disables a GPIO pin, making it available to be used by anyone */
int gpio_disable_pin(int dev_pin)
{
  rtems_status_code sc;
  rpi_gpio_pin *pin;

  pin = &gpio_pin[dev_pin-1];

  pin->pin_type = NOT_USED;

  if ( pin->enabled_interrupt != NONE )
  {
    sc = gpio_disable_interrupt(dev_pin);
    
    if ( sc != RTEMS_SUCCESSFUL )
      return -1;
  }
    
  return sc;
}

/* Allows to setup a JTAG interface using the main (P1) GPIO pin header */
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

/* 
 * Generic ISR that clears the event register on the Raspberry Pi and calls 
 * the user defined ISR.
 */
void generic_handler(void* arg)
{
  handler_arguments* handler_args;
  int rv = 0;
  int pin = 0;

  handler_args = (handler_arguments*) arg;

  pin = handler_args->pin_number;

  /*  If the interrupt was generated by the pin attached to this ISR clears it */
  if ( BCM2835_REG(BCM2835_GPIO_GPEDS0) & (1 << pin) )
    BCM2835_REG(BCM2835_GPIO_GPEDS0) &= (1 << pin);

  /* If not lets the next ISR process the interrupt */
  else
    return;
  
  if ( handler_args->debouncing_tick_count > 0 )
  {
    rv = debounce_switch(pin);
   
    if ( rv < 0 )
      return;
  }
  
  (handler_args->handler) ();
}

int gpio_debounce_switch(int dev_pin, int ticks)
{
  if ( gpio_pin[dev_pin-1].pin_type != DIGITAL_INPUT )
    return -1;

  gpio_pin[dev_pin-1].h_args.debouncing_tick_count = ticks;

  return 0;
}

int gpio_enable_interrupt(int dev_pin, gpio_interrupt interrupt, void (*handler) (void))
{
  rtems_status_code sc; 
  rpi_gpio_pin *pin;

  /* Only consider GPIO pins up to 31 */
  if ( dev_pin > 31 )
    return -1;

  pin = &gpio_pin[dev_pin-1];

  /* If the pin already has an enabled interrupt */
  if ( pin->enabled_interrupt != NONE )
  {
    sc = gpio_disable_interrupt(dev_pin);
    
    if ( sc != RTEMS_SUCCESSFUL )
      return -1;
  }

  pin->h_args.pin_number = dev_pin;
  pin->h_args.handler = handler;

  pin->h_args.last_isr_tick = rtems_clock_get_ticks_since_boot();

  sc = rtems_interrupt_handler_install(BCM2835_IRQ_ID_GPIO_0, NULL, RTEMS_INTERRUPT_SHARED, (rtems_interrupt_handler) generic_handler, &(pin->h_args));

  if ( sc != RTEMS_SUCCESSFUL )
    return -1;

  switch ( interrupt )
  {
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

int gpio_disable_interrupt(int dev_pin)
{
  rtems_status_code sc;
  rpi_gpio_pin *pin;

  pin = &gpio_pin[dev_pin-1];

  switch ( pin->enabled_interrupt )
  {
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

  /* Removes the handler */
  sc = rtems_interrupt_handler_remove(BCM2835_IRQ_ID_GPIO_0, (rtems_interrupt_handler) generic_handler, &(pin->h_args));

  if ( sc != RTEMS_SUCCESSFUL )
    return -1;

  pin->enabled_interrupt = NONE;

  return 0;
}
