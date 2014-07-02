/**
 * @file
 *
 * @ingroup raspberrypi_libgpio
 *
 * @brief Raspberry Pi libgpio API implementation.
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

#include <rtems/libgpio.h>

#include <stdlib.h>

#define select_pin_function(fn, pn) (fn<<(((pn)%10)*3))

static rtems_interval ticks = 0;

static bool is_initialized = false;

void handler(void *arg);

rtems_gpio_pin *gpio_pin;

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
void rtems_gpio_initialize(int gpio_count)
{
  int i;

  if ( is_initialized )
    return;

  is_initialized = true;

  ticks = rtems_clock_get_ticks_per_second();

  gpio_pin = (rtems_gpio_pin *) malloc(gpio_count * sizeof(rtems_gpio_pin));

  for ( i = 0; i < gpio_count; i++ )
    gpio_pin[i].pin_type = NOT_USED;
}

/* Gives an output GPIO pin the logical value of 1 */
int rtems_gpio_set(int pin)
{
  if (gpio_pin[pin-1].pin_type != DIGITAL_OUTPUT)
    return -1;

  BCM2835_REG(BCM2835_GPIO_GPSET0) = (1 << pin);

  return 0;
}

/* Gives an output GPIO pin the logical value of 0 */
int rtems_gpio_clear(int pin)
{
  if (gpio_pin[pin-1].pin_type != DIGITAL_OUTPUT)
    return -1;

  BCM2835_REG(BCM2835_GPIO_GPCLR0) = (1 << pin);

  return 0;
}

/* Gets the level, or value, of a GPIO input pin */
int rtems_gpio_get_val(int pin)
{
  return BCM2835_REG(BCM2835_GPIO_GPLEV0) &= (1 << (pin));
}

/* Selects a GPIO pin operation or function */
int rtems_gpio_select_pin(int pin, rtems_pin type)
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
static int rpi_gpio_input_mode(int pin_mask, rtems_multiio_input_mode mode)
{
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

  return 0;
}

/* Sets the operating mode for only one GPIO input pin */
int rtems_gpio_input_mode(int pin, rtems_multiio_input_mode mode)
{
  int pin_mask = (1 << pin);

  return rpi_gpio_input_mode(pin_mask, mode);
}

/* Sets the operating mode of multiple GPIO input pins */
int rtems_gpio_setup_input_mode(int *pin, int pin_count, rtems_multiio_input_mode mode)
{
  int i;

  int pin_mask = 0;

  for ( i = 0; i < pin_count; i++ )
    pin_mask |= (1 << pin[i]);

  return rpi_gpio_input_mode(pin_mask, mode);
}

/* Disables a GPIO pin, making it available to be used by anyone */
void rtems_gpio_disable_pin(int pin)
{
  gpio_pin[pin-1].pin_type = NOT_USED;
}

/* Allows to setup multiple GPIO pins to a specific configuration */
int rtems_gpio_select_config(rtems_gpio_configuration *pin_setup, int pin_count)
{
  int i;

  for ( i = 0; i < pin_count; i++ )
    if (rtems_gpio_select_pin(pin_setup[i].pin_number, pin_setup[i].pin_function) < 0)
      return -1;
    
  return 0;
}

static int debounce_switch(int pin)
{
  rtems_interval time;

  time = rtems_clock_get_ticks_since_boot();

  if (time - gpio_pin[pin-1].pin_data.din.h_args.last_isr_tick < (ticks * 0.05) )
    return -1;

  gpio_pin[pin-1].pin_data.din.h_args.last_isr_tick = time;

  return 0;
}

/* 
 * Generic ISR that clears the event register on the Raspberry Pi and calls 
 * the user defined ISR.
 */
void handler (void* arg)
{
  handler_arguments* handler_args;
  int rv = 0;
  int pin = 0;

  handler_args = (handler_arguments*) arg;

  pin = handler_args->pin_number;

  /*  If the interrupt was generated by the pin attached to this ISR clears it */
  if (BCM2835_REG(BCM2835_GPIO_GPEDS0) & (1 << pin))
    BCM2835_REG(BCM2835_GPIO_GPEDS0) &= (1 << pin);

  /* If not lets the next ISR process the interrupt */
  else
    return;
  
  if ( gpio_pin[pin-1].pin_type == DIGITAL_INPUT )
  {
    rv = debounce_switch(pin);
   
    if ( rv <= 0 )
      return;
  }

  (handler_args->handler) ();
}

int rtems_gpio_enable_interrupt(int pin, rtems_din_interrupt interrupt, void (*handler) (void))
{
  rtems_status_code sc;  

  /* Only consider GPIO pins up to 31 */
  if ( pin > 31 )
    return -1;

  if ( gpio_pin[pin-1].pin_type == DIGITAL_INPUT )
  {
    gpio_pin[pin-1].pin_data.din.h_args.pin_number = pin;
    gpio_pin[pin-1].pin_data.din.h_args.handler = handler;

    gpio_pin[pin-1].pin_data.din.h_args.last_isr_tick = rtems_clock_get_ticks_since_boot();
  }

  switch (interrupt)
  {
    case RISING_EDGE:
    
      /* Enables asynchronous rising edge detection */
      BCM2835_REG(BCM2835_GPIO_GPREN0) |= (1 << pin);
      
      break;

    case FALLING_EDGE:

      /* Enables asynchronous falling edge detection */
       BCM2835_REG(BCM2835_GPIO_GPFEN0) |= (1 << pin);

      break;

    case BOTH_EDGES:

      /* Enables asynchronous falling edge detection */
       BCM2835_REG(BCM2835_GPIO_GPAFEN0) |= (1 << pin);

      /* Enables asynchronous rising edge detection */
      BCM2835_REG(BCM2835_GPIO_GPAREN0) |= (1 << pin);
      
      break;

    default:
      return -1; 
  }

  sc = rtems_interrupt_handler_install(BCM2835_IRQ_ID_GPIO_0, NULL, RTEMS_INTERRUPT_SHARED, (rtems_interrupt_handler) handler, &(gpio_pin[pin-1].pin_data.din.h_args));

  if(sc != RTEMS_SUCCESSFUL)
    return -1;

  return 0;
}
