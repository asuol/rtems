#include <bsp/raspberrypi.h>
#include <rtems/libgpio.h>

#include <stdio.h>
#include <stdlib.h>

#define GPIO_COUNT 54

#define select_pin_function(fn, pn) (fn<<(((pn)%10)*3))

static volatile unsigned int *gpio_regs_base = (unsigned int *)BCM2835_GPIO_REGS_BASE;

static bool is_initialized = false;

rtems_gpio_pin *gpio_pin;

static void arm_delay (int cycles)
{
  int i;

  for (i = 0; i < cycles; i++)
    asm volatile ("nop");
}

void rtems_gpio_initialize (void)
{
  int i;

  if (is_initialized)
    return;

  is_initialized = true;

  gpio_pin = (rtems_gpio_pin *) malloc (GPIO_COUNT * sizeof(rtems_gpio_pin));

  for (i = 1; i <= GPIO_COUNT; i++)
  {
    gpio_pin[i-1].address = (unsigned int*) gpio_regs_base + (i/10);

    gpio_pin[i-1].pin_type = NOT_USED;
  }
}

int rtems_gpio_set (int pin)
{
  if (gpio_pin[pin-1].pin_type != DIGITAL_OUTPUT)
    return -1;

  *(gpio_regs_base+7) = 1 << pin;

  return 0;
}

int rtems_gpio_clear (int pin)
{
  if (gpio_pin[pin-1].pin_type != DIGITAL_OUTPUT)
    return -1;

  *(gpio_regs_base+10) = 1 << pin;

  return 0;
}

int rtems_gpio_get_val (int pin)
{
  return *(gpio_regs_base+13) &= (1<<(pin));
}

int rtems_gpio_setup_pin (int pin, rtems_pin type)
{
  unsigned int *pin_addr = (unsigned int *) gpio_pin[pin-1].address;

  if (gpio_pin[pin-1].pin_type != NOT_USED)
    return -1;

  /* Sets pin function select bits as zero (DIGITAL_INPUT)*/
  *(pin_addr) &= ~select_pin_function (7, pin);

  switch (type)
  {
    case DIGITAL_INPUT:

      /* Digital input is set by default before this switch */

      break;

    case DIGITAL_OUTPUT:

      *(pin_addr) |= select_pin_function (1, pin);

      break;

    case ALT_FUNC_0:

      *(pin_addr) |= select_pin_function (4, pin);

      break;

    case ALT_FUNC_1:

      *(pin_addr) |= select_pin_function (5, pin);

      break;

    case ALT_FUNC_2:

      *(pin_addr) |= select_pin_function (6, pin);

      break;

    case ALT_FUNC_3:

      *(pin_addr) |= select_pin_function (7, pin);

      break;

    case ALT_FUNC_4:

      *(pin_addr) |= select_pin_function (3, pin);

      break;

    case ALT_FUNC_5:

      *(pin_addr) |= select_pin_function (2, pin);

      break;

    default:
      return -1;
  }

  gpio_pin[pin-1].pin_type = type;

  return 0;
}

static int set_gpio_input_mode (int pin_mask, rtems_multiio_input_mode mode)
{
  /* Set control signal */
  switch (mode)
  {
    case PULL_UP:
      BCM2835_REG (BCM2835_GPIO_GPPUD) = (1<<1);
      break;

    case PULL_DOWN:
      BCM2835_REG (BCM2835_GPIO_GPPUD) = (1<<0);
      break;

    case NO_PULL_RESISTOR:
      BCM2835_REG (BCM2835_GPIO_GPPUD) = 0;
      break;

    default:
      return -1;
  }

  /* Wait 150 cyles, as per BCM2835 documentation */
  arm_delay (150);

  /* Setup clock for the control signal */
  BCM2835_REG (BCM2835_GPIO_GPPUDCLK0) = pin_mask;

  arm_delay (150);

  /* Remove the control signal */
  BCM2835_REG (BCM2835_GPIO_GPPUD) = 0;

  /* Remove the clock */
  BCM2835_REG (BCM2835_GPIO_GPPUDCLK0) = 0;

  return 0;
}

int rtems_gpio_input_mode (int pin, rtems_multiio_input_mode mode)
{
  int pin_mask = (1 << pin);

  return set_gpio_input_mode (pin_mask, mode);
}

int rtems_gpio_setup_input_mode (int *pin, int pin_count, rtems_multiio_input_mode mode)
{
  int i;

  int pin_mask = 0;

  for (i = 0; i < pin_count; i++)
    pin_mask |= (1 << pin[i]);

  return set_gpio_input_mode (pin_mask, mode);
}

void rtems_gpio_disable_pin (int pin)
{
  gpio_pin[pin-1].pin_type = NOT_USED;
}

int rtems_gpio_setup_config (rtems_gpio_configuration *pin_setup, int pin_count)
{
  int i;

  for (i = 0; i < pin_count; i++)
    if (rtems_gpio_setup_pin (pin_setup[i].pin_number, pin_setup[i].pin_function) < 0)
      return -1;
    
  return 0;
}
