#include <bsp/raspberrypi.h>

#include <rtems/libgpio.h>

#include <stdio.h>

#define GPIO_READ(g) *(gpio_regs_basegpio+13) &= (1<<(g))

static volatile unsigned int *gpio_regs_base = (unsigned int *)BCM2835_GPIO_REGS_BASE;

static bool is_initialized = false;

void rtems_gpio_initialize (void)
{
  int i;

  if (is_initialized)
    return;

  is_initialized = true;

  for (i = 1; i <= RTEMS_GPIO_COUNT; i++)
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

int rtems_gpio_setup_pin (int pin, rtems_pin_t type)
{
  unsigned int *pin_addr = (unsigned int *) gpio_pin[pin-1].address;

  if (gpio_pin[pin-1].pin_type != NOT_USED)
    return -1;

    switch (type)
  {
    case DIGITAL_INPUT:
      
      *(pin_addr) &= ~(7<<(((pin)%10)*3));

      break;

    case DIGITAL_OUTPUT:

      *(pin_addr) |= (1<<(((pin)%10)*3));

      break;

    default:
      return -1;
  }

  gpio_pin[pin-1].pin_type = type;

  return 0;
}

void rtems_gpio_disable_pin (int pin)
{
  gpio_pin[pin-1].pin_type = NOT_USED;
}
