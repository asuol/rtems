#include <bsp/raspberrypi.h>

#include <rtems/libgpio.h>

#include <stdio.h>

static volatile unsigned int *gpio_regs_base = (unsigned int *)BCM2835_GPIO_REGS_BASE;

static bool is_initialized = false;

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

int rtems_gpio_input_mode (int pin, rtems_multiio_input_mode_t mode)
{

  // check current mode first

  // add bitmask mode

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
  BCM2835_REG (BCM2835_GPIO_GPPUDCLK0) = (1<<pin);

  arm_delay (150);

  /* Remove the control signal */
  BCM2835_REG (BCM2835_GPIO_GPPUD) = 0;

  /* Remove the clock */
  BCM2835_REG (BCM2835_GPIO_GPPUDCLK0) = 0;

  return 0;
}

void rtems_gpio_disable_pin (int pin)
{
  gpio_pin[pin-1].pin_type = NOT_USED;
}
