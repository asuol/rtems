#include <bsp/raspberrypi.h>

#include <rtems/libgpio.h>

// GPIO setup macros
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |= (1<<(((g)%10)*3))

#define GPIO_SET *(gpio+7) // sets bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0
#define GPIO_READ(g) *(gpio+13) &= (1<<(g))

static volatile unsigned int *gpio = (unsigned int *)BCM2835_GPIO_REGS_BASE;

static bool is_initialized = false;

void rtems_gpio_initialize (void)
{
  if (is_initialized)
    return;

  is_initialized = true;

  int i;

  for (i = 0; i < RTEMS_GPIO_COUNT; i++)
  {
    gpio_pin[i].address = *(unsigned int*) gpio + (i/10);

    gpio_pin[i].pin_type = NOT_USED;
  }
}

int rtems_gpio_set_sb (int pin)
{
  rtems_pin_t pin_type;

  pin_type = gpio_pin[pin-1].pin_type;

  if (pin_type == NOT_USED || pin_type == DIGITAL_INPUT)
    return -1;

  GPIO_SET = 1 << pin;

  return 0;
}

int rtems_gpio_clear_sb (int pin)
{
  rtems_pin_t pin_type;

  pin_type = gpio_pin[pin-1].pin_type;

  if (pin_type == NOT_USED || pin_type == DIGITAL_OUTPUT)
    return -1;

  GPIO_CLR = 1 << pin;

  return 0;
}

int rtems_gpio_config_pin (int pin, rtems_pin_t type)
{
  rtems_pin_t pin_type;

  pin_type = gpio_pin[pin-1].pin_type;

  if (pin_type != NOT_USED)
    return -1;

  if (type == DIGITAL_INPUT)
    INP_GPIO (pin);
    
  else if (type == DIGITAL_OUTPUT)
    OUT_GPIO (pin);
  
  gpio_pin[pin-1].pin_type = type;

  return 0;
}

void rtems_gpio_disable_pin (int pin)
{
  gpio_pin[pin-1].pin_type = NOT_USED;
}
