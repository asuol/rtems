#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/test.h>

#include <bsp.h> /* for device driver prototypes */

#include <bsp/gpio.h> /* Calls the BSP gpio library */
#include <rtems/status-checks.h>

#include <stdio.h>
#include <stdlib.h>

/* forward declarations to avoid warnings */
rtems_task Init(rtems_task_argument argument);

const char rtems_test_name[] = "LIBGPIO_TEST";

rtems_task Init(
  rtems_task_argument ignored
)
{
  int rv = 0;
  int val;

  rtems_test_begin ();
  
  /* Initializes the GPIO API */
  rtems_gpio_initialize (GPIO_PIN_COUNT);

  rv = rtems_gpio_select_pin (3, DIGITAL_OUTPUT);
  RTEMS_CHECK_RV ( rv, "rtems_gpio_config_pin output") ;

  rv = rtems_gpio_select_pin (2, DIGITAL_INPUT);
  RTEMS_CHECK_RV ( rv, "rtems_gpio_config_pin input");

  /* Enables the internal pull up resistor on the GPIO 2 pin */
  rv = rtems_gpio_input_mode (2, PULL_UP);
  RTEMS_CHECK_RV ( rv, "rtems_gpio_input_mode");
  
  rv = rtems_gpio_clear (3);
  RTEMS_CHECK_RV ( rv, "rtems_gpio_clear");

  /* Polls the GPIO 2 pin.
   * 
   * If pressed sets the GPIO 3 pin (lits the connected LED), 
   * else clears the pin (turns off the LED). 
   */
  while (1)
  {
    val = rtems_gpio_get_val (2);
  
    if (val == 0)
    {
      rv = rtems_gpio_set (3);
      RTEMS_CHECK_RV ( rv, "rtems_gpio_set");
    }

    else
    {
      rv = rtems_gpio_clear (3);
      RTEMS_CHECK_RV ( rv, "rtems_gpio_clear");
    }
  }

  rtems_test_end ();
  exit ( 0 );
}

#define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_MAXIMUM_TASKS            1
#define CONFIGURE_USE_DEVFS_AS_BASE_FILESYSTEM

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
