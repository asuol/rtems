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

const char rtems_test_name[] = "LIBGPIO_JTAG";

rtems_task Init(
  rtems_task_argument ignored
)
{
  int rv = 0;

  rtems_test_begin ();
  
  /* Initializes the GPIO API */
  gpio_initialize ();

  /* Disables the setup pins internal pull resistor */
  int pins[] = {4,22,24,25,27};

  rv = gpio_setup_input_mode (pins, 5, NO_PULL_RESISTOR);
  RTEMS_CHECK_RV ( rv, "gpio_setup_input_mode");
  
  /* Setups the JTAG pin interface */
  rv = gpio_select_jtag ();
  RTEMS_CHECK_RV ( rv, "gpio_setup_config");
  
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
