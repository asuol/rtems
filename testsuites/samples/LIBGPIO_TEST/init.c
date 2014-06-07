#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/test.h>

#include <bsp.h> /* for device driver prototypes */

#include <rtems/libgpio.h>
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

  rtems_test_begin();
  
  rtems_gpio_initialize ();

  rv = rtems_gpio_setup_pin (3, DIGITAL_OUTPUT);
  RTEMS_CHECK_RV( rv, "rtems_gpio_config_pin");

  int val;
  
  while(1)
  {

  val = rtems_gpio_get_val (3);
  printf("\n val = %d\n", val);

  rv = rtems_gpio_clear (3);
  RTEMS_CHECK_RV( rv, "rtems_gpio_clear");
  
  if (rtems_task_wake_after (RTEMS_MICROSECONDS_TO_TICKS (5000000)) != RTEMS_SUCCESSFUL)
    printf("\n wake fail\n");

  val = rtems_gpio_get_val (3);
  printf("\n val = %d\n", val);

  rv = rtems_gpio_set (3);
  RTEMS_CHECK_RV( rv, "rtems_gpio_set");

  if (rtems_task_wake_after (RTEMS_MICROSECONDS_TO_TICKS (5000000)) != RTEMS_SUCCESSFUL)
    printf("\n wake fail\n");
    }
  rtems_test_end();
  exit( 0 );
}


/* NOTICE: the clock driver is explicitly disabled */
//#define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_MAXIMUM_TASKS            1
#define CONFIGURE_USE_DEVFS_AS_BASE_FILESYSTEM

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
