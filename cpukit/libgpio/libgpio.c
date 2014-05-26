#if HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/libgpio.h>

static bool is_initialized = false;

int rtems_gpio_initialize ()
{
    if ( is_initialized )
        return 0;
  
    // Set struct memory

    return 0;
}
