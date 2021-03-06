/*  system.h
 *
 *  This include file contains information that is included in every
 *  function in the test set.
 */

/*
 *  COPYRIGHT (c) 1989-2012.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <tmacros.h>

#define US_PER_TICK 10000

/* functions */

rtems_task Init(
  rtems_task_argument argument
);

/* configuration information */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_MICROSECONDS_PER_TICK US_PER_TICK

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_INIT_TASK_PRIORITY        2
#define CONFIGURE_INIT_TASK_INITIAL_MODES   RTEMS_PREEMPT

#define CONFIGURE_MAXIMUM_TASKS             2
#define CONFIGURE_MAXIMUM_TIMERS            1

#include <rtems/confdefs.h>

/* end of include file */
