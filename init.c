/*
 * Simple RTEMS configuration
 */

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

// #define CONFIGURE_UNLIMITED_OBJECTS
// #define CONFIGURE_UNIFIED_WORK_AREAS

// #define CONFIGURE_EXECUTIVE_RAM_SIZE 150000

#define CONFIGURE_MAXIMUM_TASKS 2
#define CONFIGURE_MAXIMUM_SEMAPHORES 1

// #define CONFIGURE_HEAP_SIZE (64 * 1024) // XXX: new

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
