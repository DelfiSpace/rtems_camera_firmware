/*
 * Hello world example
 */
#include <rtems.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

rtems_task Init(rtems_task_argument ignored) {
  volatile bool test_heart_beat = 1;
  while (1) {
    printf("\nHello World\n");
    sleep(1);
    test_heart_beat = 0;
    sleep(1);
    test_heart_beat = 1;
  }
  exit(0);
}
