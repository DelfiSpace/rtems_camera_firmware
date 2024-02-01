/* TODO:
 * Add licence
 */

#include <rtems.h>
#include <rtems/irq.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32l4r9xx.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* BSP includes */
#include <hwlist_agent.h>
#include <stm32l4r9_module_clk_config.h>
#include <stm32l4r9_module_dcmi.h>
#include <stm32l4r9_module_i2c.h>
#include <stm32l4r9_module_mspi.h>
#include <stm32l4r9_module_mspi_mt29.h>
#include <stm32l4r9_module_uart.h>

/* ST includes */
#include "gpio.h"
#include "i2c.h"
#include "main.h"

/* Application includes */
#include "frame_handler.h"
// #include "test_nand_routines.h"

/* TODO TABLE:
 * fix memories... be in a condition wher you can write and read consistently
 *
 * create handler for writing dcma images
 * integrate a timer to measure... time betweeen events duh
 * fix loading the images in nand
 *
 * calibrate dcmi images
 * create a better neovim configuration with dap
 */

volatile struct jpeg_image last_image;
/* WARN: rework the memory read system */
/* WARN: rework the memory system so that page 0 of block 0 is not used */

rtems_id fa_semaphore_id; // frame available

static u8 go = {0};

/*FIX: ADD HAL UART INITIALIZATION (IF NEEDED IDK)*/

rtems_task Init(rtems_task_argument ignored) {
  // ------------ SYTSTEM INITIALIZATION  -------------------------------------

  hwlist_require(&hw_head, &debug_uart_init, NULL);
  hwlist_require(&hw_head, &dcmi_init, NULL);
  hwlist_require(&hw_head, &mspi_init, NULL);
  uart_write_buf(USART2, "BSP initialization complete \n\r", 30);

  /* allows to proceed only when a debugger is attached */
  /*
  while (go == 0) {
    uart_write_buf(USART2, ".", 1);
    uart_write_buf(USART2, ".", 1);
    uart_write_buf(USART2, ".", 1);
    uart_write_buf(USART2, "\n\r", 2);
    spin(12000000);
  }
  */

  /* ---- APPLICATION INITIALIZATION START ---- */
  /* --- get octospi objects */
  struct mspi_interface octospi1 = {.interface_select = 0x01};
  struct mspi_device mt29 = mspi_device_constr();

  /* --- dcmi memory system initialization */

  /* read the memory and get context */
  get_image_storage_status(octospi1, mt29, &last_image);

  struct dcmi_isr_arg DCMI_frame_isr_arguments = {.image_storage_struct =
                                                      &last_image,
                                                  .mspi_interface = octospi1,
                                                  .mspi_device = mt29};

  /* ---- TEST EXECUTION ---------------------- */

  /* ---- TEST EXECUTION ---------------------- */
  // Create binary semaphore
  rtems_status_code status_s_create;
  status_s_create = rtems_semaphore_create(
      rtems_build_name('S', 'E', 'M', '1'),
      0, // Initial count
      RTEMS_BINARY_SEMAPHORE | RTEMS_PRIORITY | RTEMS_INHERIT_PRIORITY, 0,
      &fa_semaphore_id);

  /* ---- TASK DEFINITION --------------------- */
  rtems_id tid;
  rtems_status_code status;
  rtems_name frame_task_name;
  frame_task_name = rtems_build_name('F', 'R', 'M', '1');

  /* ---- APPLICATION INITIALIZATION START ---- */

  status = rtems_task_create(frame_task_name, 1, RTEMS_MINIMUM_STACK_SIZE,
                             RTEMS_NO_PREEMPT, RTEMS_FLOATING_POINT, &tid);

  if (status != RTEMS_SUCCESSFUL) {
    uart_write_buf(USART2, "task create failed............. \n\r", 34); //
    // XXX:
    exit(1);
  }

  status = rtems_task_start(tid, DCMI_frame_handler, 0);

  if (status != RTEMS_SUCCESSFUL) {
    uart_write_buf(USART2, "task start failed.............. \n\r", 34); //
    // XXX:
    exit(1);
  }
  uart_write_buf(USART2, "task init continue............. \n\r", 34); // XXX:

  status = rtems_task_delete(RTEMS_SELF); /* should not return */

  uart_write_buf(USART2, "task delete failed............. \n\r", 34); // XXX:
  exit(1);
#ifdef TASK
#endif
  while (1) {
  };
}

void Error_Handler(void) {
  //__disable_irq();
  while (1) {
  }
}
