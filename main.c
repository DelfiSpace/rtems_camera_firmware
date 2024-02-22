/* TODO:
 * Add licence
 */

#include <rtems.h>
#include <rtems/irq.h>
#include <rtems/rtems/tasks.h>
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

/* Test includes */
// #include "test_nand_routines.h"
// #include "test_image_storage_routines.h"

struct jpeg_image last_image; // TODO: remove from here.

// command variables for gdb interfaces
int clean_nand_n = 0;
int search_block_limit = SEARCH_BLOCK_LIMIT;

rtems_id frame_handler_tid;

rtems_task Init(rtems_task_argument ignored) {
  // ------------ SYTSTEM INITIALIZATION  -------------------------------------

  hwlist_require(&hw_head, &debug_uart_init, NULL);
  hwlist_require(&hw_head, &dcmi_init, NULL);
  hwlist_require(&hw_head, &mspi_init, NULL);
  uart_write_buf(USART2, "BSP initialization complete \n\r", 30);

  /* ---- APPLICATION INITIALIZATION START ---- */
  /* --- get octospi objects */
  struct mspi_interface octospi1 = {.interface_select = 0x01};
  struct mspi_device mt29 = mspi_device_constr();

  /* --- dcmi memory system initialization */
  struct dcmi_isr_arg DCMI_frame_task_arguments = {.image_storage_struct =
                                                       &last_image,
                                                   .mspi_interface = octospi1,
                                                   .mspi_device = mt29};

  /* ---- Debug gpios */
  GPIOE->MODER &= ~GPIO_MODER_MODE12_Msk;
  GPIOE->MODER |= 0b01 << GPIO_MODER_MODE12_Pos;

  GPIOE->MODER &= ~GPIO_MODER_MODE15_Msk;
  GPIOE->MODER |= 0b01 << GPIO_MODER_MODE15_Pos;

  /* ---- TEST CLEAR THE BLOCK ---------------- */
#ifdef CLEAR_NAND
  for (int i = 0; i < clean_nand_n; i++) {
    struct nand_addr page_nand_addr = {.block = i, .page = 0, .column = 0};
    mspi_transfer(octospi1, mt29.write_unlock, &page_nand_addr);
    mspi_transfer(octospi1, mt29.write_enable, &page_nand_addr);
    mspi_transfer(octospi1, mt29.block_erase, &page_nand_addr);
    mspi_transfer(octospi1, mt29.wait_oip, &page_nand_addr);
  }

  // while (1) {
  uart_write_buf(USART2, "nand_cleared \n\r", 15);
  //}
#endif
  /* read the memory and get context */
  get_image_storage_status(&DCMI_frame_task_arguments);

  /* ---- REGISTER ISR ------------------------ */
  volatile rtems_status_code ir_rs = {0};
  ir_rs = register_dcmi_frame_isr();
  if (ir_rs != RTEMS_SUCCESSFUL) {
    uart_write_buf(USART2, "unable to register isr  \n\r", 25);
  }

  /* ---- TASK DEFINITION --------------------- */
  rtems_status_code status;
  rtems_name frame_task_name;
  frame_task_name = rtems_build_name('F', 'R', 'M', '1');

  /* ---- APPLICATION INITIALIZATION START ---- */

  status = rtems_task_create(frame_task_name, 1, RTEMS_MINIMUM_STACK_SIZE,
                             RTEMS_NO_PREEMPT, RTEMS_FLOATING_POINT,
                             &frame_handler_tid);

  if (status != RTEMS_SUCCESSFUL) {
    uart_write_buf(USART2, "task create failed............. \n\r", 34); //
    // XXX:
    exit(1);
  }

  status = rtems_task_start(frame_handler_tid, DCMI_frame_handler,
                            (rtems_task_argument)&DCMI_frame_task_arguments);

  if (status != RTEMS_SUCCESSFUL) {
    uart_write_buf(USART2, "task start failed.............. \n\r", 34); //
    // XXX:
    exit(1);
  }
  uart_write_buf(USART2, "task init continue............. \n\r", 34); // XXX:

  status = rtems_task_delete(RTEMS_SELF); /* should not /return */

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
