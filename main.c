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

struct Node *hw_head = NULL; // TODO: move to the bsp

rtems_task Init(rtems_task_argument ignored) {
  // ------------ SYTSTEM INITIALIZATION  -------------------------------------

  // HAL_Init();
  hwlist_require(&hw_head, &debug_uart_init, NULL);
  hwlist_require(&hw_head, &dcmi_init, NULL);
  hwlist_require(&hw_head, &mspi_init, NULL);
  uart_write_buf(USART2, "play victory dance\n\r", 22); // XXX:

/* ---- APPLICATION INITIALIZATION START ---- */

/* --- dcmi memory system initialization */
#define MAX_N_STORABLE_FRAMES 100000
  /* TODO: consider making static tradeoff, move init to a sensible place */
  struct jpeg_image image_storage_registry[MAX_N_STORABLE_FRAMES];
  imwrite_storagestatus_get(&image_storage_registry[0]); // rename ffs

  /* read the memory and get context */

  /* --- isr initialization */
  rtems_status_code ir_rs = {0};
  ir_rs = register_dcmi_frame_isr();

  /* enable dcmi vsync interrupt */
  DCMI->IER |= DCMI_IER_FRAME_IE;

  /* ---- APPLICATION INITIALIZATION START ---- */

  /* set dcmi capture flag */
  DCMI->CR |= DCMI_CR_CAPTURE;

  while (1) {
    // uart_write_buf(USART2, "play victory dance\n\r", 22);
  }
  exit(0);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
