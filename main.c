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
  /* --- get octospi objects */
  struct mspi_interface octospi1 = {.interface_select = 0x01};
  struct mspi_device mt29 = mspi_device_constr();

  /* --- dcmi memory system initialization */
  struct jpeg_image image_storage_registry[MAX_N_STORABLE_FRAMES];

  /* read the memory and get context */
  get_image_storage_status(octospi1, mt29, image_storage_registry);
  u32 last_image_index = find_last_image_index(image_storage_registry);

  struct dcmi_isr_arg DCMI_frame_isr_arguments = {
      .image_storage_struct = image_storage_registry,
      .last_image_index = last_image_index};

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
