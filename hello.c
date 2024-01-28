/*
 * Hello world example
 */
// TODO: Include the following headers into asystem header that you can include
// only once.
//
#include <hwlist_handlers.h>
#include <rtems.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32l4r9xx.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <stm32l4r9_module_clk_config.h>
#include <stm32l4r9_module_dcmi.h>
#include <stm32l4r9_module_i2c.h>
#include <stm32l4r9_module_mspi.h>
#include <stm32l4r9_module_mspi_mt29.h>
#include <stm32l4r9_module_uart.h>

/* ST includes <begin> */
#include "gpio.h"
#include "i2c.h"
#include "main.h"

#include "hwlist_agent.h"

/* TODO TABLE:
 * write initialization activation agent
 * add uart to initialization system (and create module)
 * test uart and init system working
 * add multispi to init system (no dma)
 * add dcmi to init system
 * add interrupt register and interrupt to dcmi system
 * test interrupt dcmi transfer
 * create handler for writing dcma images
 * integrate a timer to measure... time betweeen events duh
 *
 * calibrate dcmi images
 * create a better neovim configuration with dap
 */

struct Node *hw_head = NULL;

rtems_task Init(rtems_task_argument ignored) {
  // ------------ SYTSTEM INITIALIZATION  -------------------------------------

  hwlist_require(&hw_head, &debug_uart_init, NULL);
  hwlist_require(&hw_head, &dcmi_init, NULL);

  /* set dcmi capture flag */
  DCMI->CR |= DCMI_CR_CAPTURE;

  spin(12000000);

  struct jpeg_image test_image;

  u32 *image_buffer = dcmi_get_buffer();
  dcmi_buffer_analisis(&test_image, image_buffer);

  while (1) {
    // uart_write_buf(USART2, "play victory dance\n\r", 22);
  }
  exit(0);
}

void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
