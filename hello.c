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
 *
 * create handler for writing dcma images
 * integrate a timer to measure... time betweeen events duh
 * fix loading the images in nand
 *
 * calibrate dcmi images
 * create a better neovim configuration with dap
 */

struct Node *hw_head = NULL;

void DCMI_IRQHandler(void); // XXX:

rtems_task Init(rtems_task_argument ignored) {
  // ------------ SYTSTEM INITIALIZATION  -------------------------------------

  // HAL_Init();
  hwlist_require(&hw_head, &debug_uart_init, NULL);
  hwlist_require(&hw_head, &dcmi_init, NULL);
  hwlist_require(&hw_head, &mspi_init, NULL);
  uart_write_buf(USART2, "play victory dance\n\r", 22); // XXX:

  //* configures interrupts */
  // sets handler pointer in vector table
  __NVIC_SetVector(DCMI_IRQn, (u32)DCMI_IRQHandler);
  // sets priority
  NVIC_SetPriority(DCMI_IRQn, 0x5C);
  // enable the DCMI interrupt
  NVIC_EnableIRQ(DCMI_IRQn);

  // enable dcmi vsync interrupt
  DCMI->IER |= DCMI_IER_FRAME_IE;

  // enable interrupts
  __enable_irq();

  /* set dcmi capture flag */
  DCMI->CR |= DCMI_CR_CAPTURE;

  while (1) {
    // uart_write_buf(USART2, "play victory dance\n\r", 22);
  }
  exit(0);
}
void DCMI_IRQHandler(void) {
  uart_write_buf(USART2, "received frame  \n\r", 20); // XXX:

  struct jpeg_image test_image;
  u32 *image_buffer = dcmi_get_buffer_ptr();
  dcmi_buffer_analisis(&test_image, image_buffer);

  /* clear at the end, to avoid overruns? or non issue due to priority?
   * what happens if they have the same priority?
   */
  /* you need a multiple interrupt handler in case more sources are enabled */
  DCMI->ICR &= ~(DCMI_ICR_FRAME_ISC_Msk);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
