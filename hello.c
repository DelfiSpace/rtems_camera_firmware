/*
 * Hello world example
 */
// TODO: Include the following headers into asystem header that you can include
// only once.
//
#include <rtems.h>
#include <rtems/irq.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32l4r9xx.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <hwlist_handlers.h>
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

// Prototype for the ISR
rtems_isr DCMI_frame_isr_handler(rtems_vector_number vector); // XXX:

// Prototype
rtems_status_code register_dcmi_frame_isr(void);

rtems_task Init(rtems_task_argument ignored) {
  // ------------ SYTSTEM INITIALIZATION  -------------------------------------

  // HAL_Init();
  hwlist_require(&hw_head, &debug_uart_init, NULL);
  hwlist_require(&hw_head, &dcmi_init, NULL);
  hwlist_require(&hw_head, &mspi_init, NULL);
  uart_write_buf(USART2, "play victory dance\n\r", 22); // XXX:

  rtems_status_code ir_rs = {0};
  ir_rs = register_dcmi_frame_isr();
  // enable interrupts
  //__enable_irq();

  /* enable dcmi vsync interrupt */
  DCMI->IER |= DCMI_IER_FRAME_IE;

  /* set dcmi capture flag */
  DCMI->CR |= DCMI_CR_CAPTURE;

  while (1) {
    // uart_write_buf(USART2, "play victory dance\n\r", 22);
  }
  exit(0);
}

// Register the interrupt
rtems_status_code register_dcmi_frame_isr(void) {
  rtems_status_code status;

  // Define the interrupt vector
  rtems_vector_number dcmi_interrupt_vector_n = DCMI_IRQn;

  // Register the ISR
  status =
      rtems_interrupt_handler_install(dcmi_interrupt_vector_n, // Vector number
                                      "DCMI_frame", // Name for the ISR
                                      RTEMS_INTERRUPT_UNIQUE, // Flags
                                      DCMI_frame_isr_handler, // ISR
                                      NULL                    // ISR argument
      );
  return status;
}

// Implementation of the ISR
rtems_isr DCMI_frame_isr_handler(rtems_vector_number vector) {
  uart_write_buf(USART2, "rtems interrupt received frame  \n\r", 34); // XXX:
  DCMI->ICR &= ~(DCMI_ICR_FRAME_ISC_Msk);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
