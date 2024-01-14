/*
 * Hello world example
 */
#include <rtems.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32l4r9xx.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <hal_uart.h>
#include <stm32l4r9_module_clk_config.h>
#include <stm32l4r9_module_mspi_mt29.h>

/* This driver functions declares the mt29 push and pull dma buffers
 * and configures the dma channels
 */
/*
TODO: move to drivers
TODO: document the difference (in particular in placement in memory
and scope) of defining a statuc global vs global variable
TODO: Study and write down best practices for memory allocation
in multithreaded systems
WARNING: is it correct to have it as global,
is there a better way, since it is a buffer that in a
larger project may be accessed by several structs?
How can mutexes be used in this case?
*/
char dma_push_buffer[MT29_PAGE_SIZE];
char dma_pull_buffer[MT29_PAGE_SIZE];

rtems_task Init(rtems_task_argument ignored) {
  // ------------ USART   INITIALIZATION  -------------------------------------
  clock_configure();
  // ------------ USART   INITIALIZATION  -------------------------------------
  uart_init(UART2, 115200);

  // ------------ QUADSPI INITIALIZATION  -------------------------------------
  // construct the mspi interface object
  // TODO: define a constructor method
  struct mspi_interface octospi1 = {0};
  struct mspi_interface octospi2 = {0};
  octospi1.interface_select = 0x01;
  octospi2.interface_select = 0x02;

  mspi_init(octospi1);
  mspi_init(octospi2);
  /*TODO: the device structure should really be constructed by initialization
   * method */
  struct mspi_device mt29 = mspi_device_constr();
  //  --------------------------------------------------------------------------

  void mspi_dmamux_cfg(void);

  // mspi_dma_ch_init();
  mspi_dmamux_cfg();

  // Define testdata
  static uint32_t testdata[MT29_PAGE_W_SIZE];
  static uint32_t verificationdata[MT29_PAGE_W_SIZE];

  // patterning of testdata
  for (uint32_t i = 0; i < MT29_PAGE_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 2;  // up to 2047
  test_n_addr.page = 0;   // up to 63
  test_n_addr.column = 0; // up to 4351

  // copy data to the dma buffers
  if (sizeof(testdata) == sizeof(dma_push_buffer)) {
    memcpy(dma_push_buffer, testdata, sizeof(testdata));
  } else {
    // throw error
  }

  // disable write protection // does not do status verification
  mspi_transfer_dma(octospi2, mt29.write_unlock, NULL);
  mspi_transfer_dma(octospi2, mt29.write_enable, NULL);
  mspi_autopoll_wait(octospi2, mt29.write_enable, NULL, WEL_MASK, WEL_MATCH);

  mspi_transfer_dma(octospi2, mt29.page_load_SINGLE, &test_n_addr);
  // mspi_transfer_dma(octospi2, mt29.page_program, &test_n_addr);
  // mspi_transfer_dma(octospi2, mt29.page_read_from_nand, &test_n_addr);
  mspi_transfer_dma(octospi2, mt29.page_read_from_cache_SINGLE, &test_n_addr);

  // copy data to the dma buffers
  if (sizeof(verificationdata) == sizeof(dma_pull_buffer)) {
    memcpy(verificationdata, dma_pull_buffer, sizeof(dma_pull_buffer));
  } else {
    // throw error
  }

  /* OCTOSPI TEST SECTION
   */

  volatile bool test_heart_beat = 1;
  time_t curr_time;
  // curr_time = time(NULL);
  while (1) {
    // curr_time = time(NULL);
    uart_write_buf(USART2, "play victory dance\n\r", 22);
    // uart_write_byte(USART2, 0x01);
    spin(1000);
  }
  exit(0);
}

/*
 * TODO:
 * Move to the bsp drivers
 *
 */

void mspi_dma_ch_init(void) {
  /*  TODO: protect the dma channels with a define guard */

  /* Enable the dma controller 1 peripheral */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

#define OCTOSPI_DR_OFF 0x050
  /* Set up DMA1_CH1 "Push" dma channel */
  /* Set the peripheral register address in the DMA_CPARx register */
  /* TODO: set from the strcuture definition */
  DMA1_Channel1->CPAR = OCTOSPI2_R_BASE + OCTOSPI_DR_OFF;
  /* Set the target memory address in the DMA_CMARx register */
  DMA1_Channel1->CMAR = (uint32_t)&dma_push_buffer;
  /* Configure the total number of data transfers in the DMA_CNDTRx register
   */
  DMA1_Channel1->CNDTR = MT29_PAGE_SIZE >> 2;
  /* Configure the CCR register */
  DMA1_Channel1->CCR |=
      /* channel priority */
      0b00 << DMA_CCR_PL_Pos |
      /* channel direction (read from memory) */
      DMA_CCR_DIR |
      /* memory size (32bit)*/
      0b11 << DMA_CCR_MSIZE_Pos |
      /* peripheral size (32bit)*/
      0b11 << DMA_CCR_PSIZE_Pos |
      /* memory increment mode */
      DMA_CCR_MINC;

  /* Set up DMA1_CH2 "Pull" dma channel */
  /* Set the peripheral register address in the DMA_CPARx register */
  /* TODO: set from the strcuture definition */
  DMA1_Channel2->CPAR = OCTOSPI2_R_BASE + OCTOSPI_DR_OFF;
  /* Set the target memory address in the DMA_CMARx register */
  DMA1_Channel2->CMAR = (uint32_t)&dma_pull_buffer;
  /* Configure the total number of data transfers in the DMA_CNDTRx register
   */
  DMA1_Channel2->CNDTR = MT29_PAGE_SIZE >> 2;
  /* Configure the CCR register */
  DMA1_Channel2->CCR |=
      /* channel priority */
      0b00 << DMA_CCR_PL_Pos |
      /* memory size (32bit)*/
      0b11 << DMA_CCR_MSIZE_Pos |
      /* peripheral size (32bit)*/
      0b11 << DMA_CCR_PSIZE_Pos |
      /* memory increment mode */
      DMA_CCR_MINC;

  /*Configure CCR register:channel direction (read from peripheral) */
  DMA1_Channel2->CCR &= ~(DMA_CCR_DIR);
}

void mspi_dmamux_cfg(void) {
  /*Enable DMAMUX clock
   */
  RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;

  /*
   * Configures the DMAMUX for the push and pull dma channels
   * */
  /* DMAMUX OCTOSPI1 to Push dma channel */
  DMAMUX1_Channel0->CCR |= 40UL;
  /* DMAMUX OCTOSPI1 to Pull dma channel */
  DMAMUX1_Channel1->CCR |= 40UL;
}

/*
 * Generate something that you can call from the mspi_transfer_dma
 * to enable the channel
 * if send do something_send
 * if receive do something_receive
 *
 * as a function of the passed device structure
 * that is used as an identifier...
 * you really need to document this stuff
 * even sketches i don't care
 */
