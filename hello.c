/*
 * Hello world example
 */
#include <rtems.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <stm32l4r9_module_mspi_mt29.h>

rtems_task Init(rtems_task_argument ignored) {
  volatile int testint = 0;
#ifdef __rtems__
  testint = 1;
#endif

  // ------------ QUADSPI INITIALIZATION  -------------------------------------
  mspi_init();
  /*TODO: the device structure should really be constructed by initialization
   * method */
  struct mspi_device mt29 = mspi_device_constr();
  //  --------------------------------------------------------------------------

  void mspi_dma_ch_init(void); // TODO: move to drivers
  void mspi_dmamux_cfg(void);
  mspi_dma_ch_init();
  mspi_dmamux_cfg();

  // Define testdata
  static uint32_t testdata[MT29_PAGE_SIZE];
  static uint32_t verificationdata[MT29_PAGE_SIZE];

  // patterning of testdata
  for (uint32_t i = 0; i < MT29_PAGE_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 2;
  test_n_addr.page = 0;
  test_n_addr.column = 0;

  // write and reaad data to the memory using the new driver
  // you need to construct the mspi interface object
  struct mspi_interface quadspi1 = {0}; // at the moment unused

  // disable write protection // does not do status verification
  mspi_transfer_dma(mt29.write_unlock, quadspi1, NULL);
  mspi_transfer_dma(mt29.write_enable, quadspi1, NULL);
  mspi_autopoll_wait(mt29.write_enable, quadspi1, NULL, WEL_MASK, WEL_MATCH);
  // TESTED UPTO HERE

  //
  mspi_transfer_dma(mt29.page_load_QUAD, quadspi1, &test_n_addr);
  mspi_transfer_dma(mt29.page_program, quadspi1, &test_n_addr);
  mspi_transfer_dma(mt29.page_read_from_nand, quadspi1, &test_n_addr);
  mspi_transfer_dma(mt29.page_read_from_cache_QUAD, quadspi1, &test_n_addr);

  volatile bool test_heart_beat = 1;
  time_t curr_time;
  curr_time = time(NULL);
  while (1) {
    curr_time = time(NULL);
  }
  exit(0);
}

/*
 * TODO:
 * Move to the bsp drivers
 *
 */

void mspi_dma_ch_init(void) {
  /* This driver functions declares the mt29 push and pull dma buffers
   * and configures the dma channels
   */
  static char dma_push_buffer[MT29_PAGE_SIZE];
  static char dma_pull_buffer[MT29_PAGE_SIZE];
  /*  TODO: protect the dma channels with a define guard */

#define OCTOSPI_DR_OFF 0x050
  /* Set up DMA1_CH1 "Push" dma channel */
  /* Set the peripheral register address in the DMA_CPARx register */
  /* TODO: set from the strcuture definition */
  DMA1_Channel1->CPAR = OCTOSPI1_R_BASE + OCTOSPI_DR_OFF;
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
  DMA1_Channel2->CPAR = OCTOSPI1_R_BASE + OCTOSPI_DR_OFF;
  /* Set the target memory address in the DMA_CMARx register */
  DMA1_Channel2->CMAR = (uint32_t)&dma_push_buffer;
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
  /*
   * Configures the DMAMUX for the push and pull dma channels
   * */
  /* DMAMUX OCTOSPI1 to Push dma channel */
  DMAMUX1_Channel1->CCR |= 40UL;
  /* DMAMUX OCTOSPI1 to Pull dma channel */
  DMAMUX1_Channel2->CCR |= 40UL;
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
