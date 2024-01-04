/*
 * Hello world example
 */
#include <rtems.h>
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
  // struct mspi_device mt29 = mspi_device_constr();
  //  --------------------------------------------------------------------------

  // Define testdata
  /*
  uint32_t testdata[MT29_PAGE_SIZE / 4] = {0};
  uint32_t verificationdata[MT29_PAGE_SIZE] = {0};
  for (uint32_t i = 0; i < MT29_PAGE_SIZE / 4; i++) {
    testdata[i] = i;
  }
  */

  /*
  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 2;
  test_n_addr.page = 0;
  test_n_addr.column = 0;

  // write and reaad data to the memory using the new driver
  // you need to construct the mspi interface object
  struct mspi_interface quadspi1 = {0}; // at the moment unused

  // disable write protection
  mspi_transfer_dma(mt29.write_unlock, quadspi1,
                    NULL); // does not do status verification
  mspi_transfer_dma(mt29.write_enable, quadspi1, NULL);
  mspi_autopoll_wait(mt29.write_enable, quadspi1, NULL, WEL_MASK,
                     WEL_MATCH); // TESTED UPTO HERE

  //
  mspi_transfer_dma(mt29.page_load_QUAD, quadspi1, &test_n_addr);
  mspi_transfer_dma(mt29.page_program, quadspi1, &test_n_addr);
  mspi_transfer_dma(mt29.page_read_from_nand, quadspi1, &test_n_addr);
  mspi_transfer_dma(mt29.page_read_from_cache_QUAD, quadspi1, &test_n_addr);
  */

  volatile bool test_heart_beat = 1;
  time_t curr_time;
  curr_time = time(NULL);
  while (1) {
    curr_time = time(NULL);
  }
  exit(0);
}
