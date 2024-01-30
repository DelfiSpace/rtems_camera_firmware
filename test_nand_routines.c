#include "test_nand_routines.h"
#include <stm32l4r9_module_mspi_mt29.h>

static uint32_t testdata[MT29_PAGE_W_SIZE];
static uint32_t verificationdata[MT29_PAGE_W_SIZE];

void nand_test_routine(void) {

  test_case_1();
  test_case_2();
  // test_case_3();
  // test_case_4();
  // test_case_5();
  // test_case_6();
  test_case_7();
  test_case_8();
}

/* test case 1:
 * write a test buffer to CACHE for OCTOSPI1:1 in SINGLE mode
 * read to a verification buffer to CACHE for OCTOSPI1:1 in SINGLE mode
 *
 * Test success criteria:
 * verification == test buffer
 */
void test_case_1(void) {
  memset(verificationdata, 0x0, sizeof(verificationdata));
  /*test initialization*/
  struct mspi_interface octospi1 = {0};
  octospi1.interface_select = 0x01;

  struct mspi_device mt29 = mspi_device_constr();
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 0;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)

  /* test operations */
  mspi_transfer(octospi1, mt29.write_unlock, NULL);
  mspi_transfer(octospi1, mt29.write_enable, NULL);

  uint32_t octospi_status = 0;
  octospi1.data_ptr = testdata;
  mspi_transfer(octospi1, mt29.page_load_SINGLE, &test_n_addr);

  octospi1.data_ptr = verificationdata;
  mspi_transfer(octospi1, mt29.page_read_from_cache_SINGLE, &test_n_addr);

  /* test result analysis */
  uart_write_buf(USART2, "--- NAND TEST CASE 1 --- \n\r", 33);
  test_data_print();
  check_test();
}

/* test case 2:
 * write a test buffer to CACHE for OCTOSPI1:2 in SINGLE mode
 * read to a verification buffer to CACHE for OCTOSPI1:2 in SINGLE mode
 *
 * Test success criteria:
 * verification == test buffer
 */
void test_case_2(void) {
  memset(verificationdata, 0x0, sizeof(verificationdata));
  /*test initialization*/
  struct mspi_interface octospi1 = {0};
  octospi1.interface_select = 0x01;
  octospi1.flash_select = 0x01;

  struct mspi_device mt29 = mspi_device_constr();
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 0;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)

  /* test operations */
  mspi_transfer(octospi1, mt29.write_unlock, NULL);
  mspi_transfer(octospi1, mt29.write_enable, NULL);

  uint32_t octospi_status = 0;
  octospi1.data_ptr = testdata;
  mspi_transfer(octospi1, mt29.page_load_SINGLE, &test_n_addr);

  octospi1.data_ptr = verificationdata;
  mspi_transfer(octospi1, mt29.page_read_from_cache_SINGLE, &test_n_addr);

  /* test result analysis */
  uart_write_buf(USART2, "--- NAND TEST CASE 2 --- \n\r", 33);
  test_data_print();
  check_test();
}

/* test case 3:
 * write a test buffer to CACHE for OCTOSPI1:1 in QUAD mode
 * read to a verification buffer to CACHE for OCTOSPI1:1 in QUAD mode
 *
 * Test success criteria:
 * verification == test buffer
 */
void test_case_3(void) {
  memset(verificationdata, 0x0, sizeof(verificationdata));
  /*test initialization*/
  struct mspi_interface octospi1 = {0};
  octospi1.interface_select = 0x01;

  struct mspi_device mt29 = mspi_device_constr();
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 0;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)

  /* test operations */
  mspi_transfer(octospi1, mt29.write_unlock, NULL);
  mspi_transfer(octospi1, mt29.write_enable, NULL);

  uint32_t octospi_status = 0;
  octospi1.data_ptr = testdata;
  mspi_transfer(octospi1, mt29.page_load_QUAD, &test_n_addr);

  octospi1.data_ptr = verificationdata;
  mspi_transfer(octospi1, mt29.page_read_from_cache_QUAD, &test_n_addr);

  /* test result analysis */
  uart_write_buf(USART2, "--- NAND TEST CASE 3 --- \n\r", 33);
  test_data_print();
  check_test();
}

/* test case 4:
 * write a test buffer to CACHE for OCTOSPI1:2 in QUAD mode
 * read to a verification buffer to CACHE for OCTOSPI1:1 in QUAD mode
 *
 * Test success criteria:
 * verification == test buffer
 */
void test_case_4(void) {
  memset(verificationdata, 0x0, sizeof(verificationdata));
  /*test initialization*/
  struct mspi_interface octospi1 = {0};
  octospi1.interface_select = 0x01;
  octospi1.flash_select = 0x02;

  struct mspi_device mt29 = mspi_device_constr();
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 0;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)

  /* test operations */
  mspi_transfer(octospi1, mt29.write_unlock, NULL);
  mspi_transfer(octospi1, mt29.write_enable, NULL);

  uint32_t octospi_status = 0;
  octospi1.data_ptr = testdata;
  mspi_transfer(octospi1, mt29.page_load_QUAD, &test_n_addr);

  octospi1.data_ptr = verificationdata;
  mspi_transfer(octospi1, mt29.page_read_from_cache_QUAD, &test_n_addr);

  /* test result analysis */
  uart_write_buf(USART2, "--- NAND TEST CASE 4 --- \n\r", 33);
  test_data_print();
  check_test();
}
/* test case 5:
 * program a test buffer to NAND for OCTOSPI1:1 in QUAD mode
 * read to a verification buffer to CACHE for OCTOSPI1:1 in QUAD mode
 *
 * Test success criteria:
 * verification == test buffer
 */
void test_case_5(void) {
  memset(verificationdata, 0x0, sizeof(verificationdata));
  /*test initialization*/
  struct mspi_interface octospi1 = {0};
  octospi1.interface_select = 0x01;

  struct mspi_device mt29 = mspi_device_constr();
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 2;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)
  /* page 0 of arbitrary block is ok */
  /* page 0 of block 0 is NOT OK */
  /* page 1 of block 0 is NOT OK */
  /* page 2 of block 0 is ok */
  /* page 1 of block 2047 is ok */

  /* test operations */
  mspi_transfer(octospi1, mt29.write_unlock, NULL);
  mspi_transfer(octospi1, mt29.ecc_enable, NULL);
  /* NOTE: ecc can be left enabled */

  mspi_transfer(octospi1, mt29.write_enable, NULL);

#define status_size 1
  uint32_t octospi_status[status_size] = {0};
  // get protection status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory write protection status: ", 32);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_unlock_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
  }
  uart_write_buf(USART2, "\n\r", 2);
  // -----------------------------------------------------
  // get configuration status ----------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory configuration status: ", 29);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_config_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
  }
  uart_write_buf(USART2, "\n\r", 2);
  // -----------------------------------------------------

  // XXX:
  octospi1.data_ptr = testdata;
  mspi_transfer(octospi1, mt29.page_load_SINGLE, &test_n_addr);

  // get program status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory status: ", 15);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
    uart_write_buf(USART2, "\n\r", 2);
  }
  // -----------------------------------------------------

  // XXX:
  mspi_transfer(octospi1, mt29.page_program, &test_n_addr);

  // get program status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory status after program: ", 29);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
    uart_write_buf(USART2, "\n\r", 2);
  }
  // -----------------------------------------------------

  // XXX:
  mspi_transfer(octospi1, mt29.wait_oip, &test_n_addr);
  mspi_transfer(octospi1, mt29.page_read_from_nand, &test_n_addr);

  // get program status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory status after read: ", 26);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
    uart_write_buf(USART2, "\n\r", 2);
  }
  // -----------------------------------------------------

  // XXX:
  mspi_transfer(octospi1, mt29.wait_oip, &test_n_addr);
  octospi1.data_ptr = verificationdata;
  memset(verificationdata, 0x0, sizeof(verificationdata));
  mspi_transfer(octospi1, mt29.page_read_from_cache_SINGLE, &test_n_addr);

  /* test result analysis */
  uart_write_buf(USART2, "--- NAND TEST CASE 5 --- \n\r", 33);
  test_data_print();
  check_test();
}

/* test case 6:
 * program a test buffer to NAND for OCTOSPI1:2 in QUAD mode
 * read to a verification buffer to CACHE for OCTOSPI1:1 in QUAD mode
 *
 * Test success criteria:
 * verification == test buffer
 */
void test_case_6(void) {
  memset(verificationdata, 0x0, sizeof(verificationdata));
  /*test initialization*/
  struct mspi_interface octospi1 = {0};
  octospi1.interface_select = 0x01;
  octospi1.flash_select = 0x01;

  struct mspi_device mt29 = mspi_device_constr();
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 2;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)
  /* page 0 of arbitrary block is ok */
  /* page 0 of block 0 is NOT OK */
  /* page 1 of block 0 is NOT OK */
  /* page 2 of block 0 is ok */
  /* page 1 of block 2047 is ok */

  /* test operations */
  mspi_transfer(octospi1, mt29.write_unlock, NULL);
  mspi_transfer(octospi1, mt29.ecc_enable, NULL);
  /* NOTE: ecc can be left enabled */

  mspi_transfer(octospi1, mt29.write_enable, NULL);

#define status_size 1
  uint32_t octospi_status[status_size] = {0};
  // get protection status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory write protection status: ", 32);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_unlock_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
  }
  uart_write_buf(USART2, "\n\r", 2);
  // -----------------------------------------------------
  // get configuration status ----------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory configuration status: ", 29);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_config_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
  }
  uart_write_buf(USART2, "\n\r", 2);
  // -----------------------------------------------------

  // XXX:
  octospi1.data_ptr = testdata;
  mspi_transfer(octospi1, mt29.page_load_SINGLE, &test_n_addr);

  // get program status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory status: ", 15);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
    uart_write_buf(USART2, "\n\r", 2);
  }
  // -----------------------------------------------------

  // XXX:
  mspi_transfer(octospi1, mt29.page_program, &test_n_addr);

  // get program status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory status after program: ", 29);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
    uart_write_buf(USART2, "\n\r", 2);
  }
  // -----------------------------------------------------

  // XXX:
  mspi_transfer(octospi1, mt29.wait_oip, &test_n_addr);
  mspi_transfer(octospi1, mt29.page_read_from_nand, &test_n_addr);

  // get program status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory status after read: ", 26);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
    uart_write_buf(USART2, "\n\r", 2);
  }
  // -----------------------------------------------------

  // XXX:
  mspi_transfer(octospi1, mt29.wait_oip, &test_n_addr);
  octospi1.data_ptr = verificationdata;
  memset(verificationdata, 0x0, sizeof(verificationdata));
  mspi_transfer(octospi1, mt29.page_read_from_cache_SINGLE, &test_n_addr);

  /* test result analysis */
  uart_write_buf(USART2, "--- NAND TEST CASE 5 --- \n\r", 33);
  test_data_print();
  check_test();
}

/* test case 7:
 * write a test buffer to CACHE for OCTOSPI1 DUALMEM in SINGLE mode
 * read to a verification buffer to CACHE for OCTOSPI1 DUALMEM in SINGLE mode
 *
 * Test success criteria:
 * verification == test buffer
 */
void test_case_7(void) {
  memset(verificationdata, 0x0, sizeof(verificationdata));
  /*test initialization*/
  struct mspi_interface octospi1 = {0};
  octospi1.interface_select = 0x01;
  octospi1.is_double_mem = 0x01;

  struct mspi_device mt29 = mspi_device_constr();
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 0;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)

  /* test operations */
  mspi_transfer(octospi1, mt29.write_unlock, NULL);
  mspi_transfer(octospi1, mt29.write_enable, NULL);

  uint32_t octospi_status = 0;
  octospi1.data_ptr = testdata;
  mspi_transfer(octospi1, mt29.page_load_SINGLE, &test_n_addr);

  octospi1.data_ptr = verificationdata;
  mspi_transfer(octospi1, mt29.page_read_from_cache_SINGLE, &test_n_addr);

  /* test result analysis */
  uart_write_buf(USART2, "--- NAND TEST CASE 7 --- \n\r", 33);
  test_data_print();
  check_test();
}

/* test case 8:
 * program a test buffer to NAND for OCTOSPI1 DUALMEM in QUAD mode
 * read to a verification buffer to CACHE for OCTOSPI1 DUALMEM in QUAD mode
 *
 * Test success criteria:
 * verification == test buffer
 */
void test_case_8(void) {
  memset(verificationdata, 0x0, sizeof(verificationdata));
  /*test initialization*/
  struct mspi_interface octospi1 = {0};
  octospi1.interface_select = 0x01;
  octospi1.is_double_mem = 0x01;

  struct mspi_device mt29 = mspi_device_constr();
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 2;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)
  /* page 0 of arbitrary block is ok */
  /* page 0 of block 0 is NOT OK */
  /* page 1 of block 0 is NOT OK */
  /* page 2 of block 0 is ok */
  /* page 1 of block 2047 is ok */

  /* test operations */
  mspi_transfer(octospi1, mt29.write_unlock, NULL);
  mspi_transfer(octospi1, mt29.ecc_enable, NULL);
  /* NOTE: ecc can be left enabled */

  mspi_transfer(octospi1, mt29.write_enable, NULL);

#define status_size 1
  uint32_t octospi_status[status_size] = {0};
  // get protection status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory write protection status: ", 32);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_unlock_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
  }
  uart_write_buf(USART2, "\n\r", 2);
  // -----------------------------------------------------
  // get configuration status ----------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory configuration status: ", 29);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_config_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
  }
  uart_write_buf(USART2, "\n\r", 2);
  // -----------------------------------------------------

  // XXX:
  octospi1.data_ptr = testdata;
  mspi_transfer(octospi1, mt29.page_load_QUAD, &test_n_addr);

  // get program status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory status: ", 15);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
    uart_write_buf(USART2, "\n\r", 2);
  }
  // -----------------------------------------------------

  // XXX:
  mspi_transfer(octospi1, mt29.page_program, &test_n_addr);

  // get program status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory status after program: ", 29);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
    uart_write_buf(USART2, "\n\r", 2);
  }
  // -----------------------------------------------------

  // XXX:
  mspi_transfer(octospi1, mt29.wait_oip, &test_n_addr);
  mspi_transfer(octospi1, mt29.page_read_from_nand, &test_n_addr);

  // get program status ------------------------------------
  memset(octospi_status, 0x0, status_size);

  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "Memory status after read: ", 26);

  octospi1.data_ptr = octospi_status;
  mspi_transfer(octospi1, mt29.get_status, NULL);

  for (int i = 0; i < status_size; i++) {
    char temp_str[20];
    int n;
    n = sprintf(temp_str, "%x ,", octospi_status[i]);
    uart_write_buf(USART2, temp_str, n);
    uart_write_buf(USART2, "\n\r", 2);
  }
  // -----------------------------------------------------

  // XXX:
  mspi_transfer(octospi1, mt29.wait_oip, &test_n_addr);
  octospi1.data_ptr = verificationdata;
  memset(verificationdata, 0x0, sizeof(verificationdata));
  mspi_transfer(octospi1, mt29.page_read_from_cache_QUAD, &test_n_addr);

  /* test result analysis */
  uart_write_buf(USART2, "--- NAND TEST CASE 8 --- \n\r", 33);
  test_data_print();
  check_test();
}

void test_data_print(void) {
  uart_write_buf(USART2, "TEST DATA: \n\r", 13);
  for (int i = 0; i < MT29_PAGE_W_SIZE; i++) {
    char temp_str[5];
    int n;
    n = sprintf(temp_str, "%x ,", testdata[i]);
    uart_write_buf(USART2, temp_str, n);
  }
  uart_write_buf(USART2, "\n\r", 2);

  uart_write_buf(USART2, "VERIFICATION DATA: \n\r", 21);
  for (int i = 0; i < MT29_PAGE_W_SIZE; i++) {
    char temp_str[5];
    int n;
    n = sprintf(temp_str, "%x ,", verificationdata[i]);
    uart_write_buf(USART2, temp_str, n);
  }
  uart_write_buf(USART2, "\n\r", 2);
}

void check_test(void) {
  u32 test_result = {0};
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    test_result |= (testdata[i] - verificationdata[i]);
  }
  if (test_result) {
    uart_write_buf(USART2, "NAND TEST CASE FAILURE \n\r", 27);
  } else {
    uart_write_buf(USART2, "NAND TEST CASE SUCCESS \n\r", 27);
  }
  uart_write_buf(USART2, "\n\r", 2);
  uart_write_buf(USART2, "\n\r", 2);
}
