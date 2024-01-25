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
#include <stm32l4r9_module_dcmi.h>
#include <stm32l4r9_module_mspi.h>
#include <stm32l4r9_module_mspi_mt29.h>

/* ST includes <begin> */
#include "gpio.h"
#include "i2c.h"
#include "main.h"

void SystemClock_Config(void);
/* ST includes <end> */

static uint32_t testdata[MT29_PAGE_W_SIZE];
static uint32_t verificationdata[MT29_PAGE_W_SIZE];

/* global dmamux debug symbols */
static u32 *sym_dmamux_channel1 = (u32 *)DMAMUX1_Channel1_BASE;

rtems_task Init(rtems_task_argument ignored) {
  // ------------ SYTSTEM INITIALIZATION  -------------------------------------
  HAL_Init();

  // XXX: make a general dma init. Init is useful when you have more sw elements
  // that need to configure a peripheral, so that you have a place to safely
  // reset
  // DMA controller reset
  RCC->AHB1RSTR |= RCC_AHB1RSTR_DMA1RST;
  RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_DMA1RST);

  if ((RCC->CFGR & RCC_CFGR_SWS_Msk) != 0b11 << RCC_CFGR_SWS_Pos) {
    SystemClock_Config();
  }
  MX_GPIO_Init();
  MX_I2C1_Init();

  enable_debug_clock();

  uart_init(UART2, 9600);

  ov5640_configure_jpeg_720p();
  // ov5640_configure_color_square();
  u32 *dcmi_dma_buffer = dcmi_cfg_transfer();

  // spin(120000000);
  // struct jpeg_image img = {0};
  // dcmi_buffer_analisis(&img, dcmi_dma_buffer);

  /* ---- TESTING NANDS -------------------------- */
  // Define testdata

  struct mspi_interface octospi1 = {0};
  octospi1.interface_select = 0x01;
  octospi1.data_ptr = &testdata[0];
  // XXX: But actually what elements of context are used? remove the others

  mspi_init(octospi1);

  struct mspi_device mt29 = mspi_device_constr();

  /* --- */

  // patterning of testdata
  for (uint32_t i = 0; i < MT29_PAGE_W_SIZE; i++) {
    testdata[i] = i;
  }

  struct nand_addr test_n_addr = {0};
  test_n_addr.block = 0;  // up to 2047
  test_n_addr.page = 0;   // up to 63
  test_n_addr.column = 0; // up to 4351 (4096)

  // disable write protection // does not do status verification
  mspi_transfer(octospi1, mt29.write_unlock, NULL); // NOTE: ok
  mspi_transfer(octospi1, mt29.write_enable, NULL); // NOTE: ok
  // XXX: The fact that the execution flow proceeded, means that
  // the check for the mask of is faulty.. since it is not possible
  // for the memories to reply with the wrong instruction.
  // XXX: The check was disabled

  /*
  uint32_t octospi_status = 0;
  mspi_transfer(octospi1, mt29.page_load_QUAD, &test_n_addr);

  mspi_transfer(octospi1, mt29.page_program, &test_n_addr);
  mspi_transfer(octospi1, mt29.get_status, &octospi_status);

  mspi_transfer(octospi1, mt29.wait_oip, &test_n_addr);

  octospi1.data_ptr = &verificationdata[0]; // emty the page cache to test read
  mspi_transfer(octospi1, mt29.page_load_SINGLE, &test_n_addr);
  */

  mspi_transfer(octospi1, mt29.page_read_from_nand, &test_n_addr);
  mspi_transfer(octospi1, mt29.wait_oip, &test_n_addr);

  octospi1.data_ptr = &verificationdata[0];
  mspi_transfer(octospi1, mt29.page_read_from_cache_QUAD, &test_n_addr);
  /* --------------------------------------------- */

  while (1) {
    uart_write_buf(USART2, "qlay victory dance\n\r", 22);
    spin(1000);
  }
  exit(0);
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) !=
      HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
