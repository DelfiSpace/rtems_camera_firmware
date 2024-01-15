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

/* ST includes <begin> */
#include "dcmi.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"

void SystemClock_Config(void);
/* ST includes <end> */

char dma_push_buffer[MT29_PAGE_SIZE];
char dma_pull_buffer[MT29_PAGE_SIZE];

rtems_task Init(rtems_task_argument ignored) {
  // ------------ USART   INITIALIZATION  -------------------------------------
  // clock_configure();
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DCMI_Init();
  MX_I2C1_Init();
  // ------------ USART   INITIALIZATION  -------------------------------------
  uart_init(UART2, 9600);

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

  octospi1.data_ptr = &verificationdata[0];
  octospi1.size_tr = MT29_PAGE_SIZE;

  // disable write protection // does not do status verification
  mspi_transfer(octospi1, mt29.write_unlock, NULL);
  mspi_transfer(octospi1, mt29.write_enable_polled, NULL);

  mspi_transfer(octospi1, mt29.page_load_SINGLE, &test_n_addr);
  // mspi_transfer_dma(octospi2, mt29.page_program, &test_n_addr);
  // mspi_transfer_dma(octospi2, mt29.page_read_from_nand, &test_n_addr);
  mspi_transfer(octospi1, mt29.page_read_from_cache_SINGLE, &test_n_addr);

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
