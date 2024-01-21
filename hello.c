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
#include <stm32l4r9_module_mspi_mt29.h>

/* ST includes <begin> */
#include "gpio.h"
#include "i2c.h"
#include "main.h"

void SystemClock_Config(void);
/* ST includes <end> */

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
  // MX_DCMI_Init();
  MX_I2C1_Init();

  enable_debug_clock();

  uart_init(UART2, 9600);

  ov5640_configure_jpeg_qsxga();
  dcmi_cfg_transfer();
  dcmi_cfg_periph();

  while (1) {
    DCMI->CR |= DCMI_CR_CAPTURE;
    uart_write_buf(USART2, "qlay victory dance\n\r", 22);
    spin(1000);
  }
  exit(0);
}

void mspi_dmamux_cfg(void) {
  /*Enable DMAMUX clock */
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
