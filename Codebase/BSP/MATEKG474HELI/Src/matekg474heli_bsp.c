//
//  matekg474heli_bsp.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/19.
//

// Includes
#include "stm32g4xx_hal.h"

// Global variables
DMA_HandleTypeDef hdma_uart3_tx;
UART_HandleTypeDef huart3;

// Functions
static void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_StatusTypeDef status = HAL_OK;

  /* Configure the main internal regulator output voltage */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /* Initializes the RCC Oscillators according to the specified parameters
     in the RCC_OscInitTypeDef structure. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42U;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  status = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  assert_param(status == HAL_OK);

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  status = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  assert_param(status == HAL_OK);
}

static void SystemDMA_Config(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, SYSTICK_INT_PRIORITY - 1UL, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void BSP_Assert_Failed(uint8_t* file, uint32_t line) {
  __disable_irq();
  GPIOC->BSRR = (uint32_t) (GPIO_PIN_14 | GPIO_PIN_15);
  while (1);
}

void BSP_MCU_Init(void) {
  HAL_StatusTypeDef status = HAL_OK;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  status = HAL_Init();
  assert_param(status == HAL_OK);

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure DMA controller */
  SystemDMA_Config();
}

void BSP_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable clocks */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Set GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);

  /* Configure GPIO pins */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void BSP_UART_Init(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  HAL_StatusTypeDef status = HAL_OK;

  /* Select peripherals' clock source */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  status = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  assert_param(status == HAL_OK);

  /* Enable clocks */
  __HAL_RCC_USART3_CLK_ENABLE();

  /* Init UART3 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000U;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart3.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  status = HAL_UART_Init(&huart3);
  assert_param(status == HAL_OK);

  /* Enable UART3 interrupts */
  HAL_NVIC_SetPriority(USART3_IRQn, SYSTICK_INT_PRIORITY - 2UL, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}
