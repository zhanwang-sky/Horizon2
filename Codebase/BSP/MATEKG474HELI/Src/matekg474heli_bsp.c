//
//  matekg474heli_bsp.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/19.
//

// Includes
#include "stm32g4xx_hal.h"

// Global variables
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_uart1_tx;
DMA_HandleTypeDef hdma_uart2_tx;
DMA_HandleTypeDef hdma_uart3_tx;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
IWDG_HandleTypeDef hiwdg;

volatile uint32_t adc2_data[2];

// Functions
static void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_StatusTypeDef status = HAL_OK;

  /* Configure the main internal regulator output voltage */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /* Initializes the RCC Oscillators according to the specified parameters
     in the RCC_OscInitTypeDef structure. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  /* Enable clocks */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Enable DMA interrupts */
  /* DMA1_Channel2_IRQn */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, SYSTICK_INT_PRIORITY - 2UL, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, SYSTICK_INT_PRIORITY - 2UL, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel6_IRQn */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, SYSTICK_INT_PRIORITY - 2UL, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, SYSTICK_INT_PRIORITY - 3UL, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA1_Channel8_IRQn */
  HAL_NVIC_SetPriority(DMA1_Channel8_IRQn, SYSTICK_INT_PRIORITY - 3UL, 0U);
  HAL_NVIC_EnableIRQ(DMA1_Channel8_IRQn);
  /* DMA2_Channel1_IRQn */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, SYSTICK_INT_PRIORITY - 4UL, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel2_IRQn */
  HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, SYSTICK_INT_PRIORITY - 4UL, 0U);
  HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
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

void BSP_ADC_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  HAL_StatusTypeDef status = HAL_OK;

  /* Enable clocks */
  __HAL_RCC_ADC12_CLK_ENABLE();

  /* Init ADC1 */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0U;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1U;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  status = HAL_ADC_Init(&hadc1);
  assert_param(status == HAL_OK);
  /* Configure channels */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR_ADC1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0U;
  status = HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  assert_param(status == HAL_OK);
  /* Start calibration */
  status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  assert_param(status == HAL_OK);
  /* Start conversion */
  status = HAL_ADC_Start(&hadc1);
  assert_param(status == HAL_OK);

  /* Init ADC2 */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0U;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 2U;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  status = HAL_ADC_Init(&hadc2);
  assert_param(status == HAL_OK);
  /* Configure channels */
  /* Channel 1 */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0U;
  status = HAL_ADC_ConfigChannel(&hadc2, &sConfig);
  assert_param(status == HAL_OK);
  /* Channel 2 */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  status = HAL_ADC_ConfigChannel(&hadc2, &sConfig);
  assert_param(status == HAL_OK);
  /* Start calibration */
  status = HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  assert_param(status == HAL_OK);
  /* Start conversion */
  status = HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adc2_data, 2U);
  assert_param(status == HAL_OK);
}

void BSP_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable clocks */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure GPIO pins */
  /* Set GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  /* resource BEEPER 1 B09 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Set GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
  /* resource LED 1 C14 */
  /* resource LED 2 C15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void BSP_EXTI_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable clocks */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure EXTI pins */
  /* resource GYRO_EXTI 1 B07 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Enable EXTI interrupts */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, SYSTICK_INT_PRIORITY - 1UL, 0U);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void BSP_PWM_Init(void) {
  TIM_OC_InitTypeDef sConfigOC = {0};
  RCC_ClkInitTypeDef clkconfig = {0};
  uint32_t pFLatency = 0U;
  uint32_t uwTimclock = 0U;
  HAL_StatusTypeDef status = HAL_OK;

  /* Enable clocks */
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Compute TIM2/TIM3 clock */
  uwTimclock = HAL_RCC_GetPCLK1Freq();
  if (clkconfig.APB1CLKDivider != RCC_HCLK_DIV1) {
    uwTimclock <<= 1U;
  }

  /* Init TIM2 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (uwTimclock / 1000000U) - 1U; // 1 MHz counter clock (resolution: 1 us)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (1000000U / 50U) - 1U; // Period: 50 Hz
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  status = HAL_TIM_PWM_Init(&htim2);
  assert_param(status == HAL_OK);
  /* Configure channels */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  status = HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  assert_param(status == HAL_OK);
  status = HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
  assert_param(status == HAL_OK);
  /* Start output */
  status = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  assert_param(status == HAL_OK);
  status = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  assert_param(status == HAL_OK);

  /* Init TIM3 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = (uwTimclock / 1000000U) - 1U; // 1 MHz counter clock (resolution: 1 us)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = (1000000U / 50U) - 1U; // Period: 50 Hz
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  status = HAL_TIM_PWM_Init(&htim3);
  assert_param(status == HAL_OK);
  /* Configure channels */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  status = HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  assert_param(status == HAL_OK);
  status = HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  assert_param(status == HAL_OK);
  status = HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  assert_param(status == HAL_OK);
  status = HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
  assert_param(status == HAL_OK);
  /* Start output */
  status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  assert_param(status == HAL_OK);
  status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  assert_param(status == HAL_OK);
  status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  assert_param(status == HAL_OK);
  status = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  assert_param(status == HAL_OK);
}

void BSP_UART_Init(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  HAL_StatusTypeDef status = HAL_OK;

  /* Select peripherals' clock source */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 |
                                       RCC_PERIPHCLK_USART2 |
                                       RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  status = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  assert_param(status == HAL_OK);

  /* Enable clocks */
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();

  /* Init UART1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200U;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  status = HAL_UART_Init(&huart1);
  assert_param(status == HAL_OK);

  /* Init UART2 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200U;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  status = HAL_UART_Init(&huart2);
  assert_param(status == HAL_OK);

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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT | UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart3.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  status = HAL_UART_Init(&huart3);
  assert_param(status == HAL_OK);

  /* Enable UART1 interrupts */
  HAL_NVIC_SetPriority(USART1_IRQn, SYSTICK_INT_PRIORITY - 2UL, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* Enable UART2 interrupts */
  HAL_NVIC_SetPriority(USART2_IRQn, SYSTICK_INT_PRIORITY - 2UL, 0U);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* Enable UART3 interrupts */
  HAL_NVIC_SetPriority(USART3_IRQn, SYSTICK_INT_PRIORITY - 2UL, 0U);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void BSP_I2C_Init(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  HAL_StatusTypeDef status = HAL_OK;

  /* Select peripherals' clock source */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  status = HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  assert_param(status == HAL_OK);

  /* Enable clocks */
  __HAL_RCC_I2C1_CLK_ENABLE();

  /* Init I2C1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20B01E5FU;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  status = HAL_I2C_Init(&hi2c1);
  assert_param(status == HAL_OK);
  /* Configure analog filter */
  status = HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  assert_param(status == HAL_OK);
  /* Configure digital filter */
  status = HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0U);
  assert_param(status == HAL_OK);

  /* Enable I2C1 interrupts */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, SYSTICK_INT_PRIORITY - 3UL, 0U);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, SYSTICK_INT_PRIORITY - 3UL, 0U);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void BSP_SPI_Init(void) {
  HAL_StatusTypeDef status = HAL_OK;

  /* Enable clocks */
  __HAL_RCC_SPI1_CLK_ENABLE();

  /* Init SPI1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7U;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  status = HAL_SPI_Init(&hspi1);
  assert_param(status == HAL_OK);

  /* Enable SPI1 interrupts */
  HAL_NVIC_SetPriority(SPI1_IRQn, SYSTICK_INT_PRIORITY - 4UL, 0U);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
}

void BSP_WDOG_Init(void) {
  HAL_StatusTypeDef status = HAL_OK;

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4; // 8KHz
  hiwdg.Init.Reload = 2048U; // 256ms
  hiwdg.Init.Window = IWDG_WINR_WIN;
  status = HAL_IWDG_Init(&hiwdg);
  assert_param(status == HAL_OK);
}
