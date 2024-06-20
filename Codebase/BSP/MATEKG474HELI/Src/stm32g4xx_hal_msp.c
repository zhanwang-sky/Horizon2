/**
  ******************************************************************************
  * @file   stm32g4xx_hal_msp.c
  * @author MCD Application Team
  * @brief  HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ----------------------------------------------------------------- */
#include "stm32g4xx_hal.h"

/* Global variables --------------------------------------------------------- */
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_uart1_tx;
extern DMA_HandleTypeDef hdma_uart2_tx;
extern DMA_HandleTypeDef hdma_uart3_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

/**
  * @brief  Initialize the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void) {
  /* Enable RCC clock */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  /* Enable GPIOF in order to use external crystal */
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral */
  HAL_PWREx_DisableUCPDDeadBattery();
}

/**
  * @brief  Initialize the ADC MSP.
  * @param  hadc ADC handle
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_StatusTypeDef status = HAL_OK;

  if (hadc == &hadc2) {
    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO pins */
    /* resource ADC_BATT 1 A04 */
    /* resource ADC_CURR 1 A05 */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    hdma_adc2.Instance = DMA2_Channel8;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    status = HAL_DMA_Init(&hdma_adc2);
    assert_param(status == HAL_OK);
    /* link peripheral to DMA channel */
    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);
  }
}

/**
  * @brief  Initializes the TIM PWM MSP.
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim == &htim2) {
    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO pins */
    /* resource MOTOR 1 A01 */
    /* resource MOTOR 2 A00 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  } else if (htim == &htim3) {
    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pins */
    /* resource SERVO 3 A07 */
    /* resource SERVO 4 A06 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* resource SERVO 1 B01 */
    /* resource SERVO 2 B00 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/**
  * @brief  Initialize the UART MSP.
  * @param  huart UART handle
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_StatusTypeDef status = HAL_OK;

  if (huart == &huart1) {
    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO pins */
    /* resource SERIAL_TX 1 A09 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* resource SERIAL_RX 1 A10 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART1 DMA Init */
    /* UART1_TX */
    hdma_uart1_tx.Instance = DMA1_Channel6;
    hdma_uart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_uart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart1_tx.Init.Mode = DMA_NORMAL;
    hdma_uart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    status = HAL_DMA_Init(&hdma_uart1_tx);
    assert_param(status == HAL_OK);
    /* link peripheral to DMA channel */
    __HAL_LINKDMA(huart, hdmatx, hdma_uart1_tx);
  } else if (huart == &huart2) {
    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO pins */
    /* resource SERIAL_TX 2 A02 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    /* resource SERIAL_RX 2 A03 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART2 DMA Init */
    /* UART2_TX */
    hdma_uart2_tx.Instance = DMA1_Channel4;
    hdma_uart2_tx.Init.Request = DMA_REQUEST_USART2_TX;
    hdma_uart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart2_tx.Init.Mode = DMA_NORMAL;
    hdma_uart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    status = HAL_DMA_Init(&hdma_uart2_tx);
    assert_param(status == HAL_OK);
    /* link peripheral to DMA channel */
    __HAL_LINKDMA(huart, hdmatx, hdma_uart2_tx);
  } else if (huart == &huart3) {
    /* Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pins */
    /* resource SERIAL_TX 3 B10 */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* resource SERIAL_RX 3 B11 */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* UART3 DMA Init */
    /* UART3_TX */
    hdma_uart3_tx.Instance = DMA1_Channel2;
    hdma_uart3_tx.Init.Request = DMA_REQUEST_USART3_TX;
    hdma_uart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_uart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart3_tx.Init.Mode = DMA_NORMAL;
    hdma_uart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    status = HAL_DMA_Init(&hdma_uart3_tx);
    assert_param(status == HAL_OK);
    /* link peripheral to DMA channel */
    __HAL_LINKDMA(huart, hdmatx, hdma_uart3_tx);
  }
}

/**
  * @brief  Initialize the I2C MSP.
  * @param  hi2c I2C handle
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_StatusTypeDef status = HAL_OK;

  if (hi2c == &hi2c1) {
    /* Enable GPIO clock */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure GPIO pins */
    /* resource I2C_SCL 1 A13 */
    /* resource I2C_SDA 1 A14 */
    GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* I2C1 DMA Init */
    /* I2C1_RX */
    hdma_i2c1_rx.Instance = DMA1_Channel7;
    hdma_i2c1_rx.Init.Request = DMA_REQUEST_I2C1_RX;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    status = HAL_DMA_Init(&hdma_i2c1_rx);
    assert_param(status == HAL_OK);
    /* link peripheral to DMA channel */
    __HAL_LINKDMA(hi2c, hdmarx, hdma_i2c1_rx);
    /* I2C1_TX */
    hdma_i2c1_tx.Instance = DMA1_Channel8;
    hdma_i2c1_tx.Init.Request = DMA_REQUEST_I2C1_TX;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    status = HAL_DMA_Init(&hdma_i2c1_tx);
    assert_param(status == HAL_OK);
    /* link peripheral to DMA channel */
    __HAL_LINKDMA(hi2c, hdmatx, hdma_i2c1_tx);
  }
}

/**
  * @brief  Initialize the SPI MSP.
  * @param  hspi SPI handle
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_StatusTypeDef status = HAL_OK;

  if (hspi == &hspi1) {
    /* Enable GPIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pins */
    /* resource SPI_SCK 1 B03 */
    /* resource SPI_MISO 1 B04 */
    /* resource SPI_MOSI 1 B05 */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /* Set GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    /* resource GYRO_CS 1 B06 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_TX */
    hdma_spi1_tx.Instance = DMA2_Channel1;
    hdma_spi1_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    status = HAL_DMA_Init(&hdma_spi1_tx);
    assert_param(status == HAL_OK);
    /* link peripheral to DMA channel */
    __HAL_LINKDMA(hspi, hdmatx, hdma_spi1_tx);
    /* SPI1_RX */
    hdma_spi1_rx.Instance = DMA2_Channel2;
    hdma_spi1_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    status = HAL_DMA_Init(&hdma_spi1_rx);
    assert_param(status == HAL_OK);
    /* link peripheral to DMA channel */
    __HAL_LINKDMA(hspi, hdmarx, hdma_spi1_rx);
  }
}
