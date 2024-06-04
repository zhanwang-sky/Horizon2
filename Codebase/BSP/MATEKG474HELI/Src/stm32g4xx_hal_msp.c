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
extern DMA_HandleTypeDef hdma_uart2_tx;
extern DMA_HandleTypeDef hdma_uart3_tx;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

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
  * @brief  Initializes the TIM PWM MSP.
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim == &htim3) {
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

  if (huart == &huart2) {
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
    hdma_uart2_tx.Instance = DMA1_Channel2;
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
    hdma_uart3_tx.Instance = DMA1_Channel1;
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

    __HAL_LINKDMA(huart, hdmatx, hdma_uart3_tx);
  }
}
