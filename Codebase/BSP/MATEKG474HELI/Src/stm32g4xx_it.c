/**
  ******************************************************************************
  * @file   stm32g4xx_it.c
  * @author MCD Application Team
  * @brief  Main Interrupt Service Routines.
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
extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_uart2_tx;
extern DMA_HandleTypeDef hdma_uart3_tx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;

/******************************************************************************/
/*                  Cortex-M4 Processor Exceptions Handlers                   */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void) {
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void) {
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1);
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void) {
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1);
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void) {
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1);
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void) {
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1);
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
#ifndef USE_FreeRTOS
void SVC_Handler(void) {
}
#endif

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void) {
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
#ifndef USE_FreeRTOS
void PendSV_Handler(void) {
}
#endif

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
#ifndef USE_FreeRTOS
void SysTick_Handler(void) {
}
#endif

/******************************************************************************/
/*                  STM32G4xx Peripherals Interrupt Handlers                  */
/* Add here the Interrupt Handler for the used peripheral(s) (PPP), for the   */
/* available peripheral interrupt handler's name please refer to the startup  */
/* file (startup_stm32g4xxxx.s).                                              */
/******************************************************************************/

/**
  * @brief  This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  * @param  None
  * @retval None
  */
void TIM6_DAC_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim6);
  HAL_IncTick();
}

/**
  * @brief  This function handles DMA2 channel1 global interrupt.
  * @param  None
  * @retval None
  */
void DMA2_Channel1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

/**
  * @brief  This function handles DMA2 channel2 global interrupt.
  * @param  None
  * @retval None
  */
void DMA2_Channel2_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

/**
  * @brief  This function handles SPI1 global interrupt.
  * @param  None
  * @retval None
  */
void SPI1_IRQHandler(void) {
  // ATTENTION:
  // Configure SPI and DMA interrupts with the same preemption priority.
  HAL_SPI_IRQHandler(&hspi1);
}

/**
  * @brief  This function handles DMA1 channel1 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void) {
  // ATTENTION:
  // `ErrorCode` might be modified by many HAL APIs in different interrupts.
  // Ensure that all interrupts which may modify `ErrorCode` have the same preemption priority.
  huart3.ErrorCode = HAL_UART_ERROR_NONE;
  HAL_DMA_IRQHandler(&hdma_uart3_tx);
}

/**
  * @brief  This function handles DMA1 channel2 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel2_IRQHandler(void) {
  huart2.ErrorCode = HAL_UART_ERROR_NONE;
  HAL_DMA_IRQHandler(&hdma_uart2_tx);
}

/**
  * @brief  This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void) {
  huart2.ErrorCode = HAL_UART_ERROR_NONE;
  HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief  This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void) {
  huart3.ErrorCode = HAL_UART_ERROR_NONE;
  HAL_UART_IRQHandler(&huart3);
}

/**
  * @brief  This function handles EXTI line[9:5] interrupts.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}
