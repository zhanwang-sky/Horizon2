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
extern DMA_HandleTypeDef hdma_uart1_tx;
extern DMA_HandleTypeDef hdma_uart2_tx;
extern DMA_HandleTypeDef hdma_uart3_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

#if defined(UNIT_TEST)
extern volatile uint32_t uart_intr_cnt[10];
extern volatile uint32_t uart_dmatx_intr_cnt[10];

extern volatile uint32_t i2c_ev_intr_cnt[10];
extern volatile uint32_t i2c_er_intr_cnt[10];
extern volatile uint32_t i2c_dmarx_intr_cnt[10];
extern volatile uint32_t i2c_dmatx_intr_cnt[10];

extern volatile uint32_t spi_intr_cnt[10];
extern volatile uint32_t spi_dmatx_intr_cnt[10];
extern volatile uint32_t spi_dmarx_intr_cnt[10];
#endif

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
  /* GREEN */
  GPIOC->BSRR = (((uint32_t) GPIO_PIN_15) << 16U) | ((uint32_t) GPIO_PIN_14);
  while (1);
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void) {
  /* Go to infinite loop when Memory Manage exception occurs */
  /* GREEN */
  GPIOC->BSRR = (((uint32_t) GPIO_PIN_15) << 16U) | ((uint32_t) GPIO_PIN_14);
  while (1);
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void) {
  /* Go to infinite loop when Bus Fault exception occurs */
  /* GREEN */
  GPIOC->BSRR = (((uint32_t) GPIO_PIN_15) << 16U) | ((uint32_t) GPIO_PIN_14);
  while (1);
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void) {
  /* Go to infinite loop when Usage Fault exception occurs */
  /* GREEN */
  GPIOC->BSRR = (((uint32_t) GPIO_PIN_15) << 16U) | ((uint32_t) GPIO_PIN_14);
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
  * @brief  This function handles SPI1 global interrupt.
  * @param  None
  * @retval None
  */
void SPI1_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++spi_intr_cnt[0];
#endif
  HAL_SPI_IRQHandler(&hspi1);
}

/**
  * @brief  This function handles DMA2 channel1 global interrupt.
  * @param  None
  * @retval None
  */
void DMA2_Channel1_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++spi_dmatx_intr_cnt[0];
#endif
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

/**
  * @brief  This function handles DMA2 channel2 global interrupt.
  * @param  None
  * @retval None
  */
void DMA2_Channel2_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++spi_dmarx_intr_cnt[0];
#endif
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

/**
  * @brief  This function handles DMA1 channel7 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel7_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++i2c_dmarx_intr_cnt[0];
#endif
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
}

/**
  * @brief  This function handles I2C1 event interrupt / I2C1 wake-up interrupt through EXTI line 23.
  * @param  None
  * @retval None
  */
void I2C1_EV_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++i2c_ev_intr_cnt[0];
#endif
  HAL_I2C_EV_IRQHandler(&hi2c1);
}

/**
  * @brief  This function handles I2C1 error interrupt.
  * @param  None
  * @retval None
  */
void I2C1_ER_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++i2c_er_intr_cnt[0];
#endif
  HAL_I2C_ER_IRQHandler(&hi2c1);
}

/**
  * @brief  This function handles DMA1 channel8 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel8_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++i2c_dmatx_intr_cnt[0];
#endif
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
}

/**
  * @brief  This function handles DMA1 channel2 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel2_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++uart_dmatx_intr_cnt[0];
#endif
  huart1.ErrorCode = HAL_UART_ERROR_NONE;
  HAL_DMA_IRQHandler(&hdma_uart1_tx);
}

/**
  * @brief  This function handles DMA1 channel4 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel4_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++uart_dmatx_intr_cnt[1];
#endif
  huart2.ErrorCode = HAL_UART_ERROR_NONE;
  HAL_DMA_IRQHandler(&hdma_uart2_tx);
}

/**
  * @brief  This function handles DMA1 channel6 global interrupt.
  * @param  None
  * @retval None
  */
void DMA1_Channel6_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++uart_dmatx_intr_cnt[2];
#endif
  huart3.ErrorCode = HAL_UART_ERROR_NONE;
  HAL_DMA_IRQHandler(&hdma_uart3_tx);
}

/**
  * @brief  This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++uart_intr_cnt[0];
#endif
  huart1.ErrorCode = HAL_UART_ERROR_NONE;
  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief  This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++uart_intr_cnt[1];
#endif
  huart2.ErrorCode = HAL_UART_ERROR_NONE;
  HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief  This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void) {
#if defined(UNIT_TEST)
  ++uart_intr_cnt[2];
#endif
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
