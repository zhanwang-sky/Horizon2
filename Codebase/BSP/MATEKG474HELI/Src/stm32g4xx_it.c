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
}
