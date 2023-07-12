/**
  ******************************************************************************
  * @file  stm32f7xx_it.c
  * @brief Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under the BSD-3-Clause license.
  * You may obtain a copy of the BSD-3-Clause at:
  * https://opensource.org/licenses/BSD-3-Clause
  *
  * Contributor(s):
  *
  * Ji Chen <jicmail@icloud.com>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Global variables ----------------------------------------------------------*/
extern TIM_HandleTypeDef htim7;

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {
  while (1);
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void) {
  while (1);
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {
  while (1);
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void) {
  while (1);
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {
  while (1);
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void) {
  return;
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void) {
  return;
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void) {
  return;
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void) {
  return;
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim7);
}
