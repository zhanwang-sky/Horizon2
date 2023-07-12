/**
  ******************************************************************************
  * @file   stm32f7xx_hal_timebase_tim.c
  * @author MCD Application Team
  * @brief  HAL time base based on the hardware TIM Template.
  *
  *         This file overrides the native HAL time base functions (defined as weak)
  *         the TIM time base:
  *          + Initializes the TIM peripheral to generate a Period elapsed Event at uwTickFreq
  *          + HAL_TIM_Base_MspInit/HAL_TIM_Base_MspDeInit are defined in stm32f7xx_hal_msp.c
  *          + HAL_IncTick is called inside TIM7_IRQHandler (stm32f7xx_it.c)
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
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
TIM_HandleTypeDef htim7;

/* Functions -----------------------------------------------------------------*/
/**
  * @brief  This function configures the TIM7 as a time base source.
  *         The time source is configured to have uwTickFreq time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  RCC_ClkInitTypeDef clkconfig = {0};
  uint32_t pFLatency = 0;
  uint32_t uwTimclock = 0;
  uint32_t uwPrescalerValue = 0;
  HAL_StatusTypeDef status = HAL_OK;

  /* Sanity check */
  if (TickPriority >= (1UL << __NVIC_PRIO_BITS)) {
    return HAL_ERROR;
  }

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Compute TIM7 clock */
  uwTimclock = HAL_RCC_GetPCLK1Freq();
  if (clkconfig.APB1CLKDivider != RCC_HCLK_DIV1) {
    uwTimclock <<= 1U;
  }
  /* It should be 16MHz at the beginning of program or 108MHz when SysClock is configured to 216MHz */

  /* Compute the prescaler value to have TIM7 counter clock equal to 100KHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 100000U) - 1U);

  /* Initialize TIM7 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = uwPrescalerValue;
  // htim7.Init.CounterMode = ?;
  htim7.Init.Period = (uint32_t) ((100U * uwTickFreq) - 1U); // 16-bit
  // htim7.Init.ClockDivision = ?;
  // htim7.Init.RepetitionCounter = ?;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  status = HAL_TIM_Base_Init(&htim7);
  if (status != HAL_OK) {
    return status;
  }

  /* Start the TIM time Base generation in interrupt mode */
  status = HAL_TIM_Base_Start_IT(&htim7);
  if (status != HAL_OK) {
    return status;
  }

  /* Set the TIM IRQ priority */
  HAL_NVIC_SetPriority(TIM7_IRQn, TickPriority, 0U);
  uwTickPrio = TickPriority;

  /* Enable TIM7 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM7_IRQn);

  /* Return function status */
  return status;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM7 update interrupt.
  * @retval None
  */
void HAL_SuspendTick(void) {
  /* Disable TIM7 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim7, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM7 update interrupt.
  * @retval None
  */
void HAL_ResumeTick(void) {
  /* Enable TIM7 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim7, TIM_IT_UPDATE);
}
