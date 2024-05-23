/**
  ******************************************************************************
  * @file   stm32g4xx_hal_timebase_tim.c
  * @author MCD Application Team
  * @brief  HAL time base based on the hardware TIM Template.
  *
  *         This file override the native HAL time base functions (defined as weak)
  *         the TIM time base:
  *          + Initializes the TIM peripheral to generate a Period elapsed Event at uwTickFreq
  *          + HAL_IncTick is called inside HAL_TIM_PeriodElapsedCallback at uwTickFreq
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
TIM_HandleTypeDef htim6;

/* Functions ---------------------------------------------------------------- */
/**
  * @brief  This function configures the TIM6 as a time base source.
  *         The time source is configured to have uwTickFreq time base with a
  *         dedicated Tick interrupt priority.
  * @note   This function is called automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  RCC_ClkInitTypeDef clkconfig = {0};
  uint32_t pFLatency = 0;
  uint32_t uwTimclock = 0;
  HAL_StatusTypeDef status = HAL_OK;

  /* Sanity check */
  if (TickPriority >= (1UL << __NVIC_PRIO_BITS) || SYSTICK_INT_PRIORITY != 0x0FUL) {
    return HAL_ERROR;
  }

  /* Enable TIM6 clock */
  __HAL_RCC_TIM6_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Compute TIM6 clock */
  uwTimclock = HAL_RCC_GetPCLK1Freq();

  /* Initialize TIM6 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = (uwTimclock / 100000U) - 1U; // 100 KHz counter clock
  htim6.Init.Period = (100U * uwTickFreq) - 1U; // (100 - 1)@1KHz || (1000 - 1)@100Hz || (10000 - 1)@10Hz, DO NOT EXCEED 0xffff!
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  status = HAL_TIM_Base_Init(&htim6);
  if (status != HAL_OK) {
    return status;
  }

  /* Start the TIM time Base generation in interrupt mode */
  status = HAL_TIM_Base_Start_IT(&htim6);
  if (status != HAL_OK) {
    return status;
  }

  /* Enable the TIM6 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* Configure the TIM IRQ priority */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, TickPriority, 0U);
  uwTickPrio = TickPriority;

  /* Return function status */
  return status;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM6 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void) {
  /* Disable TIM6 update interrupt */
  __HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by enabling TIM6 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void) {
  /* Enable TIM6 update interrupt */
  __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}
