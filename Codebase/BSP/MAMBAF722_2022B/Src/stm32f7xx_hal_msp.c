/**
  ******************************************************************************
  * @file   stm32f7xx_hal_msp.c
  * @author MCD Application Team
  * @brief  HAL MSP module.
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

/* Functions -----------------------------------------------------------------*/
void HAL_MspInit(void) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
}

void HAL_MspDeInit(void) {
  return;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM7) {
    /* Enable TIM7 clock */
    __HAL_RCC_TIM7_CLK_ENABLE();
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM7) {
    /* Disable TIM7 clock */
    __HAL_RCC_TIM7_CLK_DISABLE();
  }
}
