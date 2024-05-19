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

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* Private function prototypes ---------------------------------------------- */
/* Private functions -------------------------------------------------------- */

/**
  * @brief  Initialize the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void) {
  /* Enable RCC clock */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral */
  HAL_PWREx_DisableUCPDDeadBattery();
}

/**
  * @brief  DeInitialize the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspDeInit(void) {
}

/**
  * @brief  Initialize the PPP MSP.
  * @param  None
  * @retval None
  */
/*void HAL_PPP_MspInit(void) {
}*/

/**
  * @brief  DeInitialize the PPP MSP.
  * @param  None
  * @retval None
  */
/*void HAL_PPP_MspDeInit(void) {
}*/
