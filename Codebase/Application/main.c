/**
  ******************************************************************************
  * @file   main.c
  * @author Ji Chen <jicmail@icloud.com>
  * @brief  main function
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Ji Chen.
  * All rights reserved.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"
#include "bsp_config.h"

/* Functions -----------------------------------------------------------------*/
int main(void) {
  BSP_MCU_Init();

  BSP_GPIO_Init();

  while (1) {
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15);
  }
}

// XXX TODO: move to Abstraction Layer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
}
