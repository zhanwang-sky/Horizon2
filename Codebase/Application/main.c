//
//  main.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/18.
//

// Includes
#include "bsp.h"
#include "bsp_config.h"

int main(void) {
  // system init
  BSP_MCU_Init();
  BSP_GPIO_Init();

  HAL_Delay(2000);

  // loop here
  while (1) {
    // blue
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
    HAL_Delay(50);

    // green
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
    HAL_Delay(50);

    // blue + green
    HAL_Delay(800);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15);
    HAL_Delay(200);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14 | GPIO_PIN_15);
    HAL_Delay(1000);
  }
}
