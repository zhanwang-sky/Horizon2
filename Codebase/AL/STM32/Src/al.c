//
//  al.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/24.
//

// Includes
#include "al.h"

// Functions
void al_init(void) {
  BSP_MCU_Init();

#if (BSP_NR_GPIOs > 0)
  BSP_GPIO_Init();
#endif
}

// ISRs
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  // nothing to do
}
