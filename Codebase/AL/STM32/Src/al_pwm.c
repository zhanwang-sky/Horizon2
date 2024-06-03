//
//  al_pwm.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/3.
//

// Includes
#include "al.h"

#if (BSP_NR_PWMs > 0)

// Functions
int al_pwm_read(int fd, int* pulse) {
  TIM_HandleTypeDef* htim = NULL;
  uint32_t ch = 0U;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_PWMs) {
    return AL_ERROR_SANITY;
  }

  // get handler
  BSP_PWM_FD2HCH(fd, htim, ch);
  if (!htim) {
    return AL_ERROR_BSP;
  }

  // read register
  if (pulse) {
    *pulse = __HAL_TIM_GET_COMPARE(htim, ch);
  }

  return 0;
}

int al_pwm_write(int fd, int pulse) {
  TIM_HandleTypeDef* htim = NULL;
  uint32_t ch = 0U;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_PWMs || pulse < 0) {
    return AL_ERROR_SANITY;
  }

  // HAL limits
  if (pulse > UINT16_MAX) {
    return AL_ERROR_LIMITS;
  }

  // get handler
  BSP_PWM_FD2HCH(fd, htim, ch);
  if (!htim) {
    return AL_ERROR_BSP;
  }

  // write register
  __HAL_TIM_SET_COMPARE(htim, ch, pulse);

  return 0;
}

#endif /* BSP_NR_PWMs */
