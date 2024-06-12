//
//  al_pwm.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/3.
//

// Includes
#include "al.h"

#if (BSP_NR_PWMs > 0)

// ATTENTION:
// The timer configured in BSP should have a period of 50 Hz with a resolution of 1 us.

// Functions
void al_pwm_init(void) { }

int al_pwm_read(int fd, int* pulse) {
  TIM_HandleTypeDef* htim = NULL;
  uint32_t ch = 0U;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_PWMs) {
    return AL_ERROR_SANITY;
  }

  // get handler
  BSP_PWM_FD2HANDLECH(fd, htim, ch);
  if (!htim) {
    return AL_ERROR_BSP;
  }

  // read register
  if (pulse) {
    *pulse = (int) __HAL_TIM_GET_COMPARE(htim, ch);
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
  BSP_PWM_FD2HANDLECH(fd, htim, ch);
  if (!htim) {
    return AL_ERROR_BSP;
  }

  // write register
  __HAL_TIM_SET_COMPARE(htim, ch, (uint32_t) pulse);

  return 0;
}

#endif /* BSP_NR_PWMs */
