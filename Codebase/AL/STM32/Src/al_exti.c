//
//  al_exti.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/8.
//

// Includes
#include "al.h"

#if (BSP_NR_EXTIs > 0)

// Private variables
static SemaphoreHandle_t al_exti_line_semphrs[BSP_NR_EXTIs];
static al_exti_cb_t al_exti_callbacks[BSP_NR_EXTIs];

// Functions
void al_exti_init(void) {
  for (int i = 0; i < BSP_NR_EXTIs; ++i) {
    al_exti_line_semphrs[i] = xSemaphoreCreateBinary();
    xSemaphoreGive(al_exti_line_semphrs[i]);
  }
}

int al_exti_start(int fd, al_exti_cb_t cb) {
  // sanity check
  if (fd < 0 || fd >= BSP_NR_EXTIs || !cb) {
    return AL_ERROR_SANITY;
  }

  // lock
  if (xSemaphoreTake(al_exti_line_semphrs[fd], 0) != pdTRUE) {
    return AL_ERROR_BUSY;
  }
  al_exti_callbacks[fd] = cb;

  return 0;
}

// ISR callbacks
void HAL_GPIO_EXTI_Callback(uint16_t exti_pin) {
  int fd = -1;

  BSP_EXTI_PIN2FD(exti_pin, fd);

  if (fd >= 0 && fd < BSP_NR_EXTIs) {
    if (al_exti_callbacks[fd]) {
      al_exti_callbacks[fd](fd);
    }
  }
}

#endif /* BSP_NR_EXTIs */
