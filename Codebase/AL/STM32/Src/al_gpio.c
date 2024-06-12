//
//  al_gpio.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/8.
//

// Includes
#include "al.h"

#if (BSP_NR_GPIOs > 0)

// ATTENTION:
// `al_gpio_toggle` does not guarantee the atomicity of operations,
// which may cause the state of other pins to be incorrect.

// Functions
void al_gpio_init(void) { }

int al_gpio_get(int fd, bool* state) {
  GPIO_TypeDef* port = NULL;
  uint16_t pin = 0U;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_GPIOs) {
    return AL_ERROR_SANITY;
  }

  // get handler
  BSP_GPIO_FD2PORTPIN(fd, port, pin);
  if (!port) {
    return AL_ERROR_BSP;
  }

  // read register
  if (state) {
    *state = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? true : false;
  }

  return 0;
}

int al_gpio_set(int fd, bool state) {
  GPIO_TypeDef* port = NULL;
  uint16_t pin = 0U;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_GPIOs) {
    return AL_ERROR_SANITY;
  }

  // get handler
  BSP_GPIO_FD2PORTPIN(fd, port, pin);
  if (!port) {
    return AL_ERROR_BSP;
  }

  // write register
  HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);

  return 0;
}

int al_gpio_toggle(int fd) {
  GPIO_TypeDef* port = NULL;
  uint16_t pin = 0U;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_GPIOs) {
    return AL_ERROR_SANITY;
  }

  // get handler
  BSP_GPIO_FD2PORTPIN(fd, port, pin);
  if (!port) {
    return AL_ERROR_BSP;
  }

  // read-modify-write
  HAL_GPIO_TogglePin(port, pin);

  return 0;
}

#endif /* BSP_NR_GPIOs */
