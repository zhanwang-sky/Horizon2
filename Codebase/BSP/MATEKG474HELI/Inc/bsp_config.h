//
//  bsp_config.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/19.
//

#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// Includes
#include "stm32g4xx_hal.h"

// Definitions
#define BSP_NR_GPIOs (2)

// Macros
#define BSP_GPIO_FD2PORTPIN(FD, PORT, PIN) \
do { \
  if ((FD) == 0) { \
    /* PC14 */ \
    PORT = GPIOC; \
    PIN = GPIO_PIN_14; \
  } else { \
    /* PC15 */ \
    PORT = GPIOC; \
    PIN = GPIO_PIN_15; \
  } \
} while (0)

#ifdef __cplusplus
}
#endif

#endif /* BSP_CONFIG_H */
