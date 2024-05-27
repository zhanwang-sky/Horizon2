//
//  bsp_config.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/19.
//

#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

// Includes
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Definitions
#define BSP_NR_GPIOs (2)
#define BSP_NR_UARTs (1)

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

// Functions
void BSP_MCU_Init(void);
void BSP_GPIO_Init(void);
void BSP_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CONFIG_H */
