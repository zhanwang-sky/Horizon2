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
#define BSP_NR_UARTs (2)

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

#define BSP_UART_FD2HANDLE(FD, HANDLE) \
do { \
  if ((FD) == 0) { \
    HANDLE = &huart3; \
  } else { \
    HANDLE = &huart2; \
  } \
} while (0)

#define BSP_UART_HANDLE2FD(HANDLE, FD) \
do { \
  if ((HANDLE) == &huart3) { \
    FD = 0; \
  } else { \
    FD = 1; \
  } \
} while (0)

#define BSP_UART_HANDLE_ERROR(HANDLE, RX_ERR, TX_ERR) \
do { \
  if ((HANDLE->ErrorCode & (HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE | HAL_UART_ERROR_RTO)) != 0U) { \
    /* Read RDR to clear RXNE */ \
    volatile uint32_t dummy = HANDLE->Instance->RDR; \
    (void) dummy; \
    RX_ERR = 1; \
  } \
  if ((HANDLE->ErrorCode & HAL_UART_ERROR_DMA) != 0U) { \
    TX_ERR = 1; \
  } \
} while (0)

// Global variables
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Function prototypes
void BSP_MCU_Init(void);
void BSP_GPIO_Init(void);
void BSP_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CONFIG_H */
