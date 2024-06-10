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
#define BSP_NR_ADCs  (3)
#define BSP_NR_GPIOs (3)
#define BSP_NR_EXTIs (1)
#define BSP_NR_PWMs  (4)
#define BSP_NR_UARTs (2)
#define BSP_NR_SPIs  (1)

// Macros
#define BSP_GPIO_FD2PORTPIN(FD, PORT, PIN) \
do { \
  if ((FD) == 0) { \
    /* PC14 */ \
    PORT = GPIOC; \
    PIN = GPIO_PIN_14; \
  } else if ((FD) == 1) { \
    /* PC15 */ \
    PORT = GPIOC; \
    PIN = GPIO_PIN_15; \
  } else if ((FD) == 2) { \
    /* PB9 */ \
    PORT = GPIOB; \
    PIN = GPIO_PIN_9; \
  } \
} while (0)

#define BSP_EXTI_PIN2FD(PIN, FD) \
do { \
  if ((PIN) == GPIO_PIN_7) { \
    FD = 0; \
  } \
} while (0)

#define BSP_PWM_FD2HANDLECH(FD, HANDLE, CH) \
do { \
  if ((FD) == 0) { \
    HANDLE = &htim3; \
    CH = TIM_CHANNEL_4; \
  } else if ((FD) == 1) { \
    HANDLE = &htim3; \
    CH = TIM_CHANNEL_3; \
  } else if ((FD) == 2) { \
    HANDLE = &htim3; \
    CH = TIM_CHANNEL_2; \
  } else if ((FD) == 3) { \
    HANDLE = &htim3; \
    CH = TIM_CHANNEL_1; \
  } \
} while (0)

#define BSP_UART_FD2HANDLE(FD, HANDLE) \
do { \
  if ((FD) == 0) { \
    HANDLE = &huart3; \
  } else if ((FD) == 1) { \
    HANDLE = &huart2; \
  } \
} while (0)

#define BSP_UART_HANDLE2FD(HANDLE, FD) \
do { \
  if ((HANDLE) == &huart3) { \
    FD = 0; \
  } else if ((HANDLE) == &huart2) { \
    FD = 1; \
  } \
} while (0)

#define BSP_UART_HANDLE_ERROR(HANDLE, RX_ERR, TX_ERR) \
do { \
  if ((HANDLE->ErrorCode & (HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE | HAL_UART_ERROR_RTO)) != 0U) { \
    /* Read RDR to clear RXNE */ \
    uint16_t uhdata = (uint16_t) READ_REG(HANDLE->Instance->RDR); \
    UNUSED(uhdata); \
    RX_ERR = 1; \
  } \
  if ((HANDLE->ErrorCode & HAL_UART_ERROR_DMA) != 0U) { \
    TX_ERR = 1; \
  } \
} while (0)

#define BSP_SPI_FD2HANDLE(FD, HANDLE) \
do { \
  if ((FD) == 0) { \
    HANDLE = &hspi1; \
  } \
} while (0)

#define BSP_SPI_HANDLE2FD(HANDLE, FD) \
do { \
  if ((HANDLE) == &hspi1) { \
    FD = 0; \
  } \
} while (0)

#define BSP_SPI_CS2PORTPIN(CS, PORT, PIN) \
do { \
  if ((CS) == 0) { \
    PORT = GPIOB; \
    PIN = GPIO_PIN_6; \
  } \
} while (0)

#define BSP_SPI_HANDLE_ERROR(HANDLE) \
do { \
  HAL_NVIC_SystemReset(); \
} while (0)

// Global variables
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;

// Function prototypes
void BSP_MCU_Init(void);
void BSP_ADC_Init(void);
void BSP_GPIO_Init(void);
void BSP_EXTI_Init(void);
void BSP_PWM_Init(void);
void BSP_UART_Init(void);
void BSP_SPI_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CONFIG_H */
