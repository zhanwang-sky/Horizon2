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
#define BSP_NR_PWMs  (6)
#define BSP_NR_UARTs (3)
#define BSP_NR_I2Cs  (1)
#define BSP_NR_SPIs  (1)
#define BSP_HAS_WDOG (1)

// Macros
#define BSP_ADC_FD2PDATA(FD, PDATA) \
do { \
  if ((FD) == 0) { \
    PDATA = &hadc1.Instance->DR; \
  } else if ((FD) == 1) { \
    PDATA = &adc2_data[0]; \
  } else if ((FD) == 2) { \
    PDATA = &adc2_data[1]; \
  } \
} while (0)

#define BSP_ADC_CALCULATE_VALUE(FD, DATA) \
( \
((FD) == 0) ? \
((float)(__HAL_ADC_CALC_TEMPERATURE((VDD_VALUE), (DATA), (ADC_RESOLUTION_12B)))) : \
(((float)(DATA)) * ((float)(VDD_VALUE)) / 4095000.f) \
)

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
  } else if ((FD) == 4) { \
    HANDLE = &htim2; \
    CH = TIM_CHANNEL_2; \
  } else if ((FD) == 5) { \
    HANDLE = &htim2; \
    CH = TIM_CHANNEL_1; \
  } \
} while (0)

#define BSP_UART_FD2HANDLE(FD, HANDLE) \
do { \
  if ((FD) == 0) { \
    HANDLE = &huart1; \
  } else if ((FD) == 1) { \
    HANDLE = &huart2; \
  } else if ((FD) == 2) { \
    HANDLE = &huart3; \
  } \
} while (0)

#define BSP_UART_HANDLE2FD(HANDLE, FD) \
do { \
  if ((HANDLE) == &huart1) { \
    FD = 0; \
  } else if ((HANDLE) == &huart2) { \
    FD = 1; \
  } else if ((HANDLE) == &huart3) { \
    FD = 2; \
  } \
} while (0)

#define BSP_UART_HANDLE_ERROR(HANDLE) \
do { \
  if (((HANDLE)->ErrorCode & HAL_UART_ERROR_DMA) != 0U) { \
    /* fatal error */ \
    HAL_NVIC_SystemReset(); \
  } else { \
    /* Read RDR to clear RXNE flag */ \
    uint16_t uhdata = (uint16_t) READ_REG((HANDLE)->Instance->RDR); \
    UNUSED(uhdata); \
  } \
} while (0)

#define BSP_I2C_FD2HANDLE(FD, HANDLE) \
do { \
  if ((FD) == 0) { \
    HANDLE = &hi2c1; \
  } \
} while (0)

#define BSP_I2C_HANDLE2FD(HANDLE, FD) \
do { \
  if ((HANDLE) == &hi2c1) { \
    FD = 0; \
  } \
} while (0)

#define BSP_I2C_HANDLE_ERROR(HANDLE) \
do { \
  if (((HANDLE)->ErrorCode & (HAL_I2C_ERROR_DMA | HAL_I2C_ERROR_SIZE)) != 0U) { \
    /* fatal error */ \
    HAL_NVIC_SystemReset(); \
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
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern IWDG_HandleTypeDef hiwdg;

extern volatile uint32_t adc2_data[2];

// Function prototypes
void BSP_MCU_Init(void);
void BSP_ADC_Init(void);
void BSP_GPIO_Init(void);
void BSP_EXTI_Init(void);
void BSP_PWM_Init(void);
void BSP_UART_Init(void);
void BSP_I2C_Init(void);
void BSP_SPI_Init(void);
void BSP_WDOG_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CONFIG_H */
