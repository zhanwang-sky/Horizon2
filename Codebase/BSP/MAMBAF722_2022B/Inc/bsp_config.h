/**
  ******************************************************************************
  * @file   bsp_config.h
  * @author Ji Chen <jicmail@icloud.com>
  * @brief  Board Support Package config file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 Ji Chen.
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __BSP_CONFIG_H
#define __BSP_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Definitions ---------------------------------------------------------------*/
#define BSP_NR_GPIOs (2)

/* Macros --------------------------------------------------------------------*/
#define BSP_GPIO_FD2PORTPIN(FD, PORT, PIN) \
do { \
    if ((FD) == 0) { \
        /* C15 */ \
        PORT = GPIOC; \
        PIN = GPIO_PIN_15; \
    } else { \
        /* C14 */ \
        PORT = GPIOC; \
        PIN = GPIO_PIN_14; \
    } \
} while (0)

/* Global variables ----------------------------------------------------------*/
extern TIM_HandleTypeDef htim7;

#endif /* __BSP_CONFIG_H */
