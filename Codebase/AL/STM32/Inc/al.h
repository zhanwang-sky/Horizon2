//
//  al.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/24.
//

#ifndef AL_H
#define AL_H

// Includes
#include <limits.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#if (BSP_NR_ADCs > 0)
#include "al_analog.h"
#endif

#if (BSP_NR_GPIOs > 0)
#include "al_gpio.h"
#endif

#if (BSP_NR_EXTIs > 0)
#include "al_exti.h"
#endif

#if (BSP_NR_PWMs > 0)
#include "al_pwm.h"
#endif

#if (BSP_NR_UARTs > 0)
#include "al_uart.h"
#endif

#if (BSP_NR_I2Cs > 0)
#include "al_i2c.h"
#endif

#if (BSP_NR_SPIs > 0)
#include "al_spi.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Definitions
#define AL_ERROR_SANITY (INT_MIN)
#define AL_ERROR_BSP    (INT_MIN + 1)
#define AL_ERROR_LIMITS (INT_MIN + 2)
#define AL_ERROR_BUSY   (INT8_MIN - 1)
#define AL_ERROR_HAL    (-1)

// Function prototypes
void al_init(void);

#if defined(BSP_HAS_WDOG)
void al_wdog_init(void);
void al_wdog_feed(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* AL_H */
