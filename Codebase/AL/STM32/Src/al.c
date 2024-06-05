//
//  al.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/24.
//

// Includes
#include "al.h"

// Functions
void al_init(void) {
  BSP_MCU_Init();

#if (BSP_NR_GPIOs > 0)
  BSP_GPIO_Init();
#endif

#if (BSP_NR_PWMs > 0)
  BSP_PWM_Init();
#endif

#if (BSP_NR_UARTs > 0)
  BSP_UART_Init();
  al_uart_init();
#endif

#if (BSP_NR_SPIs > 0)
  BSP_SPI_Init();
#endif
}
