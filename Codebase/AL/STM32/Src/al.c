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

#if (BSP_NR_ADCs > 0)
  BSP_ADC_Init();
  al_analog_init();
#endif

#if (BSP_NR_GPIOs > 0)
  BSP_GPIO_Init();
  al_gpio_init();
#endif

#if (BSP_NR_EXTIs > 0)
  BSP_EXTI_Init();
  al_exti_init();
#endif

#if (BSP_NR_PWMs > 0)
  BSP_PWM_Init();
  al_pwm_init();
#endif

#if (BSP_NR_UARTs > 0)
  BSP_UART_Init();
  al_uart_init();
#endif

#if (BSP_NR_SPIs > 0)
  BSP_SPI_Init();
  al_spi_init();
#endif
}
