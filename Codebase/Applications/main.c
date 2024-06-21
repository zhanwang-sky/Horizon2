//
//  main.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/18.
//

// Includes
#include "al.h"

#if defined(UNIT_TEST)
extern void unit_test(void);
#endif

// FreeRTOS callbacks
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
  al_gpio_set(2, true);
}

// main function
int main(void) {
  // system init
  al_init();

#if defined(UNIT_TEST)
  al_wdog_init();
  unit_test();
#endif

  // start scheduler
  vTaskStartScheduler();

  // should not get here
  return 0;
}
