//
//  main.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/18.
//

// Includes
#include <stdio.h>
#include "al.h"

// Definitions
#define MAX_TASKS (100)

// Function prototypes
#if defined(UNIT_TEST)
extern void unit_test(void);
#endif
extern void fc_init(int fd);

// FreeRTOS callbacks
#if defined(DEBUG)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName) {
  al_gpio_set(2, true);
}
#endif

// Tasks
#if defined(DEBUG)
void task_monitor(void* param) {
  static TaskStatus_t task_status_arr[MAX_TASKS];
  static char msg_buf[512];

  UBaseType_t nr_tasks;
  UBaseType_t high_water_mark;
  size_t free_heap_size;
  TickType_t last_wake;
  int msg_len;

  last_wake = xTaskGetTickCount();
  for (uint32_t round = 0; ; ++round) {
    nr_tasks = uxTaskGetSystemState(task_status_arr,
                                    MAX_TASKS,
                                    NULL);

    msg_len = snprintf(msg_buf, sizeof(msg_buf),
                       "----------\r\n"
                       "(%u)\r\n"
                       "new feature: fc\r\n"
                       "Stack high water mark(word): %s\r\n",
                       round,
                       !nr_tasks ? "Oops, too many tasks!" : "");

    for (int i = 0; i < nr_tasks; ++i) {
      high_water_mark = uxTaskGetStackHighWaterMark(task_status_arr[i].xHandle);
      msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len,
                          "%s: %lu\r\n",
                          task_status_arr[i].pcTaskName,
                          high_water_mark);
    }

    free_heap_size = xPortGetFreeHeapSize();
    msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len,
                        "Free heap size(byte): %zu\r\n"
                        "\r\n",
                        free_heap_size);

    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

    vTaskDelayUntil(&last_wake, 5000 / portTICK_PERIOD_MS);
  }
}
#endif

// main function
int main(void) {
  BaseType_t ret = pdFAIL;

  // system init
  al_init();

  // module init
#if defined(UNIT_TEST)
  unit_test();
#endif
  fc_init(2);

#if defined(DEBUG)
  ret = xTaskCreate(task_monitor,
                    "TASK_MONITOR",
                    2 * configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY,
                    NULL);
  configASSERT(ret == pdPASS);
#endif

  // start scheduler
  vTaskStartScheduler();

  // should not get here
  return 0;
}
