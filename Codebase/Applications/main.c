//
//  main.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/18.
//

// Includes
#include <stdio.h>
#include "al.h"
#include "sbus_rx.h"

// Definitions
#define MAX_TASKS 100

// Function prototypes
#if defined(UNIT_TEST)
extern void unit_test(void);
#endif

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
                       "new feature: INCLUDE_vTaskSuspend\r\n"
                       "Stack high water mark(word):\r\n",
                       round);

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

void sbus_dumper(void* param) {
  static char msg_buf[256];

  int rc;
  int msg_len;
  TickType_t prev_tick;
  TickType_t curr_tick;
  const sbus_frame_t* p_frame;

  rc = sbus_rx_init(2);
  configASSERT(rc == 0);

  prev_tick = xTaskGetTickCount();
  while (1) {
    rc = sbus_rx_poll(&p_frame, 20);
    curr_tick = xTaskGetTickCount();
    if ((curr_tick - prev_tick) < (50 / portTICK_PERIOD_MS)) {
      continue;
    }
    prev_tick = curr_tick;
    msg_len = snprintf(msg_buf, sizeof(msg_buf),
                       "%08x| ", curr_tick);
    if (rc == 0) {
      msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len,
                          "%4hu %4hu %4hu %4hu %4hu %4hu %4hu %s %s\r\n",
                          p_frame->channels[0],
                          p_frame->channels[1],
                          p_frame->channels[2],
                          p_frame->channels[3],
                          p_frame->channels[4],
                          p_frame->channels[5],
                          p_frame->channels[6],
                          p_frame->frame_lost ? "*" : " ",
                          p_frame->failsafe ? "!" : "");
    } else {
      msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len,
                          "na\r\n");
    }
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);
  }
}

// main function
int main(void) {
  BaseType_t ret = pdFAIL;

  // system init
  al_init();

#if defined(UNIT_TEST)
  unit_test();
#endif

#if defined(DEBUG)
  ret = xTaskCreate(task_monitor,
                    "TASK_MONITOR",
                    2 * configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY,
                    NULL);
  configASSERT(ret == pdPASS);
#endif

  ret = xTaskCreate(sbus_dumper,
                    "SBUS_DUMPER",
                    2 * configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY + 1,
                    NULL);
  configASSERT(ret == pdPASS);

  // start scheduler
  vTaskStartScheduler();

  // should not get here
  return 0;
}
