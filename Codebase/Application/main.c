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
#define BLUE_PIN GPIO_PIN_14
#define GREEN_PIN GPIO_PIN_15

// Global variables
volatile uint32_t rx_cnt = 0;
volatile uint32_t rx_err = 0;
volatile uint8_t last_rx = 0;

// Functions
void uart_rx_cb(int fd, int ec, void* param) {
  ++rx_cnt;
  if (ec) {
    ++rx_err;
  }
  last_rx = (uint8_t) ((int) param);
}

void uart_tx_cb(int fd, int ec, void* param) {
  HAL_GPIO_TogglePin(GPIOC, GREEN_PIN);
}

void uart_demo_task(void* param) {
  TickType_t last_waken;
  char msg_buf[64];
  int msg_len;
  int rc;

  HAL_GPIO_WritePin(GPIOC, BLUE_PIN, GPIO_PIN_RESET);
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  HAL_GPIO_TogglePin(GPIOC, BLUE_PIN);

  al_uart_start_receiving(1, uart_rx_cb);

  last_waken = xTaskGetTickCount();
  for (uint32_t i = 0; ; ++i) {
    msg_len = 0;
    if (i == 0) {
      msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len, "h8ua\r\n");
    }
    msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len,
                        "[%d] rx_cnt=%u rx_err=%u last_rx=%02hhx\r\n",
                        i, rx_cnt, rx_err, last_rx);

    HAL_GPIO_WritePin(GPIOC, GREEN_PIN, GPIO_PIN_RESET);
    rc = al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, 0, uart_tx_cb, NULL);
    if (rc != 0) {
      HAL_GPIO_TogglePin(GPIOC, GREEN_PIN);
    }

    vTaskDelayUntil(&last_waken, 1000 / portTICK_PERIOD_MS);
  }
}

int main(void) {
  BaseType_t xReturned = pdPASS;

  // system init
  al_init();

  // create tasks
  xReturned = xTaskCreate(uart_demo_task,
                          "UART demo task",
                          configMINIMAL_STACK_SIZE,
                          NULL,
                          1,
                          NULL);
  assert_param(xReturned == pdTRUE);

  // start scheduler
  vTaskStartScheduler();

  // should not get here
  return 0;
}
