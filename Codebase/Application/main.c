//
//  main.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/18.
//

// Includes
#include <math.h>
#include <stdio.h>
#include "al.h"

// Definitions
#define BLUE_PIN GPIO_PIN_14
#define GREEN_PIN GPIO_PIN_15

// Global variables
extern SPI_HandleTypeDef hspi1;
volatile uint32_t rx_cnt = 0;
volatile uint32_t rx_err = 0;
volatile uint8_t last_rx = 0;
volatile uint8_t spi_done = 0;

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

  al_uart_start_receiving(1, uart_rx_cb);

  last_waken = xTaskGetTickCount();
  for (uint32_t i = 0; ; ++i) {
    msg_len = 0;
    if (i == 0) {
      msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len, "8hhd\r\n");
    }
    msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len,
                        "[%d] rx_cnt=%u rx_err=%u last_rx=%02hhx\r\n",
                        i, rx_cnt, rx_err, last_rx);

    HAL_GPIO_WritePin(GPIOC, GREEN_PIN, GPIO_PIN_RESET);
    rc = al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, 10, uart_tx_cb, NULL);
    if (rc != 0) {
      HAL_GPIO_TogglePin(GPIOC, GREEN_PIN);
    }

    vTaskDelayUntil(&last_waken, 1000 / portTICK_PERIOD_MS);
  }
}

void pid_loop(void* param) {
  const float delta = 3.141593e-3f; // step length
  TickType_t last_waken;

  last_waken = xTaskGetTickCount();
  while (1) {
    for (int i = 0; i < 2000; ++i) {
      float rad = (float) i * delta;
      float ratio = cosf(rad);
      int pulse = (int) (500.f * ratio);
      al_pwm_write(0, 1350 + pulse);
      al_pwm_write(1, 1400 + pulse);
      al_pwm_write(2, 1600 + pulse);
      al_pwm_write(3, 1750 + pulse);
      vTaskDelayUntil(&last_waken, 2 / portTICK_PERIOD_MS);
    }
  }
}

void imu_reader(void* param) {
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t rxbuf[2];
  uint8_t txbuf[2];
  char msg_buf[48];
  int msg_len;

  // power-up delay
  HAL_GPIO_WritePin(GPIOC, BLUE_PIN, GPIO_PIN_RESET);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  HAL_GPIO_TogglePin(GPIOC, BLUE_PIN);

  // read who_am_i
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  txbuf[0] = 0x80 | 0x75;
  status = HAL_SPI_TransmitReceive_DMA(&hspi1, txbuf, rxbuf, 2U);

  while (!spi_done);

  msg_len = snprintf(msg_buf, sizeof(msg_buf), "<ICM-42688-P> who_am_i=%02hhx, hal_rc=%d\r\n", rxbuf[1], status);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, 10, NULL, NULL);

  vTaskSuspend(NULL);
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
                          2,
                          NULL);
  assert_param(xReturned == pdTRUE);

  xReturned = xTaskCreate(pid_loop,
                          "PID loop",
                          configMINIMAL_STACK_SIZE,
                          NULL,
                          1,
                          NULL);
  assert_param(xReturned == pdTRUE);

  xReturned = xTaskCreate(imu_reader,
                          "IMU reader",
                          configMINIMAL_STACK_SIZE,
                          NULL,
                          3,
                          NULL);
  assert_param(xReturned == pdTRUE);

  // start scheduler
  vTaskStartScheduler();

  // should not get here
  return 0;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
  spi_done = 1;
}
