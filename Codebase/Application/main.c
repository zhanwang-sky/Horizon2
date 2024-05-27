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
#define FLASH_MS 50U
#define PERIOD_MS 1000U

// Global variables
extern UART_HandleTypeDef huart2;
uint32_t rx_cnt;
uint8_t rx_buf;

// Functions
void blue_task(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  char msg_buf[64] = {0};
  int msg_len = 0;
  HAL_StatusTypeDef status = HAL_OK;

  while (1) {
    HAL_GPIO_WritePin(GPIOC, BLUE_PIN, GPIO_PIN_RESET);
    msg_len = snprintf(msg_buf, sizeof(msg_buf), "[BLUE] %u bytes received\r\n", rx_cnt);
    status = HAL_UART_Transmit_DMA(&huart2, (const uint8_t*) msg_buf, msg_len);
    assert_param(status == HAL_OK);
    HAL_Delay(FLASH_MS << 2U);
    HAL_GPIO_TogglePin(GPIOC, BLUE_PIN);

    vTaskDelayUntil(&xLastWakeTime, PERIOD_MS / portTICK_PERIOD_MS);
  }
}

void green_task(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  char msg_buf[64] = {0};
  int msg_len = 0;
  HAL_StatusTypeDef status = HAL_OK;

  while (1) {
    vTaskDelay((PERIOD_MS / portTICK_PERIOD_MS) >> 1U);

    HAL_GPIO_WritePin(GPIOC, GREEN_PIN, GPIO_PIN_RESET);
    msg_len = snprintf(msg_buf, sizeof(msg_buf), "[GREEN] last byte is %02hhxh\r\n", rx_buf);
    status = HAL_UART_Transmit_DMA(&huart2, (const uint8_t*) msg_buf, msg_len);
    assert_param(status == HAL_OK);
    HAL_Delay(FLASH_MS);
    HAL_GPIO_TogglePin(GPIOC, GREEN_PIN);

    vTaskDelayUntil(&xLastWakeTime, PERIOD_MS / portTICK_PERIOD_MS);
  }
}

int main(void) {
  BaseType_t xReturned = pdPASS;
  TaskHandle_t xHandle = NULL;

  // system init
  al_init();

  // create tasks
  xReturned = xTaskCreate(blue_task,
                          "blue",
                          configMINIMAL_STACK_SIZE,
                          NULL,  // thread param
                          1U,     // priority
                          &xHandle);
  assert_param(xReturned == pdPASS);

  xReturned = xTaskCreate(green_task,
                          "green",
                          configMINIMAL_STACK_SIZE,
                          NULL,  // thread param
                          2U,     // priority
                          &xHandle);
  assert_param(xReturned == pdPASS);

  // start receiving
  HAL_UART_Receive_IT(&huart2, &rx_buf, 1U);

  // start scheduler
  vTaskStartScheduler();

  // should not get here
  return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == &huart2) {
    ++rx_cnt;
    // continue receiving
    HAL_UART_Receive_IT(&huart2, &rx_buf, 1U);
  }
}
