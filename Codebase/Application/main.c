//
//  main.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/18.
//

// Includes
#include "al.h"

#define BLUE_PIN GPIO_PIN_14
#define GREEN_PIN GPIO_PIN_15

#define BLUE_DUTY 300
#define GREEN_DUTY 300
#define DUMMY_DUTY 200
#define PERIOD 1000

void blue_task(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    HAL_Delay(GREEN_DUTY);
    HAL_Delay(DUMMY_DUTY);

    HAL_GPIO_WritePin(GPIOC, BLUE_PIN, GPIO_PIN_RESET);
    HAL_Delay(BLUE_DUTY);
    HAL_GPIO_TogglePin(GPIOC, BLUE_PIN);

    vTaskDelayUntil(&xLastWakeTime, PERIOD / portTICK_PERIOD_MS);
  }
}

void green_task(void* pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // vTaskDelay(BLUE_DUTY / portTICK_PERIOD_MS);
    // vTaskDelay(DUMMY_DUTY / portTICK_PERIOD_MS);

    HAL_GPIO_WritePin(GPIOC, GREEN_PIN, GPIO_PIN_RESET);
    vTaskDelay(GREEN_DUTY / portTICK_PERIOD_MS);
    HAL_GPIO_TogglePin(GPIOC, GREEN_PIN);

    vTaskDelayUntil(&xLastWakeTime, PERIOD / portTICK_PERIOD_MS);
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
                          1,     // priority
                          &xHandle);
  assert_param(xReturned == pdPASS);

  xReturned = xTaskCreate(green_task,
                          "green",
                          configMINIMAL_STACK_SIZE,
                          NULL,  // thread param
                          2,     // priority
                          &xHandle);
  assert_param(xReturned == pdPASS);

  // start scheduler
  vTaskStartScheduler();

  // should not get here
  return 0;
}
