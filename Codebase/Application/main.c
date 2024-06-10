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

// Global variables
SemaphoreHandle_t spi_done;
float servo_movement_period = 3.f;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

// Functions
void spi_cb(int fd, int ec, void* param) {
  BaseType_t should_yield = pdFALSE;
  xSemaphoreGiveFromISR(spi_done, &should_yield);
  portYIELD_FROM_ISR(should_yield);
}

void test_spi(void* param) {
  // ICM-42688-P registers
  static const uint8_t DEVICE_CONFIG = 0x11;
  static const uint8_t TEMP_DATA1 = 0x1d;
  static const uint8_t PWR_MGMT0 = 0x4e;
  static const uint8_t ACCEL_CONFIG0 = 0x50;
  static const uint8_t WHO_AM_I = 0x75;

  TickType_t last_wake;
  uint8_t spi_buf[4];
  char msg_buf[96];
  int msg_len;
  int rc;

  // power-up delay
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // reset
  spi_buf[0] = DEVICE_CONFIG;
  spi_buf[1] = 0x01; // Enable reset
  rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 2, -1, spi_cb, NULL);
  if (rc == 0) {
    xSemaphoreTake(spi_done, portMAX_DELAY);
  }
  // reset delay
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // read WHO_AM_I
  spi_buf[0] = 0x80 | WHO_AM_I;
  spi_buf[1] = 0;
  rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 2, -1, spi_cb, NULL);
  if (rc == 0) {
    xSemaphoreTake(spi_done, portMAX_DELAY);
  }
  msg_len = snprintf(msg_buf, sizeof(msg_buf), "{} WHO_AM_I=%02hhx\r\n", spi_buf[1]);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  // enable accel
  spi_buf[0] = PWR_MGMT0;
  spi_buf[1] = 0x02; // Places accelerometer in Low Power (LP) Mode
  rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 2, -1, spi_cb, NULL);
  if (rc == 0) {
    xSemaphoreTake(spi_done, portMAX_DELAY);
  }
  // stabilization delay
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // config accel
  spi_buf[0] = ACCEL_CONFIG0;
  spi_buf[1] = 0x0e; // 1.5625Hz (LP mode)
  rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 2, -1, spi_cb, NULL);
  if (rc == 0) {
    xSemaphoreTake(spi_done, portMAX_DELAY);
  }

  last_wake = xTaskGetTickCount();
  for (uint32_t i = 0; ; ++i) {
    int16_t temp_data;
    float temp;

    vTaskDelayUntil(&last_wake, 5000 / portTICK_PERIOD_MS);

    // read temp
    spi_buf[0] = 0x80 | TEMP_DATA1;
    spi_buf[1] = 0;
    spi_buf[2] = 0;
    rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 3, -1, spi_cb, NULL);
    if (rc == 0) {
      xSemaphoreTake(spi_done, portMAX_DELAY);
    }
    temp_data = (int16_t) ((((uint16_t) spi_buf[1]) << 8) | ((uint16_t) spi_buf[2]));
    temp = (temp_data / 132.48f) + 25.f;

    msg_len = snprintf(msg_buf, sizeof(msg_buf), "{%d} temp=%f\r\n", i, temp);
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);
  }
}

void test_pwm(void* param) {
  const float PI = 3.141593f;
  const int PWM_FREQ = 50;
  float* p_period = (float*) param;
  float period = (p_period != NULL) ? *p_period : 5.f;
  float cycles = PWM_FREQ * period;
  float delta = 2 * PI / cycles;
  char msg_buf[64];
  int msg_len;
  TickType_t last_wake;

  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "<> period=%f cycles=%f delta=%f\r\n",
                     period, cycles, delta);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  last_wake = xTaskGetTickCount();
  while (1) {
    for (int i = 0; i < (int) roundf(cycles); ++i) {
      float arc = (float) i * delta;
      int pulse = (int) roundf(500.f * cosf(arc));
      al_pwm_write(0, 1500 + pulse);
      al_pwm_write(1, 1350 + pulse);
      al_pwm_write(2, 1050 + pulse);
      al_pwm_write(3, 1850 + pulse);
      vTaskDelayUntil(&last_wake, 1000 / PWM_FREQ / portTICK_PERIOD_MS);
    }
  }
}

void test_gpio(void* param) {
  TickType_t last_wake;

  al_gpio_set(0, false);

  last_wake = xTaskGetTickCount();
  for (uint32_t i = 0; ; ++i) {
    vTaskDelayUntil(&last_wake, 100 / portTICK_PERIOD_MS);
    if (i % 2 == 0) {
      al_gpio_toggle(0);
    }
    if (i % 3 == 0) {
      al_gpio_toggle(1);
    }
    if (i % 5 == 0) {
      al_gpio_set(2, (i % 2 == 0));
    }
  }
}

void test_adc(void* param) {
  TickType_t last_wake;
  float adc_values[3];
  int adc_data[3];
  char msg_buf[64];
  int msg_len;

  last_wake = xTaskGetTickCount();
  for (uint32_t i = 0; ; ++i) {
    vTaskDelayUntil(&last_wake, 500 / portTICK_PERIOD_MS);

    if (i % 10 == 0) {
      for (int i = 0; i < 3; ++i) {
        al_analog_read_raw(i, &adc_data[i]);
      }
      msg_len = snprintf(msg_buf, sizeof(msg_buf),
                         "(%d) ADC_0=%d ADC_1=%d ADC_2=%d\r\n",
                         i, adc_data[0], adc_data[1], adc_data[2]);
    } else {
      for (int i = 0; i < 3; ++i) {
        al_analog_read(i, &adc_values[i]);
      }
      msg_len = snprintf(msg_buf, sizeof(msg_buf),
                         "(%d) temp=%.3f | batt=%.3f curr=%.3f\r\n",
                         i, adc_values[0], adc_values[1], adc_values[2]);
    }
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);
  }
}

int main(void) {
  BaseType_t xReturned = pdPASS;

  // system init
  al_init();

  spi_done = xSemaphoreCreateBinary();

  // create tasks
  xReturned = xTaskCreate(test_spi,
                          "TEST SPI",
                          configMINIMAL_STACK_SIZE,
                          NULL,
                          3,
                          NULL);
  configASSERT(xReturned == pdPASS);

  xReturned = xTaskCreate(test_pwm,
                          "TEST PWM",
                          configMINIMAL_STACK_SIZE,
                          &servo_movement_period,
                          1,
                          NULL);
  configASSERT(xReturned == pdPASS);

  xReturned = xTaskCreate(test_gpio,
                          "TEST GPIO",
                          configMINIMAL_STACK_SIZE,
                          NULL,
                          1,
                          NULL);
  configASSERT(xReturned == pdPASS);

  xReturned = xTaskCreate(test_adc,
                          "TEST ADC",
                          configMINIMAL_STACK_SIZE,
                          NULL,
                          1,
                          NULL);
  configASSERT(xReturned == pdPASS);

  // start scheduler
  vTaskStartScheduler();

  // should not get here
  return 0;
}
