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
#define MAX_TASKS 64

// Global variables
TaskHandle_t tasks[MAX_TASKS];
int nr_tasks = 0;
SemaphoreHandle_t spi_done;
SemaphoreHandle_t i2c_done;
float servo_movement_period = 3.f;

// Functions
void spi_cb(int fd, int ec, void* param) {
  BaseType_t should_yield = pdFALSE;
  xSemaphoreGiveFromISR(spi_done, &should_yield);
  portYIELD_FROM_ISR(should_yield);
}

void i2c_cb(int fd, int ec, void* param) {
  int *p_ec = (int*) param;
  BaseType_t should_yield = pdFALSE;
  *p_ec = ec;
  xSemaphoreGiveFromISR(i2c_done, &should_yield);
  portYIELD_FROM_ISR(should_yield);
}

// Tasks
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
      pulse = (int) roundf(800.f * cosf(arc));
      al_pwm_write(4, 1450 + pulse);
      al_pwm_write(5, 1550 + pulse);
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
  }
}

void test_adc(void* param) {
  TickType_t last_wake;
  float temp;
  float vbat;
  int temp_raw;
  int curr;
  char msg_buf[64];
  int msg_len;

  last_wake = xTaskGetTickCount();
  for (uint32_t i = 0; ; ++i) {
    vTaskDelayUntil(&last_wake, 500 / portTICK_PERIOD_MS);

    al_analog_read(0, &temp);
    al_analog_read_raw(0, &temp_raw);
    al_analog_read(1, &vbat);
    al_analog_read_raw(2, &curr);
    msg_len = snprintf(msg_buf, sizeof(msg_buf),
                       "(%d) temp=%.0f(%d) vbat=%.3f curr=%d\r\n",
                       i, temp, temp_raw, vbat, curr);
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);
  }
}

void test_i2c(void* param) {
  // Device: Goertek SPL06
  static const uint8_t spl_addr = 0x76;
  static const uint8_t reset_reg = 0x0c;
  static const uint8_t id_reg = 0x0d;
  static char msg_buf[256];
  static uint8_t i2c_buf[32];
  int msg_len;
  int rc;
  int ec;
  TickType_t last_wake;

  // power-up delay
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // ping SPL06
  ec = 0;
  rc = al_i2c_async_write(0, spl_addr, NULL, 0, -1, i2c_cb, &ec);
  if (rc == 0) {
    xSemaphoreTake(i2c_done, portMAX_DELAY);
  }
  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST I2C(400k) write: device %02hhx, rc=%d, ec=%d\r\n",
                     spl_addr, rc, ec);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  // sleep for a while
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // try to read HMC5883?
  ec = 0;
  rc = al_i2c_async_read(0, 0x1e, NULL, 0, -1, i2c_cb, &ec);
  if (rc == 0) {
    xSemaphoreTake(i2c_done, portMAX_DELAY);
  }
  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST I2C(400k) read: device 1e, rc=%d, ec=%d\r\n",
                     rc, ec);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  // sleep for a while
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // reset SPL06
  i2c_buf[0] = 0x09; // SOFT_RST
  ec = 0;
  rc = al_i2c_async_mem_write(0, spl_addr, NULL, &reset_reg, i2c_buf, 1, -1, i2c_cb, &ec);
  if (rc == 0) {
    xSemaphoreTake(i2c_done, portMAX_DELAY);
  }
  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST I2C(400k) mem write: device %02hhx, rc=%d, ec=%d\r\n",
                     spl_addr, rc, ec);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  last_wake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&last_wake, 5000 / portTICK_PERIOD_MS);

    // read ID register
    ec = 0;
    rc = al_i2c_async_mem_read(0, spl_addr, NULL, &id_reg, i2c_buf, 1, -1, i2c_cb, &ec);
    if (rc == 0) {
      xSemaphoreTake(i2c_done, portMAX_DELAY);
    }

    msg_len = snprintf(msg_buf, sizeof(msg_buf),
                       "TEST I2C(400k) mem read: device %02hhx, data=%02hhx, rc=%d, ec=%d\r\n",
                       spl_addr, i2c_buf[0], rc, ec);
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);
  }
}

void timer_task(void* param) {
  static char msg_buf[512];
  int msg_len;
  TickType_t last_wake;
  TaskHandle_t h_idle;
  UBaseType_t stack_water_mark;
  size_t free_heap_size;

  last_wake = xTaskGetTickCount();

  while (1) {
    msg_len = snprintf(msg_buf, sizeof(msg_buf),
                       "----------\r\n"
                       "new feature: implement al_i2c APIs\r\n"
                       "Stack water marker(word):\r\n");
    for (int i = 0; i < nr_tasks; ++i) {
      stack_water_mark = uxTaskGetStackHighWaterMark(tasks[i]);
      msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len,
                          "%s: %lu\r\n",
                          pcTaskGetName(tasks[i]),
                          stack_water_mark);
    }
    h_idle = xTaskGetIdleTaskHandle();
    stack_water_mark = uxTaskGetStackHighWaterMark(h_idle);
    msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len,
                        "%s: %lu\r\n",
                        pcTaskGetName(h_idle),
                        stack_water_mark);
    free_heap_size = xPortGetFreeHeapSize();
    msg_len += snprintf(msg_buf + msg_len, sizeof(msg_buf) - msg_len,
                        "Free heap size(byte): %zu\r\n"
                        "\r\n",
                        free_heap_size);
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

    vTaskDelayUntil(&last_wake, 10000 / portTICK_PERIOD_MS);
  }
}

// FreeRTOS callbacks
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  al_gpio_set(2, true);
}

// Entry point
int main(void) {
  BaseType_t xReturned = pdPASS;

  // system init
  al_init();

  spi_done = xSemaphoreCreateBinary();
  i2c_done = xSemaphoreCreateBinary();

  // create tasks
  xReturned = xTaskCreate(test_spi,
                          "TEST SPI",
                          192,
                          NULL,
                          3,
                          &tasks[nr_tasks++]);
  configASSERT(xReturned == pdPASS);

  xReturned = xTaskCreate(test_pwm,
                          "TEST PWM",
                          192,
                          &servo_movement_period,
                          1,
                          &tasks[nr_tasks++]);
  configASSERT(xReturned == pdPASS);

  xReturned = xTaskCreate(test_gpio,
                          "TEST GPIO",
                          configMINIMAL_STACK_SIZE,
                          NULL,
                          1,
                          &tasks[nr_tasks++]);
  configASSERT(xReturned == pdPASS);

  xReturned = xTaskCreate(test_adc,
                          "TEST ADC",
                          192,
                          NULL,
                          1,
                          &tasks[nr_tasks++]);
  configASSERT(xReturned == pdPASS);

  xReturned = xTaskCreate(test_i2c,
                          "TEST I2C",
                          192,
                          NULL,
                          1,
                          &tasks[nr_tasks++]);
  configASSERT(xReturned == pdPASS);

  xReturned = xTaskCreate(timer_task,
                          "TIMER TASK",
                          192,
                          NULL,
                          tskIDLE_PRIORITY + 1,
                          &tasks[nr_tasks++]);
  configASSERT(xReturned == pdPASS);

  // start scheduler
  vTaskStartScheduler();

  // should not get here
  return 0;
}
