//
//  unit_test.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/18.
//

// Includes
#include <math.h>
#include <stdio.h>
#include "al.h"

#if defined(UNIT_TEST)

// Global variables
volatile uint32_t uart_intr_cnt[10];
volatile uint32_t uart_dmatx_intr_cnt[10];

volatile uint32_t i2c_ev_intr_cnt[10];
volatile uint32_t i2c_er_intr_cnt[10];
volatile uint32_t i2c_dmarx_intr_cnt[10];
volatile uint32_t i2c_dmatx_intr_cnt[10];

volatile uint32_t spi_intr_cnt[10];
volatile uint32_t spi_dmatx_intr_cnt[10];
volatile uint32_t spi_dmarx_intr_cnt[10];

// Private variables
static SemaphoreHandle_t uart_done;
static SemaphoreHandle_t i2c_done;
static SemaphoreHandle_t spi_done;

static volatile uint32_t uart_rx_cnt;
static volatile uint32_t uart_rx_err;
static volatile uint8_t uart_last_recv;

// Callbacks
static void uart_rx_cb(int fd, int ec, void* param) {
  uart_last_recv = (uint8_t) ((uintptr_t) param);
  if (ec) {
    ++uart_rx_err;
  }
  ++uart_rx_cnt;
}

static void uart_tx_cb(int fd, int ec, void* param) {
  int* p_ec = (int*) param;
  BaseType_t should_yield = pdFALSE;
  if (p_ec) {
    *p_ec = ec;
  }
  if (uart_done) {
    xSemaphoreGiveFromISR(uart_done, &should_yield);
    portYIELD_FROM_ISR(should_yield);
  }
}

static void i2c_cb(int fd, int ec, void* param) {
  int* p_ec = (int*) param;
  BaseType_t should_yield = pdFALSE;
  if (p_ec) {
    *p_ec = ec;
  }
  if (i2c_done) {
    xSemaphoreGiveFromISR(i2c_done, &should_yield);
    portYIELD_FROM_ISR(should_yield);
  }
}

static void spi_cb(int fd, int ec, void* param) {
  int* p_ec = (int*) param;
  BaseType_t should_yield = pdFALSE;
  if (p_ec) {
    *p_ec = ec;
  }
  if (spi_done) {
    xSemaphoreGiveFromISR(spi_done, &should_yield);
    portYIELD_FROM_ISR(should_yield);
  }
}

// Tasks
static void test_spi(void* param) {
  // ICM-42688-P registers
  static const uint8_t DEVICE_CONFIG = 0x11;
  static const uint8_t TEMP_DATA1 = 0x1d;
  static const uint8_t PWR_MGMT0 = 0x4e;
  static const uint8_t ACCEL_CONFIG0 = 0x50;
  static const uint8_t WHO_AM_I = 0x75;

  static char msg_buf[192];
  static uint8_t spi_buf[4];

  int16_t temp_data;
  float temp;
  int rc;
  int ec;
  int msg_len;
  TickType_t last_wake;

  // power-up delay
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // reset
  spi_buf[0] = DEVICE_CONFIG;
  spi_buf[1] = 0x01; // Enable reset
  ec = 0;
  rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 2, -1, spi_cb, &ec);
  if (rc == 0) {
    xSemaphoreTake(spi_done, portMAX_DELAY);
  }

  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST_SPI[spi_intr=%u dmatx=%u dmarx=%u]: reset device, rc=%d, ec=%d\r\n",
                     spi_intr_cnt[0], spi_dmatx_intr_cnt[0], spi_dmarx_intr_cnt[0],
                     rc, ec);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  // sleep for a while
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // read WHO_AM_I
  spi_buf[0] = 0x80 | WHO_AM_I;
  spi_buf[1] = 0;
  ec = 0;
  rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 2, -1, spi_cb, &ec);
  if (rc == 0) {
    xSemaphoreTake(spi_done, portMAX_DELAY);
  }

  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST_SPI[spi_intr=%u dmatx=%u dmarx=%u]: who_am_i=%02hhx, rc=%d, ec=%d\r\n",
                     spi_intr_cnt[0], spi_dmatx_intr_cnt[0], spi_dmarx_intr_cnt[0],
                     spi_buf[1], rc, ec);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  // sleep for a while
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // enable accel
  spi_buf[0] = PWR_MGMT0;
  spi_buf[1] = 0x02; // Places accelerometer in Low Power (LP) Mode
  ec = 0;
  rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 2, -1, spi_cb, &ec);
  if (rc == 0) {
    xSemaphoreTake(spi_done, portMAX_DELAY);
  }

  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST_SPI[spi_intr=%u dmatx=%u dmarx=%u]: enable accel, rc=%d, ec=%d\r\n",
                     spi_intr_cnt[0], spi_dmatx_intr_cnt[0], spi_dmarx_intr_cnt[0],
                     rc, ec);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  // sleep for a while
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // config accel
  spi_buf[0] = ACCEL_CONFIG0;
  spi_buf[1] = 0x0e; // 1.5625Hz (LP mode)
  ec = 0;
  rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 2, -1, spi_cb, &ec);
  if (rc == 0) {
    xSemaphoreTake(spi_done, portMAX_DELAY);
  }

  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST_SPI[spi_intr=%u dmatx=%u dmarx=%u]: config accel, rc=%d, ec=%d\r\n",
                     spi_intr_cnt[0], spi_dmatx_intr_cnt[0], spi_dmarx_intr_cnt[0],
                     rc, ec);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  // sleep for a while
  vTaskDelay(200 / portTICK_PERIOD_MS);

  last_wake = xTaskGetTickCount();
  while (1) {
    // read temp
    spi_buf[0] = 0x80 | TEMP_DATA1;
    spi_buf[1] = 0;
    spi_buf[2] = 0;
    ec = 0;
    rc = al_spi_async_read_write(0, 0, spi_buf, spi_buf, 3, -1, spi_cb, &ec);
    if (rc == 0) {
      xSemaphoreTake(spi_done, portMAX_DELAY);
    }

    temp_data = (int16_t) ((((uint16_t) spi_buf[1]) << 8) | ((uint16_t) spi_buf[2]));
    temp = ((float) temp_data / 132.48f) + 25.f;

    msg_len = snprintf(msg_buf, sizeof(msg_buf),
                       "TEST_SPI[spi_intr=%u dmatx=%u dmarx=%u]: temp=%f(%hd), rc=%d, ec=%d\r\n",
                       spi_intr_cnt[0], spi_dmatx_intr_cnt[0], spi_dmarx_intr_cnt[0],
                       temp, temp_data, rc, ec);
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

    vTaskDelayUntil(&last_wake, 5000 / portTICK_PERIOD_MS);
  }
}

static void test_i2c(void* param) {
  // Device: Goertek SPL06
  static const uint8_t spl_addr = 0x76;
  static const uint8_t reset_reg = 0x0c;
  static const uint8_t id_reg = 0x0d;
  static char msg_buf[192];
  static uint8_t i2c_buf[4];
  int rc;
  int ec;
  int msg_len;
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
                     "TEST_I2C(400k)[ev_intr=%u er_intr=%u dmarx=%u dmatx=%u]: write(device=%02hhx), rc=%d, ec=%d\r\n",
                     i2c_ev_intr_cnt[0], i2c_er_intr_cnt[0], i2c_dmarx_intr_cnt[0], i2c_dmatx_intr_cnt[0],
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
                     "TEST_I2C(400k)[ev_intr=%u er_intr=%u dmarx=%u dmatx=%u]: read(device=1e), rc=%d, ec=%d\r\n",
                     i2c_ev_intr_cnt[0], i2c_er_intr_cnt[0], i2c_dmarx_intr_cnt[0], i2c_dmatx_intr_cnt[0],
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
                     "TEST_I2C(400k)[ev_intr=%u er_intr=%u dmarx=%u dmatx=%u]: mem_write(device=%02hhx, mem_addr=%02hhx), rc=%d, ec=%d\r\n",
                     i2c_ev_intr_cnt[0], i2c_er_intr_cnt[0], i2c_dmarx_intr_cnt[0], i2c_dmatx_intr_cnt[0],
                     spl_addr, reset_reg, rc, ec);
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
                       "TEST_I2C(400k)[ev_intr=%u er_intr=%u dmarx=%u dmatx=%u]: mem_read(device=%02hhx, mem_addr=%02hhx), data=%02hhx, rc=%d, ec=%d\r\n",
                       i2c_ev_intr_cnt[0], i2c_er_intr_cnt[0], i2c_dmarx_intr_cnt[0], i2c_dmatx_intr_cnt[0],
                       spl_addr, id_reg, i2c_buf[0], rc, ec);
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);
  }
}

static void test_uart(void* param) {
  static char msg_buf[192];
  static char data_buf[64];
  int fd = *((int*) param);
  int rc;
  int ec;
  int msg_len;
  int data_len;
  TickType_t last_wake;

  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST_UART(fd=%d)[uart_intr=%u dmatx=%u]: task started\r\n",
                     fd, uart_intr_cnt[fd], uart_dmatx_intr_cnt[fd]);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  vTaskDelay(3000 / portTICK_PERIOD_MS);

  rc = al_uart_start_receive(fd, uart_rx_cb);

  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST_UART(fd=%d)[uart_intr=%u dmatx=%u]: start reception, rc=%d\r\n",
                     fd, uart_intr_cnt[fd], uart_dmatx_intr_cnt[fd],
                     rc);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  // sleep for a while
  vTaskDelay(200 / portTICK_PERIOD_MS);

  last_wake = xTaskGetTickCount();
  while (1) {
    data_len = snprintf(data_buf, sizeof(data_buf),
                        "rx_cnt=%u rx_err=%u last_recv=%02hhx",
                        uart_rx_cnt, uart_rx_err, uart_last_recv);
    ec = 0;
    rc = al_uart_async_send(fd, (const uint8_t*) data_buf, data_len,
                            -1, uart_tx_cb, &ec);
    if (rc == 0) {
      xSemaphoreTake(uart_done, portMAX_DELAY);
    }

    msg_len = snprintf(msg_buf, sizeof(msg_buf),
                       "TEST_UART(fd=%d)[uart_intr=%u dmatx=%u]: data sent, rc=%d, ec=%d, len=%d, content={%s}\r\n",
                       fd, uart_intr_cnt[fd], uart_dmatx_intr_cnt[fd],
                       rc, ec, data_len, data_buf);
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

    vTaskDelayUntil(&last_wake, 1000 / portTICK_PERIOD_MS);
  }
}

static void test_pwm(void* param) {
  static char msg_buf[128];
  static const float PI = 3.141593f;
  static const int PWM_FREQ = 50;
  float* p_period = (float*) param;
  float period = (p_period != NULL) ? *p_period : 5.f;
  float cycles = PWM_FREQ * period;
  float delta = 2 * PI / cycles;
  int msg_len;
  TickType_t last_wake;

  msg_len = snprintf(msg_buf, sizeof(msg_buf),
                     "TEST_PWM: period=%f cycles=%f delta=%f\r\n",
                     period, cycles, delta);
  al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);

  last_wake = xTaskGetTickCount();
  while (1) {
    for (int i = 0; i < (int) roundf(cycles); ++i) {
      float arc = (float) i * delta;
      int pulse1 = (int) roundf(500.f * cosf(arc));
      al_pwm_write(0, 1500 + pulse1);
      al_pwm_write(1, 1350 + pulse1);
      al_pwm_write(2, 1050 + pulse1);
      al_pwm_write(3, 1850 + pulse1);
      vTaskDelayUntil(&last_wake, 1000 / PWM_FREQ / portTICK_PERIOD_MS);
    }
  }
}

static void test_adc(void* param) {
  static char msg_buf[128];
  int msg_len;
  float temp;
  float vbat;
  int temp_raw;
  int curr_raw;
  TickType_t last_wake;

  last_wake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&last_wake, 1000 / portTICK_PERIOD_MS);

    al_analog_read_raw(0, &temp, &temp_raw);
    al_analog_read(1, &vbat);
    al_analog_read_raw(2, NULL, &curr_raw);
    msg_len = snprintf(msg_buf, sizeof(msg_buf),
                       "TEST_ADC: temp=%.0f(%d) vbat=%.3f curr_raw=%d\r\n",
                       temp, temp_raw, vbat, curr_raw);
    al_uart_async_send(1, (const uint8_t*) msg_buf, msg_len, -1, NULL, NULL);
  }
}

// Functions
void unit_test(void) {
  static int uart_test_fd = 2;
  static float servo_period = 3.f;
  BaseType_t ret = pdFAIL;

  // crate semaphores
  uart_done = xSemaphoreCreateBinary();
  i2c_done = xSemaphoreCreateBinary();
  spi_done = xSemaphoreCreateBinary();

  // create tasks
  ret = xTaskCreate(test_spi,
                    "TEST_SPI",
                    2 * configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY + 4,
                    NULL);
  configASSERT(ret == pdPASS);

  ret = xTaskCreate(test_i2c,
                    "TEST_I2C",
                    2 * configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY + 3,
                    NULL);
  configASSERT(ret == pdPASS);

  ret = xTaskCreate(test_uart,
                    "TEST_UART",
                    2 * configMINIMAL_STACK_SIZE,
                    &uart_test_fd,
                    tskIDLE_PRIORITY + 2,
                    NULL);
  configASSERT(ret == pdPASS);

  ret = xTaskCreate(test_pwm,
                    "TEST_PWM",
                    2 * configMINIMAL_STACK_SIZE,
                    &servo_period,
                    tskIDLE_PRIORITY + 1,
                    NULL);
  configASSERT(ret == pdPASS);

  ret = xTaskCreate(test_adc,
                    "TEST_ADC",
                    2 * configMINIMAL_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY + 1,
                    NULL);
  configASSERT(ret == pdPASS);
}

#endif /* UNIT_TEST */
