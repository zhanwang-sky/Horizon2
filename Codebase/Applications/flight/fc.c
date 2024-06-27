//
//  fc.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/27.
//

// Includes
#include "al.h"
#include "sbus_rx.h"

// Typedefs
typedef struct {
  float yaw;
  float pitch;
  float roll;
  float throttles[2];
  int arm;
  int flaps;
  int boost;
  // status
  int frame_lost;
  int failsafe;
  // private
  int wd_rx_cnt; // windowed rx count
  SemaphoreHandle_t mtx;
} stick_data_t;

typedef struct {
  int sbus_fd;
  volatile uint32_t sbus_rx_cnt;
  volatile uint32_t pid_loop_cnt;
  stick_data_t stick_data;
} fc_data_t;

// Private variables
static const float inverted_sbus_half_scale = 2.f / SBUS_FULL_SCALE;

// Tasks
void sbus_receiver(void* param) {
  fc_data_t* p_fc = (fc_data_t*) param;
  stick_data_t* p_stick = &p_fc->stick_data;
  sbus_frame_t frame;
  int rc;

  rc = sbus_rx_init(p_fc->sbus_fd);
  configASSERT(rc == 0);

  while (1) {
    rc = sbus_rx_poll(&frame, -1);
    if (rc != 0) {
      continue;
    }
    ++(p_fc->sbus_rx_cnt);
    if (xSemaphoreTake(p_stick->mtx, portMAX_DELAY) == pdTRUE) {
      ++(p_stick->wd_rx_cnt);
      p_stick->failsafe = frame.failsafe ? 1 : 0;
      p_stick->frame_lost = frame.frame_lost ? 1 : 0;
      p_stick->boost = (frame.channels[6] >= SBUS_MAX_VALUE) ? 1 : 0; // CH7
      p_stick->flaps = (frame.channels[5] >= SBUS_MAX_VALUE) ? 1 :
                       ((frame.channels[5] <= SBUS_MIN_VALUE) ? -1 : 0); // CH6
      p_stick->arm = (frame.channels[4] >= SBUS_MAX_VALUE) ? 1 : 0; // CH5
      p_stick->throttles[0] = ((int) frame.channels[2] - SBUS_NEUTRAL_VALUE) * inverted_sbus_half_scale; // CH3
      p_stick->roll = ((int) frame.channels[3] - SBUS_NEUTRAL_VALUE) * inverted_sbus_half_scale; // CH4
      p_stick->pitch = ((int) frame.channels[1] - SBUS_NEUTRAL_VALUE) * inverted_sbus_half_scale; // CH2
      p_stick->yaw = ((int) frame.channels[0] - SBUS_NEUTRAL_VALUE) * inverted_sbus_half_scale; // CH1
      xSemaphoreGive(p_stick->mtx);
    }
  }
}

void pid_loop(void* param) {
  fc_data_t* p_fc = (fc_data_t*) param;
  stick_data_t* p_stick = &p_fc->stick_data;
  TickType_t last_wake;

  last_wake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&last_wake, 20 / portTICK_PERIOD_MS);
    if (xSemaphoreTake(p_stick->mtx, portMAX_DELAY) == pdTRUE) {
      ++(p_fc->pid_loop_cnt);
      p_stick->wd_rx_cnt = 0;
      xSemaphoreGive(p_stick->mtx);
    }
  }
}

// Functions
void fc_init(int fd) {
  static fc_data_t fc_data;
  BaseType_t ret = pdFAIL;

  fc_data.sbus_fd = fd;
  fc_data.stick_data.mtx = xSemaphoreCreateMutex();
  configASSERT(fc_data.stick_data.mtx != NULL);

  ret = xTaskCreate(sbus_receiver,
                    "SBUS_RECEIVER",
                    2 * configMINIMAL_STACK_SIZE,
                    &fc_data,
                    tskIDLE_PRIORITY + 2,
                    NULL);
  configASSERT(ret == pdPASS);

  ret = xTaskCreate(pid_loop,
                    "PID_LOOP",
                    2 * configMINIMAL_STACK_SIZE,
                    &fc_data,
                    tskIDLE_PRIORITY + 1,
                    NULL);
  configASSERT(ret == pdPASS);
}
