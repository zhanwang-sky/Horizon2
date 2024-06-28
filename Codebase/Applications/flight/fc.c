//
//  fc.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/27.
//

// Includes
#include "al.h"
#include "sbus_receiver.h"

// Definitions
#define PID_LOOP_PERIOD_MS (20)
#define RX_TIMEOUT_MS      (100)

// Private typedefs
typedef struct {
  float yaw;
  float pitch;
  float roll;
  float throttles[2];
  int arm;
  int flaps;
  int boost;
  // stats
  int wd_rx_cnt; // windowed rx count
} stick_data_t;

typedef struct {
  int sbus_fd;
  SemaphoreHandle_t stick_mtx;
  stick_data_t stick_data;
  // stats
  volatile uint32_t sbus_rx_cnt;
  volatile uint32_t pid_loop_cnt;
} fc_data_t;

// Private variables
static const float inverted_sbus_full_scale = 1.f / SBUS_FULL_SCALE;
static fc_data_t fc_data;

// Experimental functions

// Tasks
void sbus_loop(void* param) {
  fc_data_t* p_fc = (fc_data_t*) param;
  stick_data_t* p_stick = &p_fc->stick_data;
  sbus_receiver_t* p_rc;
  sbus_frame_t frame;
  int rc;

  p_rc = sbus_receiver_create(p_fc->sbus_fd);
  configASSERT(p_rc != NULL);

  while (1) {
    rc = sbus_receiver_poll(p_rc, &frame, -1);
    if (rc != 0) {
      continue;
    }
    // stats
    ++(p_fc->sbus_rx_cnt);
    // update stick positions
    if (xSemaphoreTake(p_fc->stick_mtx, portMAX_DELAY) == pdTRUE) {
      if (!frame.frame_lost && !frame.failsafe) {
        ++(p_stick->wd_rx_cnt);
        p_stick->boost = (frame.channels[6] >= SBUS_MAX_VALUE) ? 1 : 0; // CH7
        p_stick->flaps = (frame.channels[5] >= SBUS_MAX_VALUE) ? 1 :
                        ((frame.channels[5] <= SBUS_MIN_VALUE) ? -1 : 0); // CH6
        p_stick->arm = (frame.channels[4] >= SBUS_MAX_VALUE) ? 1 : 0; // CH5
        p_stick->throttles[0] = ((int) frame.channels[2] - SBUS_MIN_VALUE) * inverted_sbus_full_scale; // CH3
        p_stick->roll = ((int) frame.channels[3] - SBUS_MIN_VALUE) * inverted_sbus_full_scale; // CH4
        p_stick->pitch = ((int) frame.channels[1] - SBUS_MIN_VALUE) * inverted_sbus_full_scale; // CH2
        p_stick->yaw = ((int) frame.channels[0] - SBUS_MIN_VALUE) * inverted_sbus_full_scale; // CH1
      }
      xSemaphoreGive(p_fc->stick_mtx);
    }
  }
}

void pid_loop(void* param) {
  fc_data_t* p_fc = (fc_data_t*) param;
  stick_data_t* p_stick = &p_fc->stick_data;
  TickType_t last_wake;
  // failsafe
  int no_signal_ms = 0;
  int failsafe = 1;
  // stick positions
  int armed = 0;
  int boost = 0;
  int flaps = 0;
  float rudder = 0.5f;
  float elevator = 0.5f;
  float throttle = 0.f;
  float aileron = 0.5f;

  last_wake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&last_wake, PID_LOOP_PERIOD_MS / portTICK_PERIOD_MS);
    // stats
    ++(p_fc->pid_loop_cnt);
    // get stick positions
    if (xSemaphoreTake(p_fc->stick_mtx, portMAX_DELAY) == pdTRUE) {
      // failsafe
      if (p_stick->wd_rx_cnt == 0) {
        if (no_signal_ms < RX_TIMEOUT_MS) {
          no_signal_ms += PID_LOOP_PERIOD_MS;
        }
        if (no_signal_ms >= RX_TIMEOUT_MS) {
          failsafe = 1;
        }
      } else {
        p_stick->wd_rx_cnt = 0;
        no_signal_ms = 0;
        failsafe = 0;
      }
      // stick positions
      armed = p_stick->arm;
      boost = p_stick->boost;
      rudder = p_stick->yaw;
      elevator = p_stick->pitch;
      throttle = p_stick->throttles[0];
      aileron = p_stick->roll;
      flaps = p_stick->flaps;

      xSemaphoreGive(p_fc->stick_mtx);
    }
    // smooth
  }
}

// Functions
void fc_init(int fd) {
  BaseType_t ret = pdFAIL;

  fc_data.sbus_fd = fd;
  fc_data.stick_mtx = xSemaphoreCreateMutex();
  configASSERT(fc_data.stick_mtx != NULL);

  ret = xTaskCreate(sbus_loop,
                    "SBUS_LOOP",
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
