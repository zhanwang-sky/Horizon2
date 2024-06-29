//
//  fc.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/27.
//

// Includes
#include <string.h>
#include "al.h"
#include "sbus_receiver.h"

// Definitions
#define PID_LOOP_PERIOD_MS (20)
#define RX_TIMEOUT_MS      (100)
#define BOOST_TIME_MS      (20000)

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
  // boost
  int boost_activated = 0;
  int boost_time_ms = 0;
  // buttons
  int armed = 0;
  int boost = 0;
  // surfaces
  float rudder = 0.5f;
  float elevator = 0.5f;
  float throttle = 0.f;
  float aileron = 0.5f;
  float flaps = 0.f;

  last_wake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&last_wake, PID_LOOP_PERIOD_MS / portTICK_PERIOD_MS);
    // stats
    ++(p_fc->pid_loop_cnt);

    // get stick positions
    if (xSemaphoreTake(p_fc->stick_mtx, portMAX_DELAY) == pdTRUE) {
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
        // stick positions
        armed = p_stick->arm;
        boost = p_stick->boost;
        rudder = p_stick->yaw;
        elevator = p_stick->pitch;
        throttle = p_stick->throttles[0];
        aileron = p_stick->roll;
        flaps = p_stick->flaps;
      }
      xSemaphoreGive(p_fc->stick_mtx);
    } else {
      no_signal_ms = RX_TIMEOUT_MS;
      failsafe = 1;
    }

    // logic mixer
    // boost
    if (!failsafe && armed && boost && throttle > 0.95f) {
      boost_activated = 1;
      boost_time_ms = 0;
    } else if (failsafe || !armed || throttle < 0.9f || boost_time_ms >= BOOST_TIME_MS) {
      boost_activated = 0;
    }
    // throttle
    if (failsafe || !armed) {
      throttle = 0.f;
    } else if (boost_activated) {
      throttle = 1.f;
      boost_time_ms += PID_LOOP_PERIOD_MS;
    } else {
      throttle *= 0.8f;
    }
    // surfaces
    if (failsafe) {
      rudder = 0.5f;
      elevator = 0.5f;
      aileron = 0.5f;
      if (flaps < 0.f) {
        flaps = 0.f;
      }
    }
  }
}

// Functions
void fc_init(int fd) {
  fc_data_t* p_fc;
  SemaphoreHandle_t mtx;
  BaseType_t ret = pdFAIL;

  p_fc = pvPortMalloc(sizeof(fc_data_t));
  configASSERT(p_fc != NULL);

  mtx = xSemaphoreCreateMutex();
  configASSERT(mtx != NULL);

  p_fc->sbus_fd = fd;
  p_fc->stick_mtx = mtx;
  memset(&p_fc->stick_data, 0, sizeof(stick_data_t));
  p_fc->sbus_rx_cnt = 0;
  p_fc->pid_loop_cnt = 0;

  ret = xTaskCreate(sbus_loop,
                    "SBUS_LOOP",
                    2 * configMINIMAL_STACK_SIZE,
                    p_fc,
                    tskIDLE_PRIORITY + 2,
                    NULL);
  configASSERT(ret == pdPASS);

  ret = xTaskCreate(pid_loop,
                    "PID_LOOP",
                    2 * configMINIMAL_STACK_SIZE,
                    p_fc,
                    tskIDLE_PRIORITY + 1,
                    NULL);
  configASSERT(ret == pdPASS);
}
