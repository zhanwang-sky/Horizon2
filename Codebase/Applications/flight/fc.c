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
// rudder
#define RUDDER_NEUTRAL_US    (1500)
#define RUDDER_MIN_US        (1200)
#define RUDDER_MAX_US        (1800)
#define RUDDER_REVERSED      (0)
// elevator
#define ELEVATOR_NEUTRAL_US  (1600)
#define ELEVATOR_MIN_US      (1300)
#define ELEVATOR_MAX_US      (1900)
#define ELEVATOR_REVERSED    (1)
// throttle
#define THROTTLE_MIN_US      (1000)
#define THROTTLE_MAX_US      (2000)
#define THROTTLE_REVERSED    (0)
// ailerons
#define AILERON_L_NEUTRAL_US (1475)
#define AILERON_L_MIN_US     (1175)
#define AILERON_L_MAX_US     (1775)
#define AILERON_L_REVERSED   (1)
#define AILERON_R_NEUTRAL_US (1500)
#define AILERON_R_MIN_US     (1200)
#define AILERON_R_MAX_US     (1800)
#define AILERON_R_REVERSED   (1)

inline int calculate_pwm(float x, int min_us, int max_us, int reversed) {
  int scale_us = max_us - min_us;
  int result = min_us;
  if (reversed) { x = 1.f - x; }
  result += x * scale_us;
  return (result < min_us) ? min_us : ((result > max_us) ? max_us : result);
}

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

  last_wake = xTaskGetTickCount();
  while (1) {
    // buttons
    int armed;
    int boost;
    // throttle
    float throttle;
    // surfaces
    float rudder;
    float elevator;
    float aileron;
    int flaps;
    // PWMs
    int rudder_pwm;
    int elevator_pwm;
    int throttle_pwm;
    int aileronL_pwm;
    int aileronR_pwm;

    // XXX TODO: remove
    // failsafe indicator
    if (failsafe) {
      if (p_fc->pid_loop_cnt % 5 == 0) {
        bool state;
        al_gpio_get(2, &state);
        al_gpio_set(2, !state);
      }
    } else {
      al_gpio_set(2, false);
    }
    // feed
    al_wdog_feed();

    vTaskDelayUntil(&last_wake, PID_LOOP_PERIOD_MS / portTICK_PERIOD_MS);

    // stats
    ++(p_fc->pid_loop_cnt);

    // get stick positions
    if (xSemaphoreTake(p_fc->stick_mtx, portMAX_DELAY) == pdTRUE) {
      // update failsafe state
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
      throttle = p_stick->throttles[0];
      rudder = p_stick->yaw;
      elevator = p_stick->pitch;
      aileron = p_stick->roll;
      flaps = p_stick->flaps;
      xSemaphoreGive(p_fc->stick_mtx);
    } else {
      // update failsafe state
      no_signal_ms = RX_TIMEOUT_MS;
      failsafe = 1;
    }

    // Mixing
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
      if (flaps < 0) {
        flaps = 0;
      }
    } else {
      aileron = aileron * 0.6f + 0.2f;
    }

    // PWMs
    rudder_pwm = calculate_pwm(rudder, RUDDER_MIN_US, RUDDER_MAX_US, RUDDER_REVERSED);
    elevator_pwm = calculate_pwm(elevator, ELEVATOR_MIN_US, ELEVATOR_MAX_US, ELEVATOR_REVERSED);
    throttle_pwm = calculate_pwm(throttle, THROTTLE_MIN_US, THROTTLE_MAX_US, THROTTLE_REVERSED);
    if (flaps > 0) {
      aileronL_pwm = calculate_pwm(aileron + 0.2f, AILERON_L_MIN_US, AILERON_L_MAX_US, AILERON_L_REVERSED);
      aileronR_pwm = calculate_pwm(aileron - 0.2f, AILERON_R_MIN_US, AILERON_R_MAX_US, AILERON_R_REVERSED);
    } else if (flaps < 0) {
      aileronL_pwm = calculate_pwm(aileron - 0.1f, AILERON_L_MIN_US, AILERON_L_MAX_US, AILERON_L_REVERSED);
      aileronR_pwm = calculate_pwm(aileron + 0.1f, AILERON_R_MIN_US, AILERON_R_MAX_US, AILERON_R_REVERSED);
    } else {
      aileronL_pwm = calculate_pwm(aileron, AILERON_L_MIN_US, AILERON_L_MAX_US, AILERON_L_REVERSED);
      aileronR_pwm = calculate_pwm(aileron, AILERON_R_MIN_US, AILERON_R_MAX_US, AILERON_R_REVERSED);
    }

    al_pwm_write(0, rudder_pwm);
    al_pwm_write(1, elevator_pwm);
    al_pwm_write(2, aileronL_pwm);
    al_pwm_write(3, aileronR_pwm);
    al_pwm_write(4, throttle_pwm);
    // for ESC signal calibration
    al_pwm_write(5, (!failsafe && armed) ? THROTTLE_MAX_US : THROTTLE_MIN_US);
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
