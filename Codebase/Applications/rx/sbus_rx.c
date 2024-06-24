//
//  sbus_rx.c
//  sbus
//
//  Created by zhanwang-sky on 2024/6/24.
//

// Includes
#include "sbus_rx.h"

// Private variables
static SemaphoreHandle_t sbus_rx_semphr;
static sbus_frame_t sbus_rx_frame;
static sbus_context_t sbus_rx_ctxarr[2];
static sbus_context_t* sbus_rx_task_ctxptr = &sbus_rx_ctxarr[0];
static sbus_context_t* sbus_rx_isr_ctxptr = &sbus_rx_ctxarr[1];
static TickType_t sbus_rx_prev_tick;

// Private functions
static void sbus_rx_cb(int fd, int ec, void* param) {
  TickType_t curr_tick;
  uint8_t data;

  if (ec) {
    sbus_reset_context(sbus_rx_isr_ctxptr);
    return;
  }

  curr_tick = xTaskGetTickCountFromISR();
  if ((curr_tick - sbus_rx_prev_tick) > (3 / portTICK_PERIOD_MS)) {
    sbus_reset_context(sbus_rx_isr_ctxptr);
  }
  sbus_rx_prev_tick = curr_tick;

  data = (uint8_t) ((uintptr_t) param);
  if (sbus_receive_data(sbus_rx_isr_ctxptr, data)) {
    BaseType_t should_yield = pdFALSE;
    sbus_context_t* ctxptr = sbus_rx_isr_ctxptr;
    sbus_rx_isr_ctxptr = sbus_rx_task_ctxptr;
    sbus_rx_task_ctxptr = ctxptr;
    xSemaphoreGiveFromISR(sbus_rx_semphr, &should_yield);
    portYIELD_FROM_ISR(should_yield);
  }
}

// Functions
int sbus_rx_init(int fd) {
  if (!(sbus_rx_semphr = xSemaphoreCreateBinary())) {
    return -1;
  }

  if (al_uart_start_receive(fd, sbus_rx_cb) != 0) {
    return -1;
  }

  return 0;
}

int sbus_rx_poll(const sbus_frame_t** pp_frame, int timeout) {
  TickType_t ticks2wait = (timeout < 0) ? portMAX_DELAY : (timeout / portTICK_PERIOD_MS);

  if (xSemaphoreTake(sbus_rx_semphr, ticks2wait) != pdTRUE) {
    return -1;
  }

  if (pp_frame) {
    sbus_unpack_frame(sbus_rx_task_ctxptr, &sbus_rx_frame);
    *pp_frame = &sbus_rx_frame;
  }

  return 0;
}
