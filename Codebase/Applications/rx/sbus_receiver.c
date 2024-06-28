//
//  sbus_receiver.c
//  sbus
//
//  Created by zhanwang-sky on 2024/6/24.
//

// Includes
#include "sbus_receiver.h"

// Private functions
static void sbus_receiver_rx_cb(int fd, int ec, void* param) {
  sbus_receiver_t* p_rc = (sbus_receiver_t*) param;
  TickType_t curr_tick;

  if (ec) {
    sbus_reset_context(p_rc->isr_ctx_ptr);
    return;
  }

  curr_tick = xTaskGetTickCountFromISR();
  if ((curr_tick - p_rc->prev_tick) > (3 / portTICK_PERIOD_MS)) {
    sbus_reset_context(p_rc->isr_ctx_ptr);
  }
  p_rc->prev_tick = curr_tick;

  if (sbus_receive_data(p_rc->isr_ctx_ptr, p_rc->rx_data)) {
    BaseType_t should_yield = pdFALSE;
    sbus_context_t* ctx_ptr = p_rc->isr_ctx_ptr;
    p_rc->isr_ctx_ptr = p_rc->task_ctx_ptr;
    p_rc->task_ctx_ptr = ctx_ptr;
    xSemaphoreGiveFromISR(p_rc->semphr, &should_yield);
    portYIELD_FROM_ISR(should_yield);
  }
}

// Functions
sbus_receiver_t* sbus_receiver_create(int fd) {
  SemaphoreHandle_t semphr;
  sbus_receiver_t* p_rc;
  int rc = 0;

  semphr = xSemaphoreCreateBinary();
  configASSERT(semphr != NULL);

  p_rc = pvPortMalloc(sizeof(sbus_receiver_t));
  configASSERT(p_rc != NULL);

  p_rc->semphr = semphr;
  p_rc->prev_tick = 0;
  sbus_reset_context(&p_rc->ctxs[0]);
  sbus_reset_context(&p_rc->ctxs[1]);
  p_rc->task_ctx_ptr = &p_rc->ctxs[0];
  p_rc->isr_ctx_ptr = &p_rc->ctxs[1];

  rc = al_uart_start_receive(fd, &p_rc->rx_data, sbus_receiver_rx_cb, p_rc);
  configASSERT(rc == 0);

  return p_rc;
}

int sbus_receiver_poll(sbus_receiver_t* p_rc, sbus_frame_t* p_frame, int timeout) {
  TickType_t ticks2wait;

  // sanity check
  if (!p_rc) {
    return -1;
  }

  // convert ms to ticks
  if (timeout < 0) {
    ticks2wait = portMAX_DELAY;
  } else {
    ticks2wait = timeout / portTICK_PERIOD_MS;
  }

  // wait
  if (xSemaphoreTake(p_rc->semphr, ticks2wait) != pdTRUE) {
    return -1;
  }

  // unpack
  if (p_frame) {
    sbus_unpack_frame(p_rc->task_ctx_ptr, p_frame);
  }

  return 0;
}
