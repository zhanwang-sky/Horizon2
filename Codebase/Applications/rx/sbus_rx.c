//
//  sbus_rx.c
//  sbus
//
//  Created by zhanwang-sky on 2024/6/24.
//

// Includes
#include "sbus_rx.h"

// Private functions
static void sbus_rx_cb(int fd, int ec, void* param) {
  sbus_rx_ctx_t* p_ctx = (sbus_rx_ctx_t*) param;
  TickType_t curr_tick;

  if (ec) {
    sbus_reset_context(p_ctx->isr_ctx_ptr);
    return;
  }

  curr_tick = xTaskGetTickCountFromISR();
  if ((curr_tick - p_ctx->prev_tick) > (3 / portTICK_PERIOD_MS)) {
    sbus_reset_context(p_ctx->isr_ctx_ptr);
  }
  p_ctx->prev_tick = curr_tick;

  if (sbus_receive_data(p_ctx->isr_ctx_ptr, p_ctx->rx_data)) {
    BaseType_t should_yield = pdFALSE;
    sbus_context_t* ctx_ptr = p_ctx->isr_ctx_ptr;
    p_ctx->isr_ctx_ptr = p_ctx->task_ctx_ptr;
    p_ctx->task_ctx_ptr = ctx_ptr;
    xSemaphoreGiveFromISR(p_ctx->semphr, &should_yield);
    portYIELD_FROM_ISR(should_yield);
  }
}

// Functions
sbus_rx_ctx_t* sbus_rx_init(int fd) {
  SemaphoreHandle_t semphr;
  sbus_rx_ctx_t* p_ctx;
  int rc = 0;

  semphr = xSemaphoreCreateBinary();
  configASSERT(semphr != NULL);

  p_ctx = pvPortMalloc(sizeof(sbus_rx_ctx_t));
  configASSERT(p_ctx != NULL);

  p_ctx->semphr = semphr;
  p_ctx->prev_tick = 0;
  sbus_reset_context(&p_ctx->ctxs[0]);
  sbus_reset_context(&p_ctx->ctxs[1]);
  p_ctx->task_ctx_ptr = &p_ctx->ctxs[0];
  p_ctx->isr_ctx_ptr = &p_ctx->ctxs[1];

  rc = al_uart_start_receive(fd, &p_ctx->rx_data, sbus_rx_cb, p_ctx);
  configASSERT(rc == 0);

  return p_ctx;
}

int sbus_rx_poll(sbus_rx_ctx_t* p_ctx, sbus_frame_t* p_frame, int timeout) {
  TickType_t ticks2wait;

  // sanity check
  if (!p_ctx) {
    return -1;
  }

  // convert ms to ticks
  if (timeout < 0) {
    ticks2wait = portMAX_DELAY;
  } else {
    ticks2wait = timeout / portTICK_PERIOD_MS;
  }

  // wait
  if (xSemaphoreTake(p_ctx->semphr, ticks2wait) != pdTRUE) {
    return -1;
  }

  // unpack
  if (p_frame) {
    sbus_unpack_frame(p_ctx->task_ctx_ptr, p_frame);
  }

  return 0;
}
