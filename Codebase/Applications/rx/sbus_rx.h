//
//  sbus_rx.h
//  sbus
//
//  Created by zhanwang-sky on 2024/6/24.
//

#ifndef SBUS_RX_H
#define SBUS_RX_H

// Includes
#include "al.h"
#include "sbus.h"

#ifdef __cplusplus
extern "C" {
#endif

// Typedefs
typedef struct {
  SemaphoreHandle_t semphr;
  TickType_t prev_tick;
  sbus_context_t ctxs[2];
  sbus_context_t* task_ctx_ptr;
  sbus_context_t* isr_ctx_ptr;
  uint8_t rx_data;
} sbus_rx_ctx_t;

// Function prototypes
sbus_rx_ctx_t* sbus_rx_init(int fd);

int sbus_rx_poll(sbus_rx_ctx_t* p_ctx, sbus_frame_t* p_frame, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_RX_H */
