//
//  sbus_receiver.h
//  sbus
//
//  Created by zhanwang-sky on 2024/6/24.
//

#ifndef SBUS_RECEIVER_H
#define SBUS_RECEIVER_H

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
} sbus_receiver_t;

// Function prototypes
sbus_receiver_t* sbus_receiver_create(int fd);

int sbus_receiver_poll(sbus_receiver_t* p_rc, sbus_frame_t* p_frame, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_RECEIVER_H */
