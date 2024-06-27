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

// Function prototypes
int sbus_rx_init(int fd);

int sbus_rx_poll(sbus_frame_t* p_frame, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* SBUS_RX_H */
