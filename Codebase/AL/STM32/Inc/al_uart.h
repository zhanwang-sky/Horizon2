//
//  al_uart.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/28.
//

#ifndef AL_UART_H
#define AL_UART_H

// Includes
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Typedefs
typedef void (*al_uart_cb_t)(int, int, void*);

// Function prototypes
void al_uart_init(void);

int al_uart_start_receive(int fd, uint8_t* buf, al_uart_cb_t cb, void* param);

int al_uart_async_send(int fd, const uint8_t* buf, int len,
                       int timeout, al_uart_cb_t cb, void* param);

#ifdef __cplusplus
}
#endif

#endif /* AL_UART_H */
