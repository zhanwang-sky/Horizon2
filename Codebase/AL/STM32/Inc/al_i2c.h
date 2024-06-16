//
//  al_i2c.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/16.
//

#ifndef AL_I2C_H
#define AL_I2C_H

// Includes
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Typedefs
typedef void (*al_i2c_cb_t)(int, int, void*);

// Function prototypes
void al_i2c_init(void);

int al_i2c_async_read(int fd, uint8_t dev_addr, uint8_t* buf, int len,
                      int timeout, al_i2c_cb_t cb, void* param);
int al_i2c_async_write(int fd, uint8_t dev_addr, uint8_t* buf, int len,
                       int timeout, al_i2c_cb_t cb, void* param);

int al_i2c_async_mem_read(int fd, uint8_t dev_addr,
                          const uint8_t* mem_addr_h, const uint8_t* mem_addr_l,
                          uint8_t* buf, int len,
                          int timeout, al_i2c_cb_t cb, void* param);
int al_i2c_async_mem_write(int fd, uint8_t dev_addr,
                           const uint8_t* mem_addr_h, const uint8_t* mem_addr_l,
                           uint8_t* buf, int len,
                           int timeout, al_i2c_cb_t cb, void* param);

#ifdef __cplusplus
}
#endif

#endif /* AL_I2C_H */
