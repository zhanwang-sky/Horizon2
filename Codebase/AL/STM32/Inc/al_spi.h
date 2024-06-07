//
//  al_spi.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/6.
//

#ifndef AL_SPI_H
#define AL_SPI_H

// Includes
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Typedefs
typedef void (*al_spi_cb_t)(int, int, void*);

// Function prototypes
void al_spi_init(void);

int al_spi_async_read_write(int fd, int cs, uint8_t* rx_buf, uint8_t* tx_buf, int len,
                            int timeout, al_spi_cb_t cb, void* param);

#ifdef __cplusplus
}
#endif

#endif /* AL_SPI_H */
