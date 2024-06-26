//
//  al_dshot.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/25.
//

#ifndef AL_DSHOT_H
#define AL_DSHOT_H

// Includes
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void al_dshot_init(void);

int al_dshot_start(void);
int al_dshot_read(int offset, int nchannels, uint16_t* values);
int al_dshot_write(int offset, int nchannels, const uint16_t* values);

#ifdef __cplusplus
}
#endif

#endif /* AL_DSHOT_H */
