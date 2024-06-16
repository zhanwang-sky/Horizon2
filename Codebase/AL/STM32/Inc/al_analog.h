//
//  al_analog.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/10.
//

#ifndef AL_ANALOG_H
#define AL_ANALOG_H

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void al_analog_init(void);

int al_analog_read(int fd, float* value);
int al_analog_read_raw(int fd, float* value, int* data);

#ifdef __cplusplus
}
#endif

#endif /* AL_ANALOG_H */
