//
//  al_pwm.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/3.
//

#ifndef AL_PWM_H
#define AL_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
int al_pwm_read(int fd, int* pulse);

int al_pwm_write(int fd, int pulse);

#ifdef __cplusplus
}
#endif

#endif /* AL_PWM_H */
