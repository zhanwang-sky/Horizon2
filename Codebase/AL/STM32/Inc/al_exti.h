//
//  al_exti.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/8.
//

#ifndef AL_EXTI_H
#define AL_EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

// Typedefs
typedef void (*al_exti_cb_t)(int);

// Function prototypes
void al_exti_init(void);

int al_exti_start(int fd, al_exti_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* AL_EXTI_H */
