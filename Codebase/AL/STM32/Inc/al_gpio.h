//
//  al_gpio.h
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/8.
//

#ifndef AL_GPIO_H
#define AL_GPIO_H

// Includes
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void al_gpio_init(void);

int al_gpio_get(int fd, bool* state);
int al_gpio_set(int fd, bool state);
int al_gpio_toggle(int fd);

#ifdef __cplusplus
}
#endif

#endif /* AL_GPIO_H */
