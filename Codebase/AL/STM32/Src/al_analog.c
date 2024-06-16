//
//  al_analog.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/10.
//

// Includes
#include "al.h"

#if (BSP_NR_ADCs > 0)

// ATTENTION:
// To optimize performance, do not enable ADC and DMA interrupts.
// ATTENTION:
// The raw data should be 32-bit aligned.

// Functions
void al_analog_init(void) { }

int al_analog_read(int fd, float* value) {
  return al_analog_read_raw(fd, value, NULL);
}

int al_analog_read_raw(int fd, float* value, int* data) {
  volatile uint32_t* p_data = NULL;
  uint32_t raw_data = 0U;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_ADCs) {
    return AL_ERROR_SANITY;
  }

  // get data pointer
  BSP_ADC_FD2PDATA(fd, p_data);
  if (!p_data) {
    return AL_ERROR_BSP;
  }

  raw_data = *p_data;

  if (value) {
    *value = BSP_ADC_CALCULATE_VALUE(fd, raw_data);
  }

  if (data) {
    *data = (int) raw_data;
  }

  return 0;
}

#endif /* BSP_NR_ADCs */
