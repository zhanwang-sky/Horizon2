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
// To optimize performance, do not enable DMA interrupts.
// ATTENTION:
// The raw data should be 32-bit aligned.

// Functions
void al_analog_init(void) { }

int al_analog_read(int fd, float* value) {
  volatile uint32_t* p_data = NULL;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_ADCs) {
    return AL_ERROR_SANITY;
  }

  // get data
  BSP_ADC_FD2PDATA(fd, p_data);
  if (!p_data) {
    return AL_ERROR_BSP;
  }

  // calculate value
  if (value) {
    uint32_t data = *p_data;
    *value = BSP_ADC_CALCULATE_VALUE(fd, data);
  }

  return 0;
}

int al_analog_read_raw(int fd, int* data) {
  volatile uint32_t* p_data = NULL;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_ADCs) {
    return AL_ERROR_SANITY;
  }

  // get data
  BSP_ADC_FD2PDATA(fd, p_data);
  if (!p_data) {
    return AL_ERROR_BSP;
  }

  if (data) {
    *data = (int) (*p_data);
  }

  return 0;
}

#endif /* BSP_NR_ADCs */
