//
//  al_dshot.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/25.
//

// Includes
#include "al.h"

#if (BSP_NR_DSHOTs > 0)

// ATTENTION:
// 1. Only supports all DShot channels on a single timer instance.
// 2. Modify the locking mechanism to prevent ISR starvation.

// Definitions
#define AL_DSHOT_BITS (30 * BSP_NR_DSHOTs)

// Private variables
static SemaphoreHandle_t al_dshot_mutex;
static uint16_t al_dshot_values[BSP_NR_DSHOTs];
static uint32_t al_dshot_bit_values[2][AL_DSHOT_BITS];
static uint32_t* al_dshot_bv_task_ptr = al_dshot_bit_values[0];
static uint32_t* al_dshot_bv_dma_ptr = al_dshot_bit_values[1];
static int al_dshot_started;
static int al_dshot_updated;

// Private functions
inline uint16_t al_dshot_make_pattern(uint16_t value) {
  uint16_t crc = 0U;
  uint16_t crc_data = 0U;

  crc_data = value;
  for (int i = 0; i < 3; ++i) {
    crc ^= crc_data;
    crc_data >>= 4U;
  }

  return (uint16_t) ((value << 4U) | (crc & 0xfU));
}

// Functions
void al_dshot_init(void) {
  al_dshot_mutex = xSemaphoreCreateMutex();
  // calculate bit value
  for (int i = 0; i < BSP_NR_DSHOTs; ++i) {
    uint16_t pattern = al_dshot_make_pattern(al_dshot_values[i] = 0U);
    for (int j = 0; j < 16; ++j) {
      al_dshot_bv_task_ptr[j * BSP_NR_DSHOTs + i] =
        ((pattern & 0x8000U) != 0U) ? BSP_DSHOT_BIT_1 : BSP_DSHOT_BIT_0;
      pattern <<= 1U;
    }
  }
}

int al_dshot_start(void) {
  TIM_HandleTypeDef* htim = NULL;
  uint32_t burst_base = 0U;
  uint32_t burst_request = 0U;
  uint32_t burst_length = 0U;
  HAL_StatusTypeDef status = HAL_OK;

  // get handler
  BSP_DSHOT_GET_BURST_PARAMS(htim, burst_base, burst_request, burst_length);
  if (!htim) {
    return AL_ERROR_BSP;
  }

  // enter critical area
  if (xSemaphoreTake(al_dshot_mutex, portMAX_DELAY) != pdTRUE) {
    return AL_ERROR_BUSY;
  }
  if (!al_dshot_started) {
    // start
    status = HAL_TIM_DMABurst_MultiWriteStart(htim, burst_base, burst_request,
                                              al_dshot_bv_task_ptr,
                                              burst_length,
                                              (uint32_t) AL_DSHOT_BITS);
    if (status == HAL_OK) {
      uint32_t* bv_ptr = al_dshot_bv_task_ptr;
      al_dshot_bv_task_ptr = al_dshot_bv_dma_ptr;
      al_dshot_bv_dma_ptr = bv_ptr;
      al_dshot_started = 1;
    }
  }
  xSemaphoreGive(al_dshot_mutex);
  // exit critical area

  return (status != HAL_OK) ? AL_ERROR_HAL : 0;
}

int al_dshot_read(int offset, int nchannels, uint16_t* values) {
  // sanity check
  if (offset < 0 || offset >= BSP_NR_DSHOTs ||
      nchannels <= 0 || offset + nchannels > BSP_NR_DSHOTs ||
      !values) {
    return AL_ERROR_SANITY;
  }

  // enter critical area
  if (xSemaphoreTake(al_dshot_mutex, portMAX_DELAY) != pdTRUE) {
    return AL_ERROR_BUSY;
  }
  for (int i = 0; i < nchannels; ++i) {
    values[i] = al_dshot_values[offset + i];
  }
  xSemaphoreGive(al_dshot_mutex);
  // exit critical area

  return 0;
}

int al_dshot_write(int offset, int nchannels, const uint16_t* values) {
  // sanity check
  if (offset < 0 || offset >= BSP_NR_DSHOTs ||
      nchannels <= 0 || offset + nchannels > BSP_NR_DSHOTs ||
      !values) {
    return AL_ERROR_SANITY;
  }

  // enter critical area
  if (xSemaphoreTake(al_dshot_mutex, portMAX_DELAY) != pdTRUE) {
    return AL_ERROR_BUSY;
  }
  for (int i = 0; i < nchannels; ++i) {
    uint16_t pattern = 0U;
    if (al_dshot_values[offset + i] == values[i]) {
      continue;
    }
    pattern = al_dshot_make_pattern(al_dshot_values[offset + i] = values[i]);
    for (int j = 0; j < 16; ++j) {
      al_dshot_bv_task_ptr[j * BSP_NR_DSHOTs + offset + i] =
        ((pattern & 0x8000U) != 0U) ? BSP_DSHOT_BIT_1 : BSP_DSHOT_BIT_0;
      pattern <<= 1U;
    }
    al_dshot_updated = 1;
  }
  xSemaphoreGive(al_dshot_mutex);
  // exit critical area

  return 0;
}

// ISR callbacks
void al_dshot_isr_cb(TIM_HandleTypeDef* htim) {
  BaseType_t should_yield = pdFALSE;
  uint32_t burst_base = 0U;
  uint32_t burst_request = 0U;
  uint32_t burst_length = 0U;
  HAL_StatusTypeDef status = HAL_OK;

  BSP_DSHOT_GET_BURST_PARAMS(htim, burst_base, burst_request, burst_length);
  if (htim) {
    if (xSemaphoreTakeFromISR(al_dshot_mutex, &should_yield) == pdTRUE) {
      if (al_dshot_updated) {
        uint32_t* bv_ptr = al_dshot_bv_task_ptr;
        al_dshot_bv_task_ptr = al_dshot_bv_dma_ptr;
        al_dshot_bv_dma_ptr = bv_ptr;
        al_dshot_updated = 0;
      }
      xSemaphoreGiveFromISR(al_dshot_mutex, &should_yield);
    }
    status = HAL_TIM_DMABurst_WriteStop(htim, burst_request);
    assert_param(status == HAL_OK);
    status = HAL_TIM_DMABurst_MultiWriteStart(htim, burst_base, burst_request,
                                              al_dshot_bv_dma_ptr,
                                              burst_length,
                                              (uint32_t) AL_DSHOT_BITS);
    assert_param(status == HAL_OK);
    portYIELD_FROM_ISR(should_yield);
  }
}

#endif /* BSP_NR_DSHOTs */
