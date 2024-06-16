//
//  al_i2c.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/16.
//

// Includes
#include "al.h"

#if (BSP_NR_I2Cs > 0)

// Private typedefs
typedef HAL_StatusTypeDef (*hal_i2c_master_xfer_dma)(I2C_HandleTypeDef*, uint16_t, uint8_t*, uint16_t);
typedef HAL_StatusTypeDef (*hal_i2c_mem_xfer_dma)(I2C_HandleTypeDef*, uint16_t, uint16_t, uint16_t, uint8_t*, uint16_t);

// Private variables
static SemaphoreHandle_t al_i2c_bus_semphrs[BSP_NR_I2Cs];
static al_i2c_cb_t volatile al_i2c_callbacks[BSP_NR_I2Cs];
static void* volatile al_i2c_cb_params[BSP_NR_I2Cs];

// Private functions
int al_i2c_xfer(int fd, uint8_t dev_addr, uint8_t* buf, int len,
                int timeout, al_i2c_cb_t cb, void* param,
                hal_i2c_master_xfer_dma xfer) {
  I2C_HandleTypeDef* hi2c = NULL;
  TickType_t ticks2wait = 0;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_I2Cs || len < 0) {
    return AL_ERROR_SANITY;
  }

  // HAL limits
  if (len > UINT16_MAX) {
    return AL_ERROR_LIMITS;
  }

  // get handler
  BSP_I2C_FD2HANDLE(fd, hi2c);
  if (!hi2c) {
    return AL_ERROR_BSP;
  }

  // convert ms to ticks
  if (timeout < 0) {
    ticks2wait = portMAX_DELAY;
  } else {
    ticks2wait = timeout / portTICK_PERIOD_MS;
  }

  // lock
  if (xSemaphoreTake(al_i2c_bus_semphrs[fd], ticks2wait) != pdTRUE) {
    return AL_ERROR_BUSY;
  }
  al_i2c_callbacks[fd] = cb;
  al_i2c_cb_params[fd] = param;

  // start xfer
  if (xfer(hi2c, (uint16_t) (dev_addr << 1U), buf, (uint16_t) len) != HAL_OK) {
    xSemaphoreGive(al_i2c_bus_semphrs[fd]);
    return AL_ERROR_HAL;
  }

  return 0;
}

int al_i2c_mem_xfer(int fd, uint8_t dev_addr,
                    const uint8_t* mem_addr_h, const uint8_t* mem_addr_l,
                    uint8_t* buf, int len,
                    int timeout, al_i2c_cb_t cb, void* param,
                    hal_i2c_mem_xfer_dma xfer) {
  I2C_HandleTypeDef* hi2c = NULL;
  TickType_t ticks2wait = 0;
  uint16_t mem_addr = 0U;
  uint16_t mem_addr_size = I2C_MEMADD_SIZE_8BIT;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_I2Cs || !mem_addr_l || !buf || len <= 0) {
    return AL_ERROR_SANITY;
  }

  // HAL limits
  if (len > UINT16_MAX) {
    return AL_ERROR_LIMITS;
  }

  // get handler
  BSP_I2C_FD2HANDLE(fd, hi2c);
  if (!hi2c) {
    return AL_ERROR_BSP;
  }

  // convert ms to ticks
  if (timeout < 0) {
    ticks2wait = portMAX_DELAY;
  } else {
    ticks2wait = timeout / portTICK_PERIOD_MS;
  }

  // make mem_addr
  if (mem_addr_h) {
    mem_addr = *mem_addr_h;
    mem_addr <<= 8U;
    mem_addr_size = I2C_MEMADD_SIZE_16BIT;
  }
  mem_addr |= *mem_addr_l;

  // lock
  if (xSemaphoreTake(al_i2c_bus_semphrs[fd], ticks2wait) != pdTRUE) {
    return AL_ERROR_BUSY;
  }
  al_i2c_callbacks[fd] = cb;
  al_i2c_cb_params[fd] = param;

  // start xfer
  if (xfer(hi2c, (uint16_t) (dev_addr << 1U), mem_addr, mem_addr_size, buf, (uint16_t) len) != HAL_OK) {
    xSemaphoreGive(al_i2c_bus_semphrs[fd]);
    return AL_ERROR_HAL;
  }

  return 0;
}

void al_i2c_xfer_cplt(I2C_HandleTypeDef* hi2c) {
  int fd = -1;
  BaseType_t should_yield = pdFALSE;

  BSP_I2C_HANDLE2FD(hi2c, fd);
  if (fd >= 0 && fd < BSP_NR_I2Cs) {
    if (al_i2c_callbacks[fd]) {
      al_i2c_callbacks[fd](fd, 0, al_i2c_cb_params[fd]);
    }
    xSemaphoreGiveFromISR(al_i2c_bus_semphrs[fd], &should_yield);
    portYIELD_FROM_ISR(should_yield);
  }
}

// Functions
void al_i2c_init(void) {
  for (int i = 0; i < BSP_NR_I2Cs; ++i) {
    al_i2c_bus_semphrs[i] = xSemaphoreCreateBinary();
    xSemaphoreGive(al_i2c_bus_semphrs[i]);
  }
}

int al_i2c_async_read(int fd, uint8_t dev_addr, uint8_t* buf, int len,
                      int timeout, al_i2c_cb_t cb, void* param) {
  return al_i2c_xfer(fd, dev_addr, buf, len, timeout, cb, param,
                     HAL_I2C_Master_Receive_DMA);
}

int al_i2c_async_write(int fd, uint8_t dev_addr, uint8_t* buf, int len,
                       int timeout, al_i2c_cb_t cb, void* param) {
  return al_i2c_xfer(fd, dev_addr, buf, len, timeout, cb, param,
                     HAL_I2C_Master_Transmit_DMA);
}

int al_i2c_async_mem_read(int fd, uint8_t dev_addr,
                          const uint8_t* mem_addr_h, const uint8_t* mem_addr_l,
                          uint8_t* buf, int len,
                          int timeout, al_i2c_cb_t cb, void* param) {
  return al_i2c_mem_xfer(fd, dev_addr, mem_addr_h, mem_addr_l, buf, len,
                         timeout, cb, param,
                         HAL_I2C_Mem_Read_DMA);
}

int al_i2c_async_mem_write(int fd, uint8_t dev_addr,
                           const uint8_t* mem_addr_h, const uint8_t* mem_addr_l,
                           uint8_t* buf, int len,
                           int timeout, al_i2c_cb_t cb, void* param) {
  return al_i2c_mem_xfer(fd, dev_addr, mem_addr_h, mem_addr_l, buf, len,
                         timeout, cb, param,
                         HAL_I2C_Mem_Write_DMA);
}

// ISR callbacks
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
  al_i2c_xfer_cplt(hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  al_i2c_xfer_cplt(hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c) {
  al_i2c_xfer_cplt(hi2c);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  al_i2c_xfer_cplt(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
  int fd = -1;
  BaseType_t should_yield = pdFALSE;

  // low level handling
  BSP_I2C_HANDLE_ERROR(hi2c);

  // All I2C errors reaching here are blocking errors,
  // invoke the callback and release the bus.
  BSP_I2C_HANDLE2FD(hi2c, fd);
  if (fd >= 0 && fd < BSP_NR_I2Cs) {
    if (al_i2c_callbacks[fd]) {
      al_i2c_callbacks[fd](fd, -1, al_i2c_cb_params[fd]);
    }
    xSemaphoreGiveFromISR(al_i2c_bus_semphrs[fd], &should_yield);
    portYIELD_FROM_ISR(should_yield);
  }
}

#endif /* BSP_NR_I2Cs */
