//
//  al_uart.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/5/28.
//

// Includes
#include "al.h"

#if (BSP_NR_UARTs > 0)

// Private variables
static SemaphoreHandle_t al_uart_rx_bus_semphrs[BSP_NR_UARTs];
static SemaphoreHandle_t al_uart_tx_bus_semphrs[BSP_NR_UARTs];
static al_uart_cb_t al_uart_rx_callbacks[BSP_NR_UARTs];
static al_uart_cb_t al_uart_tx_callbacks[BSP_NR_UARTs];
static uint8_t al_uart_rx_bufs[BSP_NR_UARTs];
static void* al_uart_tx_params[BSP_NR_UARTs];

// Functions
void al_uart_init(void) {
  for (int i = 0; i < BSP_NR_UARTs; ++i) {
    al_uart_rx_bus_semphrs[i] = xSemaphoreCreateBinary();
    al_uart_tx_bus_semphrs[i] = xSemaphoreCreateBinary();
    xSemaphoreGive(al_uart_rx_bus_semphrs[i]);
    xSemaphoreGive(al_uart_tx_bus_semphrs[i]);
  }
}

int al_uart_start_receiving(int fd, al_uart_cb_t cb) {
  UART_HandleTypeDef* huart = NULL;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_UARTs || !cb) {
    return AL_ERROR_SANITY;
  }

  // get handler
  BSP_UART_FD2HANDLE(fd, huart);
  if (!huart) {
    return AL_ERROR_BSP;
  }

  // lock
  if (xSemaphoreTake(al_uart_rx_bus_semphrs[fd], 0) != pdTRUE) {
    return AL_ERROR_BUSY;
  }
  al_uart_rx_callbacks[fd] = cb;

  // start receiving
  if (HAL_UART_Receive_IT(huart, &al_uart_rx_bufs[fd], 1U) != HAL_OK) {
    xSemaphoreGive(al_uart_rx_bus_semphrs[fd]);
    return AL_ERROR_HAL;
  }

  return 0;
}

int al_uart_async_send(int fd, const uint8_t* buf, int len, int timeout,
                       al_uart_cb_t cb, void* param) {
  UART_HandleTypeDef* huart = NULL;
  TickType_t ticks2wait = 0;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_UARTs || !buf || len <= 0) {
    return AL_ERROR_SANITY;
  }

  // HAL limits
  if (len > UINT16_MAX) {
    return AL_ERROR_LIMITS;
  }

  // get handler
  BSP_UART_FD2HANDLE(fd, huart);
  if (!huart) {
    return AL_ERROR_BSP;
  }

  // convert ms to ticks
  if (timeout < 0) {
    ticks2wait = portMAX_DELAY;
  } else {
    ticks2wait = timeout / portTICK_PERIOD_MS;
  }

  // lock
  if (xSemaphoreTake(al_uart_tx_bus_semphrs[fd], ticks2wait) != pdTRUE) {
    return AL_ERROR_BUSY;
  }
  al_uart_tx_callbacks[fd] = cb;
  al_uart_tx_params[fd] = param;

  // start sending
  if (HAL_UART_Transmit_DMA(huart, buf, (uint16_t) len) != HAL_OK) {
    xSemaphoreGive(al_uart_tx_bus_semphrs[fd]);
    return AL_ERROR_HAL;
  }

  return 0;
}

// ISRs
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  int fd = -1;
  int ec = 0;

  BSP_UART_HANDLE2FD(huart, fd);

  if (fd >= 0 && fd < BSP_NR_UARTs) {
    if (al_uart_rx_callbacks[fd]) {
      ec = (huart->ErrorCode == HAL_UART_ERROR_NONE) ? 0 : -1;
      al_uart_rx_callbacks[fd](fd, ec, (void*) ((int) al_uart_rx_bufs[fd]));
    }
    // continue receiving
    // ATTENTION:
    // Calling `HAL_UART_Receive_IT()` will clear `ErrorCode`,
    // causing UART_IRQHandler to ignore some "blocking errors",
    // it is okay because those errors have not been enabled.
    HAL_UART_Receive_IT(huart, &al_uart_rx_bufs[fd], 1U);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  int fd = -1;
  BaseType_t xShouldYield = pdFALSE;

  BSP_UART_HANDLE2FD(huart, fd);

  if (fd >= 0 && fd < BSP_NR_UARTs) {
    if (al_uart_tx_callbacks[fd]) {
      al_uart_tx_callbacks[fd](fd, 0, al_uart_tx_params[fd]);
    }
    xSemaphoreGiveFromISR(al_uart_tx_bus_semphrs[fd], &xShouldYield);
    portYIELD_FROM_ISR(xShouldYield);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  int rx_err = 0;
  int tx_err = 0;
  int fd = -1;
  BaseType_t xShouldYield = pdFALSE;

  // do some low level handling
  BSP_UART_HANDLE_ERROR(huart, rx_err, tx_err);
  if (!tx_err) {
    return;
  }

  BSP_UART_HANDLE2FD(huart, fd);

  if (fd >= 0 && fd < BSP_NR_UARTs) {
    if (al_uart_tx_callbacks[fd]) {
      al_uart_tx_callbacks[fd](fd, -1, al_uart_tx_params[fd]);
    }
    xSemaphoreGiveFromISR(al_uart_tx_bus_semphrs[fd], &xShouldYield);
    portYIELD_FROM_ISR(xShouldYield);
  }
}

#endif /* BSP_NR_UARTs > 0 */
