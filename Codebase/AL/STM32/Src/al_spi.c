//
//  al_spi.c
//  Horizon
//
//  Created by zhanwang-sky on 2024/6/6.
//

// Includes
#include "al.h"

#if (BSP_NR_SPIs > 0)

// ATTENTION:
// The `data size` configured in BSP should be precisely 8 bits,
// setting it shorter or longer may lead to unpredictable errors.

// Private variables
static SemaphoreHandle_t al_spi_bus_semphrs[BSP_NR_SPIs];
static int al_spi_cs_maps[BSP_NR_SPIs];
static al_spi_cb_t al_spi_callbacks[BSP_NR_SPIs];
static void* al_spi_cb_params[BSP_NR_SPIs];

// Functions
void al_spi_init(void) {
  for (int i = 0; i < BSP_NR_SPIs; ++i) {
    al_spi_bus_semphrs[i] = xSemaphoreCreateBinary();
    xSemaphoreGive(al_spi_bus_semphrs[i]);
  }
}

int al_spi_async_read_write(int fd, int cs, uint8_t* rx_buf, uint8_t* tx_buf, int len,
                            int timeout, al_spi_cb_t cb, void* param) {
  SPI_HandleTypeDef* hspi = NULL;
  GPIO_TypeDef* nss_port = NULL;
  uint16_t nss_pin = 0U;
  TickType_t ticks2wait = 0;

  // sanity check
  if (fd < 0 || fd >= BSP_NR_SPIs || cs < 0 || !rx_buf || !tx_buf || len <= 0) {
    return AL_ERROR_SANITY;
  }
  // addr overflow check
  if ((UINTPTR_MAX - (uintptr_t) rx_buf < len) || (UINTPTR_MAX - (uintptr_t) tx_buf < len)) {
    return AL_ERROR_SANITY;
  }
  // addr overlap check
  if ((tx_buf < rx_buf) && (tx_buf + len > rx_buf)) {
    // tx data might be overwritten by the rx data
    return AL_ERROR_SANITY;
  }

  // HAL limits
  if (len > UINT16_MAX) {
    return AL_ERROR_LIMITS;
  }

  // get handler
  BSP_SPI_FD2HANDLE(fd, hspi);
  if (!hspi) {
    return AL_ERROR_BSP;
  }

  // get nss port & pin
  BSP_SPI_CS2PORTPIN(cs, nss_port, nss_pin);
  if (!nss_port) {
    return AL_ERROR_BSP;
  }

  // convert ms to ticks
  if (timeout < 0) {
    ticks2wait = portMAX_DELAY;
  } else {
    ticks2wait = timeout / portTICK_PERIOD_MS;
  }

  // lock
  if (xSemaphoreTake(al_spi_bus_semphrs[fd], ticks2wait) != pdTRUE) {
    return AL_ERROR_BUSY;
  }
  al_spi_cs_maps[fd] = cs;
  al_spi_callbacks[fd] = cb;
  al_spi_cb_params[fd] = param;

  // start transfer
  HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_RESET);
  // ATTENTION:
  // It is necessary to insert some delays here when using a high frequency CPU (close to 1GHz).
  // ATTENTION:
  // Before calling `HAL_SPI_TransmitReceive_DMA`, check whether it correctly
  // sets the DMA error callbacks for both RX and TX channels. If not, correct it!
  if (HAL_SPI_TransmitReceive_DMA(hspi, tx_buf, rx_buf, (uint16_t) len) != HAL_OK) {
    HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);
    xSemaphoreGive(al_spi_bus_semphrs[fd]);
    return AL_ERROR_HAL;
  }

  return 0;
}

// ISR callbacks
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  int fd = -1;
  GPIO_TypeDef* nss_port = NULL;
  uint16_t nss_pin = 0U;
  BaseType_t xShouldYield = pdFALSE;

  BSP_SPI_HANDLE2FD(hspi, fd);

  if (fd >= 0 && fd < BSP_NR_SPIs) {
    BSP_SPI_CS2PORTPIN(al_spi_cs_maps[fd], nss_port, nss_pin);
    if (nss_port != NULL) {
      HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);
    }
    if (al_spi_callbacks[fd]) {
      al_spi_callbacks[fd](fd, 0, al_spi_cb_params[fd]);
    }
    xSemaphoreGiveFromISR(al_spi_bus_semphrs[fd], &xShouldYield);
    portYIELD_FROM_ISR(xShouldYield);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
  int fd = -1;
  GPIO_TypeDef* nss_port = NULL;
  uint16_t nss_pin = 0U;
  BaseType_t xShouldYield = pdFALSE;

  // do some low level handling
  BSP_SPI_HANDLE_ERROR(hspi);

  BSP_SPI_HANDLE2FD(hspi, fd);

  if (fd >= 0 && fd < BSP_NR_SPIs) {
    BSP_SPI_CS2PORTPIN(al_spi_cs_maps[fd], nss_port, nss_pin);
    if (nss_port != NULL) {
      HAL_GPIO_WritePin(nss_port, nss_pin, GPIO_PIN_SET);
    }
    if (al_spi_callbacks[fd]) {
      al_spi_callbacks[fd](fd, -1, al_spi_cb_params[fd]);
    }
    xSemaphoreGiveFromISR(al_spi_bus_semphrs[fd], &xShouldYield);
    portYIELD_FROM_ISR(xShouldYield);
  }
}

#endif /* BSP_NR_SPIs */
