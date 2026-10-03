#ifndef __DRIVERS_REMOTE_H
#define __DRIVERS_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32h7xx_hal.h"

void MA_UART_Receive_DMA_Init(UART_HandleTypeDef *_huartx, DMA_HandleTypeDef * hdma_usart_rx ,uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_REMOTE_H */
