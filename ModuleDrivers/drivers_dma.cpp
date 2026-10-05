#include "drivers_dma.h"

void MA_UART_Receive_DMA_Init(UART_HandleTypeDef *_huartx, DMA_HandleTypeDef * hdma_usart_rx ,uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 允许 UART 接收事件触发 DMA 请求。
    SET_BIT(_huartx->Instance->CR3, USART_CR3_DMAR);

    // 用串口空闲中断判断一帧接收结束。
    __HAL_UART_ENABLE_IT(_huartx, UART_IT_IDLE);

    // 停止 DMA 后再配置地址和传输长度。
    __HAL_DMA_DISABLE(hdma_usart_rx);
    while(((DMA_Stream_TypeDef*) hdma_usart_rx->Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hdma_usart_rx);
    }

    ((DMA_Stream_TypeDef*) hdma_usart_rx->Instance)->PAR = (uint32_t) & (_huartx->Instance->RDR);
    // 双缓冲区的第一个接收地址。
    ((DMA_Stream_TypeDef*) hdma_usart_rx->Instance)->M0AR = (uint32_t)(rx1_buf);
	// 双缓冲区的第二个接收地址。
    ((DMA_Stream_TypeDef*) hdma_usart_rx->Instance)->M1AR = (uint32_t)(rx2_buf);
    // 每个缓冲区可接收的字节数。
    ((DMA_Stream_TypeDef*) hdma_usart_rx->Instance)->NDTR = dma_buf_num;
    // 启用 DMA 双缓冲模式。
    SET_BIT(((DMA_Stream_TypeDef*) hdma_usart_rx->Instance)->CR, DMA_SxCR_DBM);

    // 启动 DMA 接收。
    __HAL_DMA_ENABLE(hdma_usart_rx);

}
