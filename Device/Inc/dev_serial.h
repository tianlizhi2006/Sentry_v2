#ifndef _DEV_SERIAL
#define _DEV_SERIAL

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "drivers_buffer.h"
//#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
	
	
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

typedef void (*USART_CallbackFunction_t)(bool mode);

class Serialctrl: public Buffer
{
public:
	Serialctrl(UART_HandleTypeDef *_huartx, DMA_HandleTypeDef * hdma_usart_rx , uint32_t BufferSize , uint8_t Serial_Mode);
	void attachInterrupt(USART_CallbackFunction_t Function);
//	void IRQHandler(void);
void IRQHandler_RXNE(uint8_t c);
void IRQHandler_IDLE(void);

uint8_t Serial_Mode;

uint8_t receive_RXNE;
uint8_t receive_IDLE;

	void sendData(uint8_t ch);
	void sendData(const void *str);
	void sendData(const void *buf, uint8_t len);

	int available(void);
	uint8_t read(void);
	int peek(void);

	DMA_HandleTypeDef * hdma_usart_rx;
private:
	void flush(void);
//	USART_TypeDef * USARTx;
	UART_HandleTypeDef * huartx;
	USART_CallbackFunction_t USART_Function;
  RingBuffer _rx_buffer;
};

extern Serialctrl Serial3_Ctrl;
extern Serialctrl Serial5_Ctrl;
extern Serialctrl Serial10_Ctrl;

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart10;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;

#endif /* _DEV_SERIAL */
