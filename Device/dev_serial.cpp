#include "dev_serial.h"
#include "app_preference.h"

Serialctrl Serial3_Ctrl(&huart3, &hdma_usart3_rx, Serial3_Buffer_Size, Serial3_Mode);
Serialctrl Serial5_Ctrl(&huart5, &hdma_uart5_rx, Serial5_Buffer_Size, Serial5_Mode);
Serialctrl Serial10_Ctrl(&huart10, &hdma_usart10_rx, Serial10_Buffer_Size, Serial10_Mode);

Serialctrl::Serialctrl(UART_HandleTypeDef *_huartx, DMA_HandleTypeDef *hdma_usart_rx, uint32_t BufferSize, uint8_t Serial_Mode)
{
	this->huartx = _huartx;
	this->hdma_usart_rx = hdma_usart_rx;
	this->Serial_Mode = Serial_Mode;
	USART_Function = 0;
	if (Serial_Mode == Serial_NORMAL_Mode)
	{
		newBuffer(&_rx_buffer, BufferSize);
	}
}

void Serialctrl::attachInterrupt(USART_CallbackFunction_t Function)
{
	USART_Function = Function;
}

void Serialctrl::IRQHandler_RXNE(uint8_t c)
{
	Buffer_Write(&_rx_buffer, c);
	if (USART_Function)
	{
		USART_Function(0);
	}
}

void Serialctrl::IRQHandler_IDLE(void)
{
	if (USART_Function)
	{
		USART_Function(1);
	}
}

void Serialctrl::sendData(uint8_t ch)
{
	while (__HAL_UART_GET_FLAG(huartx, UART_FLAG_TC) != SET)
	{
		osDelay(1);
	}
	HAL_UART_Transmit_DMA(huartx, &ch, 1);
}

void Serialctrl::sendData(const void *str)
{
	unsigned int index = 0;
	do
	{
		sendData(*((uint8_t *)str + index));
		++index;
	} while (*((uint8_t *)str + index) != '\0');
}

void Serialctrl::sendData(const void *buf, uint8_t len)
{
	HAL_UART_Transmit_DMA(huartx, (uint8_t *)buf, len);
}

int Serialctrl::available(void)
{
	return ((unsigned int)(_rx_buffer.buf_size + _rx_buffer.pw - _rx_buffer.pr)) % _rx_buffer.buf_size;
}

uint8_t Serialctrl::read(void)
{
	uint8_t value = 0;
	Buffer_Read(&_rx_buffer, &value);
	return value;
}

int Serialctrl::peek(void)
{
	if (_rx_buffer.pr == _rx_buffer.pw)
	{
		return -1;
	}
	return _rx_buffer.fifo[_rx_buffer.pr];
}

void Serialctrl::flush(void)
{
	_rx_buffer.pr = _rx_buffer.pw;
}

extern "C" {

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		Serial3_Ctrl.IRQHandler_RXNE(Serial3_Ctrl.receive_RXNE);
		HAL_UART_Receive_IT(&huart3, &Serial3_Ctrl.receive_RXNE, 1);
	}
	else if (huart->Instance == UART5)
	{
		Serial5_Ctrl.IRQHandler_RXNE(Serial5_Ctrl.receive_RXNE);
		HAL_UART_Receive_IT(&huart5, &Serial5_Ctrl.receive_RXNE, 1);
	}
	else if (huart->Instance == USART10)
	{
		Serial10_Ctrl.IRQHandler_RXNE(Serial10_Ctrl.receive_RXNE);
		HAL_UART_Receive_IT(&huart10, &Serial10_Ctrl.receive_RXNE, 1);
	}
}

void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		Serial3_Ctrl.IRQHandler_IDLE();
	}
	else if (huart->Instance == UART5)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		Serial5_Ctrl.IRQHandler_IDLE();
	}
	else if (huart->Instance == USART10)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart10);
		Serial10_Ctrl.IRQHandler_IDLE();
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (HAL_UART_GetError(huart) & HAL_UART_ERROR_PE)
	{
		__HAL_UART_CLEAR_PEFLAG(huart);
	}
	else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_NE)
	{
		__HAL_UART_CLEAR_NEFLAG(huart);
	}
	else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_FE)
	{
		__HAL_UART_CLEAR_FEFLAG(huart);
	}
	else if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
	{
		__HAL_UART_CLEAR_OREFLAG(huart);
	}

	if (huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3, &Serial3_Ctrl.receive_RXNE, 1);
	}
	else if (huart->Instance == UART5)
	{
		HAL_UART_Receive_IT(&huart5, &Serial5_Ctrl.receive_RXNE, 1);
	}
	else if (huart->Instance == USART10)
	{
		HAL_UART_Receive_IT(&huart10, &Serial10_Ctrl.receive_RXNE, 1);
	}
}

}
