#include "app_serial.h"
#include "Message_Task.h"
#include "app_preference.h"
#include "drivers_dma.h"

Serial_Ctrl Serial_Cmd;

void Serial3_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL3, mode);
}

void Serial5_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL5, mode);
}

void Serial10_Hook(bool mode)
{
    Serial_Cmd.Hook(SERIAL10, mode);
}

void Serial_ALL_Init(void)
{
#if ( Serial3_Mode == Serial_NORMAL_Mode )
    HAL_UART_Receive_IT(&huart3, &Serial3_Ctrl.receive_RXNE, 1);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
#endif
#if ( Serial3_Mode == Serial_DMA_Mode )
    MA_UART_Receive_DMA_Init(&huart3, &hdma_usart3_rx, (uint8_t *)&(Serial_Cmd.Serial3.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial3.Data[1][1]), Serial3_Buffer_Size);
#endif

#if ( Serial5_Mode == Serial_NORMAL_Mode )
    HAL_UART_Receive_IT(&huart5, &Serial5_Ctrl.receive_RXNE, 1);   // 重新使能接收中断
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);  //idle interrupt
#endif
#if ( Serial5_Mode == Serial_DMA_Mode )
    MA_UART_Receive_DMA_Init(&huart5, &hdma_uart5_rx, (uint8_t *)&(Serial_Cmd.Serial5.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial5.Data[1][1]), Serial5_Buffer_Size);
#endif

#if ( Serial10_Mode == Serial_NORMAL_Mode )
    HAL_UART_Receive_IT(&huart10, &Serial10_Ctrl.receive_RXNE, 1);
		__HAL_UART_ENABLE_IT(&huart10, UART_IT_IDLE);
#endif
#if ( Serial10_Mode == Serial_DMA_Mode )
    MA_UART_Receive_DMA_Init(&huart10, &hdma_uart10_rx, (uint8_t *)&(Serial_Cmd.Serial10.Data[0][1]), (uint8_t *)&(Serial_Cmd.Serial10.Data[1][1]), Serial10_Buffer_Size);
#endif
    Serial3_Ctrl.attachInterrupt(Serial3_Hook);
    Serial5_Ctrl.attachInterrupt(Serial5_Hook);
    Serial10_Ctrl.attachInterrupt(Serial10_Hook);
}

void Serial_Ctrl::Hook(USART_TypeDef *SERIAL, bool mode)
{
    if (SERIAL == SERIAL3)
    {
        Handle(&Serial3_Ctrl, &Serial3, mode);
    }
    else if (SERIAL == SERIAL5)
    {
        Handle(&Serial5_Ctrl, &Serial5, mode);
    }
    else if (SERIAL == SERIAL10)
    {
        Handle(&Serial10_Ctrl, &Serial10, mode);
    }
}

void Serial_Ctrl::Handle(Serialctrl *SerialCtrl, Serial_Data_t *Serial, bool mode)
{
    if (Serial->Mode == Serial_NORMAL_Mode)
    {
        if (mode == 0)
        {
            Serial->Temp = SerialCtrl->peek();
					
            if (Serial->Header != NULL && Serial->Temp != Serial->Header)
            {
                SerialCtrl->read();
                return;
            }
			if (Serial->Len == Serial->buffer_size - 1)
            {
                for (uint8_t i = 0; i < Serial->Len; i++)
                {
                    SerialCtrl->read();
                }
            }
        }
        if (mode == 1)
        {
            Serial->Len = SerialCtrl->available();
            if ((Serial->Len == Serial->Lenth0 || Serial->Len == Serial->Lenth1 || Serial->Len == Serial->Lenth2 || Serial->Len == Serial->Lenth3) && (Serial->Len != NULL))
            {
                Serial->Data[0][0] = Serial->Len;
                for (uint8_t i = 0; i < Serial->Len; i++)
                {
                    Serial->Data[0][i + 1] = SerialCtrl->read();
                }
                if (Serial->Tail != NULL && Serial->Data[0][Serial->Len] != Serial->Tail)
                {
                    Serial->Data[0][0] = 0;
                }
                Serial->Len = SerialCtrl->available();
                if (Serial->Data[0][0] != 0)
                {
                    Send_to_Message(SerialCtrl, 0);
                }
            }
            else if (Serial->Lenth0 == NULL & Serial->Lenth1 == NULL & Serial->Lenth2 == NULL & Serial->Lenth3 == NULL)
            {
                Serial->Data[0][0] = Serial->Len;
                if (SerialCtrl->peek() == 0xA5)
                {
                    for (uint8_t i = 0; i < Serial->Len; i++)
                    {
                        Serial->Data[0][i + 1] = SerialCtrl->read();
                    }
                    if (Serial->Data[0][0] != 0)
                    {
                        Send_to_Message(SerialCtrl, 0);
                    }
                }
                //						Serial->Len = SerialCtrl->available();
            }
            else
            {
                for (uint8_t i = 0; i < Serial->Len; i++)
                {
                    SerialCtrl->read();
                }
            }
						if (Serial->Len == Serial->buffer_size - 1)
            {
                for (uint8_t i = 0; i < Serial->Len; i++)
                {
                    SerialCtrl->read();
                }
            }
        }
    }
    if (Serial->Mode == Serial_DMA_Mode)
    {
        if (mode == 1)
        {
            bool Memory;
            /* Current memory buffer used is Memory 0 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(SerialCtrl->hdma_usart_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            Serial->Len = Serial->buffer_size - ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->NDTR = Serial->buffer_size;

            if ((((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->CR & DMA_SxCR_CT) == RESET)
            {
                // set memory buffer 1
                // 设定缓冲区1
                ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->CR |= DMA_SxCR_CT;
                Memory = 0;
            }
            else
            {
                // set memory buffer 0
                // 设定缓冲区0
                ((DMA_Stream_TypeDef *)SerialCtrl->hdma_usart_rx->Instance)->CR &= ~(DMA_SxCR_CT);
                Memory = 1;
            }

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(SerialCtrl->hdma_usart_rx);


            if ((Serial->Len == Serial->Lenth0 || Serial->Len == Serial->Lenth1 || Serial->Len == Serial->Lenth2 || Serial->Len == Serial->Lenth3) && (Serial->Len != NULL))
            {
                Serial->Data[Memory][0] = Serial->Len;
                if (Serial->Header != NULL && Serial->Header != Serial->Data[Memory][1])
                {
                    Serial->Len = 0;
                    return;
                }
                if (Serial->Tail != NULL && Serial->Tail != Serial->Data[Memory][Serial->Len])
                {
                    Serial->Len = 0;
                    return;
                }
                if (Serial->Len != 0)
                {
                    Send_to_Message(SerialCtrl, Memory);
                }
            }
						if (Serial->Len == Serial->buffer_size - 1)
            {
                for (uint8_t i = 0; i < Serial->Len; i++)
                {
                    SerialCtrl->read();
                }
            }
        }
    }
}

uint8_t Serial_Ctrl::Get_Data(Serial_Data_t *Serial, uint8_t *buf)
{
    if (Serial->Len == 0)
    {
        return 0;
    }
    buf = Serial->Data[0];
    return Serial->Len;
}

void Serial_Ctrl::Send_to_Message(Serialctrl *SerialCtrl, bool Memory)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (SerialCtrl == &Serial3_Ctrl)
    {
        ID_Data[SerialData3].Data_Ptr = Serial3.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData3], &xHigherPriorityTaskWoken);
    }
    else if (SerialCtrl == &Serial5_Ctrl)
    {
        ID_Data[SerialData5].Data_Ptr = Serial5.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData5], &xHigherPriorityTaskWoken);
    }
    else if (SerialCtrl == &Serial10_Ctrl)
    {
        ID_Data[SerialData10].Data_Ptr = Serial10.Data[Memory];
        xQueueSendFromISR(Serial_Rx_Queue, &ID_Data[SerialData10], &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
