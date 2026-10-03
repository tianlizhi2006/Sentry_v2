#include "dev_can.h"
#include "app_motor.h"
#include "Message_Task.h"

CANctrl CAN1_Ctrl(&hfdcan1,FDCAN1_Buffer_Size);
CANctrl CAN2_Ctrl(&hfdcan2,FDCAN2_Buffer_Size);
CANctrl CAN3_Ctrl(&hfdcan3,FDCAN3_Buffer_Size );

CANctrl::CANctrl( FDCAN_HandleTypeDef *CANx, uint32_t BufferSize )
{
    this->CANx = CANx;
	  this->CAN_Function = 0;
		newBuffer(&_rx_buffer, BufferSize);
}

void CANctrl::attachInterrupt(CAN_CallbackFunction_t Function)
{
    this->CAN_Function = Function;
}

void CANctrl::ChangeID(uint16_t StdID)
{
    this->StdId = StdID;
}

void CANctrl::SendData(const void *buf, uint8_t len)
{	
		FDCAN_TxHeader.Identifier=StdId;
		FDCAN_TxHeader.IdType=FDCAN_STANDARD_ID;
		FDCAN_TxHeader.TxFrameType=FDCAN_DATA_FRAME;
		FDCAN_TxHeader.DataLength=FDCAN_DLC_BYTES_8;
		FDCAN_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
		FDCAN_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;
		FDCAN_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;
		FDCAN_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
		FDCAN_TxHeader.MessageMarker=0;

		HAL_FDCAN_AddMessageToTxFifoQ(CANx,&FDCAN_TxHeader,(uint8_t *)buf);
}

int CANctrl::available(void)
{
    return ((unsigned int)(_rx_buffer.buf_size + _rx_buffer.pw - _rx_buffer.pr)) % _rx_buffer.buf_size;
}

uint8_t CANctrl::read(void)
{
    uint8_t c = 0;
    Buffer_Read(&_rx_buffer, &c);
    return c;
}

void CANctrl::IRQHandler(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs&FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=RESET)
		{
			if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&FDCAN_RxHeader,FDCAN_RxData.Data) == HAL_OK)
			{
				FDCAN_RxData.StdId.u32 = FDCAN_RxHeader.Identifier;
				Buffer_Write(&_rx_buffer, 0xA5);
        Buffer_Write(&_rx_buffer, 0xA6);
				for(uint8_t x=0;x<4;x++)
				{
					Buffer_Write(&_rx_buffer, FDCAN_RxData.StdId.u8[x]);
				}
				for(uint8_t x=0;x<8;x++)
				{
					Buffer_Write(&_rx_buffer, FDCAN_RxData.Data[x]);
				}
				if(CAN_Function)
        {
            CAN_Function(&FDCAN_RxData);
        }
			}
		}
}

extern"C"
{
   void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if(hfdcan->Instance==FDCAN1)
	{
		CAN1_Ctrl.IRQHandler(hfdcan , RxFifo0ITs);
	}
	else if(hfdcan->Instance==FDCAN2)
	{
		CAN2_Ctrl.IRQHandler(hfdcan , RxFifo0ITs);
	}
	else if(hfdcan->Instance==FDCAN3)
	{
		CAN3_Ctrl.IRQHandler(hfdcan , RxFifo0ITs);
	}
}
}
