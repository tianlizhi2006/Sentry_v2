#ifndef __APP_SERIAL_H
#define __APP_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include <stdint.h>	
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "app_preference.h"
#include "dev_serial.h"

	
#ifdef __cplusplus
}
#endif

#define SERIAL3 USART3
#define SERIAL5 UART5
#define SERIAL10 USART10

struct Serial_Data_t
{
    uint8_t Header;
    uint8_t Tail;
    uint8_t Lenth0;
    uint8_t Lenth1;
    uint8_t Lenth2;
    uint8_t Lenth3;
		uint8_t buffer_size;
    uint8_t Len;
    uint8_t Temp;
		uint8_t Mode;
    uint8_t **Data;
    Serial_Data_t(uint8_t Header_, uint8_t Tail_, uint8_t Lenth0_,uint8_t Lenth1_,uint8_t Lenth2_,uint8_t Lenth3_, uint8_t buffer_size_, uint8_t Mode_)
        :Header(Header_), Tail(Tail_), Lenth0(Lenth0_),Lenth1(Lenth1_),Lenth2(Lenth2_),Lenth3(Lenth3_), buffer_size(buffer_size_), Mode(Mode_)
    {
        Data = new uint8_t*[2];
				Data[0] = new uint8_t[buffer_size_];
				Data[1] = new uint8_t[buffer_size_];
    };
};

class Serial_Ctrl
{
public:
    Serial_Ctrl()
        :Serial3(Serial3_Data_Header, Serial3_Data_Tail, Serial3_Data_Lenth0, Serial3_Data_Lenth1, Serial3_Data_Lenth2, Serial3_Data_Lenth3, Serial3_Buffer_Size, Serial3_Mode),
        Serial5(Serial5_Data_Header, Serial5_Data_Tail, Serial5_Data_Lenth0, Serial5_Data_Lenth1, Serial5_Data_Lenth2, Serial5_Data_Lenth3, Serial5_Buffer_Size, Serial5_Mode),
        Serial10(Serial10_Data_Header, Serial10_Data_Tail, Serial10_Data_Lenth0, Serial10_Data_Lenth1, Serial10_Data_Lenth2, Serial10_Data_Lenth3, Serial10_Buffer_Size, Serial10_Mode)
    {}

    void Hook(USART_TypeDef *SERIAL, bool mode);
    void Handle(Serialctrl *Serial, Serial_Data_t *Usart, bool mode);
    void Send_to_Message(Serialctrl *SerialCtrl , bool Memory);

    uint8_t Get_Data(Serial_Data_t *Serial, uint8_t *buf);

    ~Serial_Ctrl() {}

    Serial_Data_t Serial3;
    Serial_Data_t Serial5;
    Serial_Data_t Serial10;
			
private:
    void *buf;
};
extern Serial_Ctrl Serial_Cmd;

extern void Serial_ALL_Init();
#endif
