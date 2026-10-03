/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef BSP_DWT_H
#define BSP_DWT_H

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "stdint.h"
//#include "stm32h7xx_hal.h"	
	
#define UINT32_MAX1           4294967295u
	
void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);

	
extern uint64_t CYCCNT64;
	
#ifdef __cplusplus
}
#endif


typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;
extern DWT_Time_t SysTime;



#endif /* BSP_DWT_H_ */
