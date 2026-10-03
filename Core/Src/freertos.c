/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "app_preference.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 每条 CAN 总线独立串行化发送，避免多个任务交叉修改发送 ID 和帧头。
osMutexId_t CAN1_TxMutexHandle;
osMutexId_t CAN2_TxMutexHandle;
osMutexId_t CAN3_TxMutexHandle;

/* USER CODE END Variables */
/* Definitions for Message_task */
osThreadId_t Message_taskHandle;
const osThreadAttr_t Message_task_attributes = {
  .name = "Message_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for CAN1_Rx_task */
osThreadId_t CAN1_Rx_taskHandle;
const osThreadAttr_t CAN1_Rx_task_attributes = {
  .name = "CAN1_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for CAN2_Rx_task */
osThreadId_t CAN2_Rx_taskHandle;
const osThreadAttr_t CAN2_Rx_task_attributes = {
  .name = "CAN2_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for Serial_Rx_task */
osThreadId_t Serial_Rx_taskHandle;
const osThreadAttr_t Serial_Rx_task_attributes = {
  .name = "Serial_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for DR16_Rx_task */
osThreadId_t DR16_Rx_taskHandle;
const osThreadAttr_t DR16_Rx_task_attributes = {
  .name = "DR16_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for CAN3_Rx_task */
osThreadId_t CAN3_Rx_taskHandle;
const osThreadAttr_t CAN3_Rx_task_attributes = {
  .name = "CAN3_Rx_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for Chassis_task */
osThreadId_t Chassis_taskHandle;
const osThreadAttr_t Chassis_task_attributes = {
  .name = "Chassis_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for Gimbal_task */
osThreadId_t Gimbal_taskHandle;
const osThreadAttr_t Gimbal_task_attributes = {
  .name = "Gimbal_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//消息队列声明
QueueHandle_t Message_Queue;
QueueHandle_t CAN1_Rx_Queue;
QueueHandle_t CAN2_Rx_Queue;
QueueHandle_t CAN3_Rx_Queue;
QueueHandle_t Serial_Rx_Queue;
QueueHandle_t DR16_Rx_Queue;
/* USER CODE END FunctionPrototypes */

extern void Message_Task(void *argument);
extern void CAN1_Rx_Task(void *argument);
extern void CAN2_Rx_Task(void *argument);
extern void Serial_Rx_Task(void *argument);
extern void DR16_Rx_Task(void *argument);
extern void CAN3_Rx_Task(void *argument);
extern void Chassis_Task(void *argument);
extern void Gimbal_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  CAN1_TxMutexHandle = osMutexNew(NULL);
  CAN2_TxMutexHandle = osMutexNew(NULL);
  CAN3_TxMutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	Message_Queue = xQueueCreate(8, sizeof(ID_Data_t));
	CAN1_Rx_Queue = xQueueCreate(8, sizeof(ID_Data_t));
	CAN2_Rx_Queue = xQueueCreate(8, sizeof(ID_Data_t));
	CAN3_Rx_Queue = xQueueCreate(8, sizeof(ID_Data_t));
	Serial_Rx_Queue = xQueueCreate(4, sizeof(ID_Data_t));
	DR16_Rx_Queue = xQueueCreate(2, sizeof(void *));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Message_task */
  Message_taskHandle = osThreadNew(Message_Task, NULL, &Message_task_attributes);

  /* creation of CAN1_Rx_task */
  CAN1_Rx_taskHandle = osThreadNew(CAN1_Rx_Task, NULL, &CAN1_Rx_task_attributes);

  /* creation of CAN2_Rx_task */
  CAN2_Rx_taskHandle = osThreadNew(CAN2_Rx_Task, NULL, &CAN2_Rx_task_attributes);

  /* creation of Serial_Rx_task */
  Serial_Rx_taskHandle = osThreadNew(Serial_Rx_Task, NULL, &Serial_Rx_task_attributes);

  /* creation of DR16_Rx_task */
  DR16_Rx_taskHandle = osThreadNew(DR16_Rx_Task, NULL, &DR16_Rx_task_attributes);

  /* creation of CAN3_Rx_task */
  CAN3_Rx_taskHandle = osThreadNew(CAN3_Rx_Task, NULL, &CAN3_Rx_task_attributes);

  /* creation of Chassis_task */
  Chassis_taskHandle = osThreadNew(Chassis_Task, NULL, &Chassis_task_attributes);

  /* creation of Gimbal_task */
  Gimbal_taskHandle = osThreadNew(Gimbal_Task, NULL, &Gimbal_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
