#ifndef MESSAGE_TASK_H
#define MESSAGE_TASK_H

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "app_preference.h"
#include "dev_can.h"
#include "app_serial.h"
#include "protocol_dbus.h"
#include "drivers_statistic.h"
#include "app_motor.h"
#include "bsp_dwt.h"

#ifdef __cplusplus
extern "C" {
#endif

void Message_Task(void *pvParameters);
void CAN1_Rx_Task(void *pvParameters);
void CAN2_Rx_Task(void *pvParameters);
void CAN3_Rx_Task(void *pvParameters);
void Serial_Rx_Task(void *pvParameters);
void DR16_Rx_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

extern QueueHandle_t Message_Queue;
extern QueueHandle_t CAN1_Rx_Queue;
extern QueueHandle_t CAN2_Rx_Queue;
extern QueueHandle_t CAN3_Rx_Queue;
extern QueueHandle_t Serial_Rx_Queue;
extern QueueHandle_t DR16_Rx_Queue;

#define CAP_CLOSE 0x0f
#define CAP_OPEN  0xf0

#define FP32_MPU_RAD          0.005494505494505494f
#define BMI088_GYRO_2000_SEN 0.0010652644360316953f
#define ANGLE_TO_RAD          0.017453292519943295f

union int16_t_uint8_t
{
    uint8_t uint_8[2];
    int16_t int_16;
};

struct MPU_Data_tZ
{
    uint8_t HHH;
    uint8_t KEY;
    int16_t_uint8_t AngleZ;
    int16_t_uint8_t Speed_Z;
    int16_t_uint8_t Acce_Z;
    int16_t_uint8_t Acce_X;
};

struct MPU_Data_tXY
{
    uint8_t HHH;
    uint8_t KEY;
    int16_t_uint8_t AngleX;
    int16_t_uint8_t Speed_X;
    int16_t_uint8_t AngleY;
    int16_t_uint8_t Speed_Y;
};

struct Gimbal_IMU_Data_t
{
    int16_t_uint8_t Pitch_Angle;
    int16_t_uint8_t Pitch_Gyro;
    int16_t_uint8_t Yaw_Angle;
    int16_t_uint8_t Yaw_Gyro;
};

struct Gimbal_Gyro_Data_t
{
    float Pitch_angle;
    float Pitch_speed;
    float Yaw_angle;
    float Yaw_speed;
    float Last_Yaw_angle;
    float Error_angle;
    int32_t Yaw_cycle;
};

struct supercap_Receive_Data_t
{
    uint8_t situation;
    uint8_t mode;
    float power;
    float power_all;
    uint8_t energy;
    uint8_t power_limit;
    uint32_t Motor_Message_count;
    float Motor_Messagr_dt;
};

class Message_Ctrl : public Statistic
{
public:
    supercap_Receive_Data_t SuperCapR;
    MPU_Data_tZ MPU_DataZ;
    MPU_Data_tXY MPU_DataXY;
    Gimbal_IMU_Data_t GimbalIMU;
    Gimbal_Gyro_Data_t GimbalGyro;
    bool GimbalFeedbackReceived[4];
    uint32_t GimbalFeedbackTick[4];
    bool ShooterFeedbackReceived[3]; // 摩擦轮1、摩擦轮2、拨弹电机
    uint32_t ShooterFeedbackTick[3];
    RC_ctrl_t *RC_Ptr;

    void Init(void);
    void CAN1_Process(CanRxMsg *Rx_Message);
    void CAN2_Process(CanRxMsg *Rx_Message);
    void CAN3_Process(CanRxMsg *Rx_Message);
    void Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl);
    void Gyro_Chassis_Hook(uint8_t *Rx_Message);
    bool GimbalFeedbackReady(void) const;
    bool FrictionFeedbackReady(void) const;
    bool TriggerFeedbackReady(void) const;
};

extern Message_Ctrl Message;
Message_Ctrl *get_message_ctrl_pointer(void);
RC_ctrl_t *get_remote_control_point(void);

#endif
