#include "Message_Task.h"
#include "tasks.h"
#include "protocol_crc.h"

Message_Ctrl Message;

void Message_Task(void *pvParameters)
{
    Message.Init();
    for (;;)
    {
        if (xQueueReceive(Message_Queue, &ID_Data[MessageData], portMAX_DELAY))
        {
            Message.Statistic_Update(xTaskGetTickCount());
            if (RC_data_is_error(Message.RC_Ptr))
            {
                slove_RC_lost();
            }
        }
    }
}

void CAN1_Rx_Task(void *pvParameters)
{
    static ID_Data_t rx_data;
    for (;;)
    {
        if (xQueueReceive(CAN1_Rx_Queue, &rx_data, portMAX_DELAY))
        {
            Message.CAN1_Process((CanRxMsg *)rx_data.Data_Ptr);
        }
    }
}

void CAN2_Rx_Task(void *pvParameters)
{
    static ID_Data_t rx_data;
    for (;;)
    {
        if (xQueueReceive(CAN2_Rx_Queue, &rx_data, portMAX_DELAY))
        {
            Message.CAN2_Process((CanRxMsg *)rx_data.Data_Ptr);
        }
    }
}

void CAN3_Rx_Task(void *pvParameters)
{
    static ID_Data_t rx_data;
    for (;;)
    {
        if (xQueueReceive(CAN3_Rx_Queue, &rx_data, portMAX_DELAY))
        {
            Message.CAN3_Process((CanRxMsg *)rx_data.Data_Ptr);
        }
    }
}

void Serial_Rx_Task(void *pvParameters)
{
    static ID_Data_t rx_data;
    for (;;)
    {
        if (xQueueReceive(Serial_Rx_Queue, &rx_data, portMAX_DELAY))
        {
            if (rx_data.Data_ID == SerialData3)
            {
                Message.Serialx_Hook((uint8_t *)rx_data.Data_Ptr, &Serial3_Ctrl);
            }
            else if (rx_data.Data_ID == SerialData5)
            {
                Message.Serialx_Hook((uint8_t *)rx_data.Data_Ptr, &Serial5_Ctrl);
            }
            else if (rx_data.Data_ID == SerialData10)
            {
                Message.Serialx_Hook((uint8_t *)rx_data.Data_Ptr, &Serial10_Ctrl);
            }
        }
    }
}

void DR16_Rx_Task(void *pvParameters)
{
    static uint8_t *rx_message;
    for (;;)
    {
        if (xQueueReceive(DR16_Rx_Queue, &rx_message, portMAX_DELAY))
        {
            uint8_t length = rx_message[0];
            uint8_t *payload = &rx_message[1];

            /* 自定义键鼠帧沿用 0xA5/0x0311 的封装格式。 */
            if (length == 39 && payload[0] == 0xA5)
            {
                uint16_t data_len = payload[1] | (payload[2] << 8);
                uint16_t total_len = 5 + data_len + 2;
                if (data_len >= 2 && total_len == length
                    && Verify_CRC8_Check_Sum(payload, 5)
                    && Verify_CRC16_Check_Sum(payload, total_len)
                    && (payload[5] | (payload[6] << 8)) == 0x0311)
                {
                    custom_keymouse_to_rc(&payload[7], Message.RC_Ptr);
                    RC_mark_frame_received();
                }
            }
            else if (length == 21 && payload[0] == 0xA9 && payload[1] == 0x53)
            {
                serial_to_rc(payload, Message.RC_Ptr);
                RC_mark_frame_received();
            }
            else if (length == 18)
            {
                sbus_to_rc(payload, Message.RC_Ptr);
                RC_mark_frame_received();
                if (RC_data_is_error(Message.RC_Ptr))
                {
                    slove_data_error();
                }
            }
        }
    }
}

void Message_Ctrl::Init(void)
{
    RC_Ptr = &RC_ctrl;
    CAN_ALL_Init();
    Prefence_Init();
    Serial_ALL_Init();
}

void Message_Ctrl::Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl)
{
    if (Serialx_Ctrl == &GYRO_SERIAL)
    {
        Gyro_Chassis_Hook(Rx_Message);
    }
    else if (Serialx_Ctrl == &KEYBOARD_MOUSE_CONTROL_SERIAL || Serialx_Ctrl == &DR16_SERIAL)
    {
        xQueueSend(DR16_Rx_Queue, &Rx_Message, 0);
    }
}

void Message_Ctrl::Gyro_Chassis_Hook(uint8_t *Rx_Message)
{
    if (Rx_Message[1] != 0xAA)
    {
        return;
    }

    if (Rx_Message[2] == 0xA5)
    {
        MPU_DataZ.HHH = Rx_Message[1];
        MPU_DataZ.KEY = Rx_Message[2];
        MPU_DataZ.AngleZ.uint_8[0] = Rx_Message[3];
        MPU_DataZ.AngleZ.uint_8[1] = Rx_Message[4];
        MPU_DataZ.Speed_Z.uint_8[0] = Rx_Message[5];
        MPU_DataZ.Speed_Z.uint_8[1] = Rx_Message[6];
        MPU_DataZ.Acce_Z.uint_8[0] = Rx_Message[7];
        MPU_DataZ.Acce_Z.uint_8[1] = Rx_Message[8];
        MPU_DataZ.Acce_X.uint_8[0] = Rx_Message[9];
        MPU_DataZ.Acce_X.uint_8[1] = Rx_Message[10];
    }
    else if (Rx_Message[2] == 0xA6)
    {
        MPU_DataXY.HHH = Rx_Message[1];
        MPU_DataXY.KEY = Rx_Message[2];
        MPU_DataXY.AngleX.uint_8[0] = Rx_Message[3];
        MPU_DataXY.AngleX.uint_8[1] = Rx_Message[4];
        MPU_DataXY.Speed_X.uint_8[0] = Rx_Message[5];
        MPU_DataXY.Speed_X.uint_8[1] = Rx_Message[6];
        MPU_DataXY.AngleY.uint_8[0] = Rx_Message[7];
        MPU_DataXY.AngleY.uint_8[1] = Rx_Message[8];
        MPU_DataXY.Speed_Y.uint_8[0] = Rx_Message[9];
        MPU_DataXY.Speed_Y.uint_8[1] = Rx_Message[10];
    }
}

static bool ReadCanFrame(CANctrl &can, CanRxMsg &rx_data)
{
    uint8_t marker = 0;
    while (marker != 0xA5)
    {
        marker = can.read();
        if (can.available() <= 12)
        {
            return false;
        }
    }
    if (can.available() <= 12 || can.read() != 0xA6 || can.available() <= 11)
    {
        return false;
    }
    for (uint8_t i = 0; i < 4; ++i)
    {
        rx_data.StdId.u8[i] = can.read();
    }
    for (uint8_t i = 0; i < 8; ++i)
    {
        rx_data.Data[i] = can.read();
    }
    return true;
}

void Message_Ctrl::CAN1_Process(CanRxMsg *Rx_Message)
{
    (void)Rx_Message;
    CanRxMsg rx_data;
    if (!ReadCanFrame(CAN1_Ctrl, rx_data))
    {
        return;
    }

    switch (rx_data.StdId.u32)
    {
    case CAN_POWER_BOARD_GET_ID:
        PowerBoardR.situation = rx_data.Data[0];
        PowerBoardR.mode = rx_data.Data[1];
        PowerBoardR.power = (float)(uint16_t)(rx_data.Data[2] | (rx_data.Data[3] << 8)) * 0.1f;
        PowerBoardR.power_out = (float)(uint16_t)(rx_data.Data[4] | (rx_data.Data[5] << 8)) * 0.1f;
        PowerBoardR.power_limit = rx_data.Data[7];
        PowerBoardR.feedback_dt = DWT_GetDeltaT(&PowerBoardR.feedback_dwt_count);
        ++PowerBoardFeedbackCount;
        break;
    case CAN_DJI_Motor6_ID:
        MA_get_motor_measure(CAN_Cmd.GimbalSmallYaw.GetData(1), rx_data.Data);
        GimbalFeedbackReceived[0] = true;
        GimbalFeedbackTick[0] = xTaskGetTickCount();
        break;
    case DM_Gimbal_LargeYaw_Read_ID:
        Get_DM_Motor_Measure(CAN_Cmd.GimbalLargeYaw.GetData(), CAN_Cmd.GimbalLargeYaw.LIMIT, rx_data.Data);
        GimbalFeedbackReceived[1] = true;
        GimbalFeedbackTick[1] = xTaskGetTickCount();
        break;
    case DM_Gimbal_Pitch_Read_ID:
        Get_DM_Motor_Measure(CAN_Cmd.GimbalPitch.GetData(), CAN_Cmd.GimbalPitch.LIMIT, rx_data.Data);
        GimbalFeedbackReceived[2] = true;
        GimbalFeedbackTick[2] = xTaskGetTickCount();
        break;
    default:
        break;
    }
}

void Message_Ctrl::CAN2_Process(CanRxMsg *Rx_Message)
{
    (void)Rx_Message;
    CanRxMsg rx_data;
    if (!ReadCanFrame(CAN2_Ctrl, rx_data))
    {
        return;
    }

    switch (rx_data.StdId.u32)
    {
    case CAN_DJI_Motor1_ID:
        MA_get_motor_measure(CAN_Cmd.Chassis.GetData(0), rx_data.Data);
        Chassis.Motor[0].Motor_Messagr_dt = DWT_GetDeltaT(&Chassis.Motor[0].Motor_Message_count);
        break;
    case CAN_DJI_Motor2_ID:
        MA_get_motor_measure(CAN_Cmd.Chassis.GetData(1), rx_data.Data);
        Chassis.Motor[1].Motor_Messagr_dt = DWT_GetDeltaT(&Chassis.Motor[1].Motor_Message_count);
        break;
    case CAN_DJI_Motor3_ID:
        MA_get_motor_measure(CAN_Cmd.Chassis.GetData(2), rx_data.Data);
        Chassis.Motor[2].Motor_Messagr_dt = DWT_GetDeltaT(&Chassis.Motor[2].Motor_Message_count);
        break;
    case CAN_DJI_Motor4_ID:
        MA_get_motor_measure(CAN_Cmd.Chassis.GetData(3), rx_data.Data);
        Chassis.Motor[3].Motor_Messagr_dt = DWT_GetDeltaT(&Chassis.Motor[3].Motor_Message_count);
        break;
    default:
        break;
    }
}

void Message_Ctrl::CAN3_Process(CanRxMsg *Rx_Message)
{
    (void)Rx_Message;
    CanRxMsg rx_data;
    if (!ReadCanFrame(CAN3_Ctrl, rx_data))
    {
        return;
    }

    switch (rx_data.StdId.u32)
    {
    case CAN_DJI_Motor1_ID:
        MA_get_motor_measure(CAN_Cmd.Fric.GetData(0), rx_data.Data);
        ShooterFeedbackReceived[0] = true;
        ShooterFeedbackTick[0] = xTaskGetTickCount();
        break;
    case CAN_DJI_Motor2_ID:
        MA_get_motor_measure(CAN_Cmd.Fric.GetData(1), rx_data.Data);
        ShooterFeedbackReceived[1] = true;
        ShooterFeedbackTick[1] = xTaskGetTickCount();
        break;
    case CAN_DJI_Motor5_ID:
        MA_get_motor_measure(CAN_Cmd.Trigger.GetData(0), rx_data.Data);
        ShooterFeedbackReceived[2] = true;
        ShooterFeedbackTick[2] = xTaskGetTickCount();
        break;
    case 0x01:
    {
        GimbalIMU.Pitch_Angle.uint_8[0] = rx_data.Data[0];
        GimbalIMU.Pitch_Angle.uint_8[1] = rx_data.Data[1];
        GimbalIMU.Pitch_Gyro.uint_8[0] = rx_data.Data[2];
        GimbalIMU.Pitch_Gyro.uint_8[1] = rx_data.Data[3];
        GimbalIMU.Yaw_Angle.uint_8[0] = rx_data.Data[4];
        GimbalIMU.Yaw_Angle.uint_8[1] = rx_data.Data[5];
        GimbalIMU.Yaw_Gyro.uint_8[0] = rx_data.Data[6];
        GimbalIMU.Yaw_Gyro.uint_8[1] = rx_data.Data[7];

        const float yaw = GimbalIMU.Yaw_Angle.int_16 * FP32_MPU_RAD;
        GimbalGyro.Error_angle = yaw - GimbalGyro.Last_Yaw_angle;
        if (GimbalGyro.Error_angle > 270.0f)
        {
            GimbalGyro.Yaw_cycle--;
        }
        else if (GimbalGyro.Error_angle < -270.0f)
        {
            GimbalGyro.Yaw_cycle++;
        }

        GimbalGyro.Yaw_angle = yaw + GimbalGyro.Yaw_cycle * 360.0f;
        GimbalGyro.Yaw_speed = GimbalIMU.Yaw_Gyro.int_16 * BMI088_GYRO_2000_SEN;
        GimbalGyro.Pitch_angle = GimbalIMU.Pitch_Angle.int_16 * FP32_MPU_RAD;
        GimbalGyro.Pitch_speed = GimbalIMU.Pitch_Gyro.int_16 * BMI088_GYRO_2000_SEN;
        GimbalGyro.Last_Yaw_angle = yaw;
        GimbalFeedbackReceived[3] = true;
        GimbalFeedbackTick[3] = xTaskGetTickCount();
        break;
    }
    default:
        break;
    }
}

RC_ctrl_t *get_remote_control_point(void)
{
    return &RC_ctrl;
}

Message_Ctrl *get_message_ctrl_pointer(void)
{
    return &Message;
}

bool Message_Ctrl::GimbalFeedbackReady(void) const
{
    const uint32_t now = xTaskGetTickCount();
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (!GimbalFeedbackReceived[i] || (now - GimbalFeedbackTick[i]) > 100u)
        {
            return false;
        }
    }
    return true;
}

bool Message_Ctrl::FrictionFeedbackReady(void) const
{
    // 两颗摩擦轮必须都收到过 CAN3 反馈，并且最近一次反馈距现在不超过 100 ms。
    const uint32_t now = xTaskGetTickCount();
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (!ShooterFeedbackReceived[i] || (now - ShooterFeedbackTick[i]) > 100u)
        {
            return false;
        }
    }
    return true;
}

bool Message_Ctrl::TriggerFeedbackReady(void) const
{
    // 拨弹电机必须收到过 CAN3 的 0x205 反馈，并且最近一次反馈距现在不超过 100 ms。
    const uint32_t now = xTaskGetTickCount();
    if (!ShooterFeedbackReceived[2] || (now - ShooterFeedbackTick[2]) > 100u)
    {
        return false;
    }
    return true;
}
