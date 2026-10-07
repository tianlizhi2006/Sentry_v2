#ifndef __APP_PREFERENCE_H
#define __APP_PREFERENCE_H

// 保留工程原有的 Keil 编译器诊断抑制项。
#pragma diag_suppress 177
#pragma diag_suppress 550
#pragma diag_suppress 3337
// #pragma diag_suppress 1299

// 各输入源使用对应的 Serialx_Ctrl 对象，不要把同一串口分配给多个接收源。
// SerialDatax 枚举值用于标记接收数据来自哪个串口。
#define KEYBOARD_MOUSE_CONTROL_SERIAL Serial10_Ctrl

#define GYRO_SERIAL Serial3_Ctrl

//达妙H7的DBus接口
//#define DR16_SERIAL Serial5_Ctrl
//#define DbusData SerialData5
//#define DR16_SERIAL_Data_Lenth 18
//#define DR16_SERIAL_BAUD 100000

#define DR16_SERIAL Serial5_Ctrl
#define DbusData SerialData5
#define DR16_SERIAL_Data_Lenth 18
#define DR16_SERIAL_BAUD 100000

// 帧头、帧尾和候选长度设为 NULL（0）时，不检查对应条件。
// Buffer_Size 是接收缓冲区容量，需大于允许的帧长度。
// Serialx_ITPending 可选 USART_IT_IDLE USART_IT_RXNE USART_IT_RXNE_AND_IDLE

#define Serial_NORMAL_Mode 0
#define Serial_DMA_Mode 1

#define Serial3_Data_Header 0xAA
#define Serial3_Data_Tail   NULL
#define Serial3_Data_Lenth0 10
#define Serial3_Data_Lenth1 NULL
#define Serial3_Data_Lenth2 NULL
#define Serial3_Data_Lenth3 NULL
#define Serial3_Buffer_Size 40
#define Serial3_Mode        Serial_NORMAL_Mode

#define Serial5_Data_Header NULL
#define Serial5_Data_Tail   NULL
#define Serial5_Data_Lenth0 DR16_SERIAL_Data_Lenth
#define Serial5_Data_Lenth1 NULL
#define Serial5_Data_Lenth2 NULL
#define Serial5_Data_Lenth3 NULL
#define Serial5_Buffer_Size 38
#define Serial5_Mode Serial_NORMAL_Mode


#define Serial10_Data_Header NULL /* 曾为 0xA9; NULL=兼容 0xA9 和 0x5A 两种帧头 */
#define Serial10_Data_Tail   NULL
#define Serial10_Data_Lenth0 21//10
#define Serial10_Data_Lenth1 39 // 自定义键鼠帧: 5B帧头+2B CmdID+30B数据+2B CRC16
#define Serial10_Data_Lenth2 NULL
#define Serial10_Data_Lenth3 NULL
#define Serial10_Buffer_Size 56//42
#define Serial10_Mode        Serial_NORMAL_Mode

#define FDCAN1_Buffer_Size 70
#define FDCAN2_Buffer_Size 70
#define FDCAN3_Buffer_Size 70

//双 Yaw 云台的遥控演示参数
#define GIMBAL_LARGE_YAW_ZERO_RAD          0.538f
#define GIMBAL_SMALL_YAW_ZERO_ECD          1656u
#define GIMBAL_SMALL_YAW_LIMIT_ECD         1000.0f
#define GIMBAL_SMALL_YAW_RELEASE_ECD       500.0f
#define GIMBAL_PITCH_MIN_ANGLE             (-20.0f)
#define GIMBAL_PITCH_MAX_ANGLE             30.0f

//实机坐标系一致；若台架方向相反，只需调整这两个符号。
#define GIMBAL_LARGE_YAW_DIRECTION          (-1.0f)
#define GIMBAL_SMALL_YAW_DIRECTION          (-1.0f)
#define ECD_TO_RAD                          0.000766990394f

//遥控器按键每周期增量。
#define GIMBAL_YAW_RC_SENSITIVITY           0.00015f
#define GIMBAL_PITCH_RC_SENSITIVITY         0.00010f
// 键鼠模式的鼠标位移为每帧增量，单位 deg/count；大位移单帧限幅。
#define GIMBAL_MOUSE_YAW_SENSITIVITY        0.017f
#define GIMBAL_MOUSE_PITCH_SENSITIVITY      0.01f
#define GIMBAL_MOUSE_MAX_DELTA              200.0f
#define GIMBAL_CHASSIS_WZ_FEEDFORWARD       (-1.0f)
// DM4310 大 Yaw：根据 GM6020 小 Yaw 的相对偏角自动回中，输出为力矩。
#define GIMBAL_LARGE_YAW_POSITION_KP        0.09f
#define GIMBAL_LARGE_YAW_POSITION_KI        0.001f
#define GIMBAL_LARGE_YAW_POSITION_KD        0.1f
#define GIMBAL_LARGE_YAW_POSITION_MAX_OUT   30.0f // rad/s，保持当前实际生效的上限
#define GIMBAL_LARGE_YAW_POSITION_MAX_IOUT  0.0f
#define GIMBAL_LARGE_YAW_POSITION_BAND_I    0.0f
#define GIMBAL_LARGE_YAW_SPEED_KP           1.1f  // 大 Yaw 速度环增益
#define GIMBAL_LARGE_YAW_SPEED_KI           0.0f // 每 1 ms 周期累加，PID 内部不乘 dt
#define GIMBAL_LARGE_YAW_SPEED_KD           0.03f
#define GIMBAL_LARGE_YAW_SPEED_MAX_OUT      5.0f
#define GIMBAL_LARGE_YAW_SPEED_MAX_IOUT     0.4f    // Nm，积分输出限幅
#define GIMBAL_LARGE_YAW_SPEED_BAND_I       1.0f    // rad/s，误差绝对值小于此值才累加

// 小 Yaw：角度环/速度环主参数。
#define GIMBAL_SMALL_YAW_POSITION_KP        2.0f
#define GIMBAL_SMALL_YAW_POSITION_KI        0.0f
#define GIMBAL_SMALL_YAW_POSITION_KD        0.0f
#define GIMBAL_SMALL_YAW_POSITION_MAX_OUT   30.0f
#define GIMBAL_SMALL_YAW_POSITION_MAX_IOUT  0.0f
#define GIMBAL_SMALL_YAW_POSITION_BAND_I    0.0f
#define GIMBAL_SMALL_YAW_SPEED_KP           400.0f
#define GIMBAL_SMALL_YAW_SPEED_KI           0.0f
#define GIMBAL_SMALL_YAW_SPEED_KD           0.0f
#define GIMBAL_SMALL_YAW_SPEED_MAX_OUT      MAX_MOTOR_6020_CAN_CURRENT
#define GIMBAL_SMALL_YAW_SPEED_MAX_IOUT     4000.0f
#define GIMBAL_SMALL_YAW_SPEED_BAND_I       3000.0f

// Pitch：角度环 + 速度环，最终力矩限制在 DM4310 范围内。
#define GIMBAL_PITCH_POSITION_KP            0.5f
#define GIMBAL_PITCH_POSITION_KI            0.0f
#define GIMBAL_PITCH_POSITION_KD            3.0f
#define GIMBAL_PITCH_POSITION_MAX_OUT       2.0f
#define GIMBAL_PITCH_POSITION_MAX_IOUT      0.0f
#define GIMBAL_PITCH_POSITION_BAND_I        0.0f
#define GIMBAL_PITCH_SPEED_KP               0.8f
#define GIMBAL_PITCH_SPEED_KI               0.0f
#define GIMBAL_PITCH_SPEED_KD               0.0f
#define GIMBAL_PITCH_SPEED_MAX_OUT          10.0f
#define GIMBAL_PITCH_SPEED_MAX_IOUT         0.0f
#define GIMBAL_PITCH_SPEED_BAND_I           0.0f

// 发射机构，摩擦轮使用 CAN3 的 ID1/ID2，拨弹电机使用 CAN3 的 ID5。
#define FRIC_SPEED_SET_RPM                   6000.0f
#define FRIC_READY_TOLERANCE_RPM             300.0f
#define FRIC_READY_STABLE_TIME_MS             100u

#define FRIC1_SPEED_PID_KP                    10.0f
#define FRIC1_SPEED_PID_KI                     0.1f
#define FRIC1_SPEED_PID_KD                     0.01f
#define FRIC1_SPEED_PID_MAX_OUT            15000.0f
#define FRIC1_SPEED_PID_MAX_IOUT            8000.0f
#define FRIC1_SPEED_PID_BAND_I              3000.0f

#define FRIC2_SPEED_PID_KP                    10.0f
#define FRIC2_SPEED_PID_KI                     0.1f
#define FRIC2_SPEED_PID_KD                     0.01f
#define FRIC2_SPEED_PID_MAX_OUT            15000.0f
#define FRIC2_SPEED_PID_MAX_IOUT            8000.0f
#define FRIC2_SPEED_PID_BAND_I              3000.0f

#define TRIGGER_MOTOR_REVERSE                   1.0f
#define TRIGGER_REDUCTION_RATIO                36.0f
//转一圈的发弹数
#define TRIGGER_ONCE_SHOOT_NUM                  8.0f
#define TRIGGER_SHOOT_FREQUENCY                 10.0f
#define TRIGGER_SPEED_PID_KP                    10.0f
#define TRIGGER_SPEED_PID_KI                     0.5f
#define TRIGGER_SPEED_PID_KD                     0.0f
#define TRIGGER_SPEED_PID_MAX_OUT             9000.0f
#define TRIGGER_SPEED_PID_MAX_IOUT            5000.0f
#define TRIGGER_SPEED_PID_BAND_I              3000.0f

// 拨弹盘卡弹检测与退弹；转速均为电机转子 RPM。
#define TRIGGER_JAM_LOW_SPEED_RPM              675.0f
#define TRIGGER_JAM_START_GRACE_MS             150U
#define TRIGGER_JAM_CONFIRM_MS                 100U
#define TRIGGER_JAM_FEEDBACK_MAX_AGE_MS         20U
#define TRIGGER_JAM_REVERSE_SPEED_RPM          1350.0f
#define TRIGGER_JAM_REVERSE_SLOT_FRACTION         0.5f
#define TRIGGER_JAM_REVERSE_TIMEOUT_MS         300U
#define TRIGGER_JAM_SETTLE_MS                   50U


// 遥控速度映射使用固定上限，不随功率板反馈变化。
#define CHASSIS_RC_MAX_VX_MPS           1.61f
#define CHASSIS_RC_MAX_VY_MPS           1.29f
#define CHASSIS_LITTLE_TOP_MAX_WZ_RADPS 8.0f
#define CHASSIS_LITTLE_TOP_MIN_WZ_RADPS 6.0f
#define CHASSIS_LITTLE_TOP_SECOND_HALF_RATIO (2.0f / 3.0f)
#define CHASSIS_LITTLE_TOP_PERIOD_MS    1800U
#define CHASSIS_LITTLE_TOP_MAX_ACCEL_RADPS2 6.0f

// 遥控器死区
#define CHASSIS_RC_DEADLINE 3

// 四全向轮对称底盘：两组对角电机距离均为 0.50m
#define MOTOR_DISTANCE_TO_CENTER 0.25f
#define OMNI_WHEEL_RADIUS        0.075f

// ID1 所在方向为 +X，+Y 向左，+WZ 为俯视逆时针
#define OMNI_TRANSLATION_FEEDBACK_SCALE 0.5f
#define OMNI_ROTATION_FEEDBACK_SCALE    0.25f

// 底盘任务控制间隔 2 ms。
#define CHASSIS_CONTROL_TIME_MS 2

// GM6020 的 CAN 电流指令幅值上限。
#define MAX_MOTOR_6020_CAN_CURRENT 30000.0f

// 3508 转子 RPM 转轮缘线速度：2*pi*0.075/(60*19)
#define M3508_MOTOR_RPM_TO_VECTOR       4.13367454e-4f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

// 3508 + 19:1 + 0.075m 轮子的演示轮速上限
#define MAX_WHEEL_SPEED            3.5f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 10.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 10.0f
// 哨兵四个 3508 使用同一组速度环参数。
#define CHASSIS_3508_SPEED_PID_KP       5500.0f
#define CHASSIS_3508_SPEED_PID_KI       0.2f
#define CHASSIS_3508_SPEED_PID_KD       0.0f
#define CHASSIS_3508_SPEED_PID_MAX_OUT  8000.0f
#define CHASSIS_3508_SPEED_PID_MAX_IOUT 3000.0f
#define CHASSIS_3508_SPEED_PID_BAND_I   3000.0f

typedef enum
{
	FaultData = 0x00,
	CanData1,
	CanData2,
	CanData3,
	SerialData3,
	SerialData5,
	SerialData10,
	RCData,
	MessageData,
	ChassisData,
	SupercapData,
	ID_e_count
} ID_e;

typedef struct
{
	ID_e Data_ID;
	void *Data_Ptr;
} ID_Data_t;

extern ID_Data_t ID_Data[ID_e_count];

void Prefence_Init(void);
#endif
