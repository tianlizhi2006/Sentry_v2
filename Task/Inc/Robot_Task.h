#ifndef Robot_TASK_H
#define Robot_TASK_H

#include "cmsis_os2.h" // ::CMSIS:RTOS2
#include "string.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "app_preference.h"
#include "app_motor.h"

#include "Message_Task.h"
#include "dev_system.h"

#include "protocol_dbus.h"

#include "algorithm_pid.h"
#include "algorithm_user_lib.h"

#include "drivers_statistic.h"
#include "chassis_power_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

	void Chassis_Task(void *pvParameters);

#ifdef __cplusplus
}
#endif

// 底盘-------------------------------------------------------------------------

extern QueueHandle_t Message_Queue;

struct Chassis_IMU_Data_t
{
	float Last_AngleZ;
	float AngleX;
	float AngleY;
	float AngleZ;
	float Speed_X;
	float Speed_Y;
	float Speed_Z;
	float Acce_X;
	float Acce_Z;
};

// 前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3
// 左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
// 选择底盘状态 开关通道号
#define CHANNEL_LEFT 1
#define CHANNEL_RIGHT 0

typedef struct
{
	const motor_measure_t *chassis_motor_measure;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;
	uint32_t Motor_Message_count;
	float Motor_Messagr_dt;

} Chassis_Motor_t; // 3508 底盘电机数据

typedef struct
{
	bool CAP_ENERGY_STOP; 

} Chassis_Ctrl_Flags_t; // 底盘控制标志位

typedef enum
{
	CHASSIS_NO_MOVE = 0,
	CHASSIS_LITTLE_TOP, // 小陀螺
	CHASSIS_NORMAL_MODE,
} chassis_mode_e; // 底盘工作状态

typedef struct
{
	fp32 vx; // 底盘速度 前进方向 前为正，单位 m/s
	fp32 vy;	//底盘速度 左右方向 左为正  单位 m/s
	fp32 wz;	 // 底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 vx_set; // 底盘设定速度 前进方向 前为正，单位 m/s
	fp32 vy_set; //底盘设定速度 左右方向 左为正，单位 m/s
	fp32 wz_set;  //底盘设定旋转角速度，逆时针为正 单位 rad/s

	fp32 vx_max_speed;  //前进方向最大速度 单位m/s
	fp32 vx_min_speed;  //前进方向最小速度 单位m/s
	fp32 vy_max_speed;  //左右方向最大速度 单位m/s
	fp32 vy_min_speed;  //左右方向最小速度 单位m/s

	fp32 Speed;

} Chassis_Velocity_t;


class Chassis_Ctrl : public Statistic, public ValidData
{
public:
	RC_ctrl_t *RC_Ptr;

	uint32_t Chassis_Task_DWT_Count;
	float Chassis_Task_DWT_dt;

	uint32_t Feed_Back_Count;
	float Feed_Back_dt;

	Chassis_IMU_Data_t IMU_Data;

	// 四个 3508 全向轮，数组顺序对应 CAN ID 0x201~0x204
	Chassis_Motor_t Motor[4];


	PowerClass Power_Ctrl;
	sPidTypeDef Motor_Speed_Pid[4];
	

	Chassis_Velocity_t Velocity;
	Chassis_Ctrl_Flags_t Flags;

	chassis_mode_e Mode;
	bool KeyboardLittleTop;      // 键鼠模式下 C 键切换小陀螺
	bool KeyboardNoForce;        // 键鼠模式下 E 键切换整车无力
	bool LastCPressed;
	bool LastEPressed;
	
	void Chassis_Init(void);
	void Feedback_Update(void);
	void Control(void);
	void Behaviour_Mode(void);
	void Control_loop(void);

private:
	void RC_to_Control(fp32 *vx_set, fp32 *vy_set);
	void Vector_to_Wheel_Speed(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);
	void Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set);
	void Flag_Behaviour_Control(void);
};

extern Chassis_Ctrl *get_chassis_ctrl_pointer(void);

#endif /* __Robot_TASK_H */
