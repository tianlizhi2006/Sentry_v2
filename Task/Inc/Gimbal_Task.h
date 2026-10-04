#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "cmsis_os2.h"
#include "app_motor.h"
#include "algorithm_pid.h"
#include "drivers_statistic.h"
#include "protocol_dbus.h"

/*
 * 哨兵遥控演示云台
 *
 * 机械结构为串联双 Yaw：
 *   底盘 -> 大 Yaw(DM4310) -> 小 Yaw(GM6020) -> Pitch(DM4310)
 * 小 Yaw 负责跟踪遥控目标，大 Yaw 负责将小 Yaw 的相对偏角拉回中位。
 * 本模块只保留遥控逻辑，不包含视觉和导航；普通模式使用拨杆控制发射，
 * 双拨杆向上的键鼠模式使用 F 键和鼠标左键。
 */

#ifdef __cplusplus
extern "C" {
#endif

void Gimbal_Task(void *argument);

#ifdef __cplusplus
}
#endif

// 任务周期和 DR16 通道映射。
#define GIMBAL_CONTROL_TIME_MS 1  // 1 kHz 控制周期
#define GIMBAL_DM_ENABLE_RETRY_MS 50U // 未使能时每 50 ms 清错并重新使能
#define GIMBAL_YAW_CHANNEL     0  // 右摇杆水平
#define GIMBAL_PITCH_CHANNEL   1  // 右摇杆竖直
#define GIMBAL_RIGHT_SWITCH    0  // 右拨杆下档为无力
#define GIMBAL_LEFT_SWITCH     1  // 左拨杆控制摩擦轮和拨弹盘
#define GIMBAL_RC_DEADLINE     20 // 摇杆中位死区

// DM 电机的反馈、目标与串级 PID 状态。
struct GimbalDMMotor
{
    const DM_Motor_measure_t *measure; // CAN 反馈指针
    uint32_t last_enable_attempt_tick;
    bool enable_attempted;
    float angle;                       // deg，控制反馈角度
    float speed;                       // rad/s
    float speed_set;                   // rad/s，送入速度环的实际目标
    float angle_set;                   // deg
    float torque_set;                  // Nm，MIT 模式前馈力矩
    sPidTypeDef position_pid;           // 角度环 -> 目标速度
    sPidTypeDef speed_pid;              // 速度环 -> 力矩
};

// 小 Yaw DJI 电机的反馈、目标与串级 PID 状态。
struct GimbalDJIMotor
{
    const motor_measure_t *measure; // 0x206 反馈指针
    float angle;                    // deg，来自云台 IMU 的连续 Yaw
    float speed;                    // rad/s，来自云台 IMU
    float angle_set;                // deg，遥控积分得到的目标角
    float relative_ecd;             // ECD，小 Yaw 相对大 Yaw 的最短偏差
    int16_t give_current;            // DJI CAN 电流指令
    sPidTypeDef position_pid;        // 角度环 -> 目标速度
    sPidTypeDef speed_pid;           // 速度环 -> 电流
};

// 摩擦轮和拨弹盘均使用 DJI 电机速度环，输出为 CAN 电流指令。
struct LauncherDJIMotor
{
    const motor_measure_t *measure;
    float speed;
    float speed_set;
    int16_t give_current;
    sPidTypeDef speed_pid;
};

// 保留允许标志，后续接入裁判系统时只需更新 Heat_Allow_Flag。
struct Gimbal_Ctrl_Flags_t
{
    bool Fric_Flag;          // 遥控请求：左拨杆中/上挡时允许摩擦轮运行
    bool Shoot_Flag;         // 遥控请求：左拨杆上挡时请求拨弹，不等于已经实际输出
    bool Fric_Ready_Flag;    // 到速联锁：两个摩擦轮均已到速并连续稳定 100 ms
    bool Heat_Allow_Flag;    // 热量联锁：预留给裁判系统；当前无裁判系统，固定允许
};

enum gimbal_mode_e
{
    GIMBAL_NO_MOVE = 0,
    GIMBAL_REMOTE_CONTROL,
};

class Gimbal_Ctrl : public Statistic, public ValidData
{
public:
    RC_ctrl_t *RC_Ptr;       // DR16 解析结果
    GimbalDMMotor LargeYaw;  // DM4310 下级粗调 Yaw，使 GM6020 小 Yaw 自动回中
    GimbalDJIMotor SmallYaw; // 上级精调 Yaw，跟踪驾驶员目标
    GimbalDMMotor Pitch;     // 俯仰轴
    LauncherDJIMotor Fric1;  // 摩擦轮1，目标为正转
    LauncherDJIMotor Fric2;  // 摩擦轮2，目标为反转
    LauncherDJIMotor Trigger;// 拨弹盘
    Gimbal_Ctrl_Flags_t Flags;
    gimbal_mode_e Mode;
    gimbal_mode_e Last_Mode;

    volatile bool Initialized;       // 四路云台必要反馈就绪后置位
    bool SmallYawLimited;            // 小 Yaw 是否已进入软限幅滞回区
    volatile float RelativeYawRad;   // 完整云台朝向相对底盘的角度
    uint16_t FricReadyCount;         // 摩擦轮连续到速计数，1计数约1 ms

    bool FricKeyLatched;             // 键鼠模式下 F 键切换的摩擦轮状态
    bool FricKeyWasPressed;          // 上周期 F 键状态，用于检测按下沿

    void Init(void);
    void Feedback_Update(void);
    void Behaviour_Mode(void);
    void Control(void);
    void Send(void);

    // 底盘用该角度将“以云台为前”的速度旋转到车体坐标系。
    float GetRelativeYawRad(void) const;

private:
    float LargeYawZeroRad;            // 大 Yaw 对正底盘时的 DM4310 编码零点
    uint16_t SmallYawZeroEcd;         // 小 Yaw 对正大 Yaw 时的 GM6020 ECD 零点
    static float Deadband(int16_t input, int16_t deadband); // DR16 死区
    static float WrapEcd(float ecd);                        // ECD -> [-4096, 4096]
    static float RelativeDMToDeg(float angle, float zero);  // DM rad -> 相对角 deg
    void Launcher_Behaviour(void);
    void Launcher_Control(void);
    void Launcher_Reset(void);
    void RecoverDmMotors(void);
};

extern Gimbal_Ctrl Gimbal;

#endif
