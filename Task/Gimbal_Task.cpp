#include "Gimbal_Task.h"
#include "tasks.h"
#include "arm_math.h"
#include <math.h>

//重力补偿
float shit = 0.7;
Gimbal_Ctrl Gimbal;

void Gimbal_Task(void *argument)
{
    (void)argument;
    osDelay(5); //预留启动时间
    Gimbal.Init();

    uint32_t last_wake_time = xTaskGetTickCount();
    
    for (;;)
    {
        vTaskDelayUntil(&last_wake_time, GIMBAL_CONTROL_TIME_MS);
        Gimbal.Feedback_Update();
        Gimbal.Behaviour_Mode();
        Gimbal.Control();
        Gimbal.Send();
    }
}

void Gimbal_Ctrl::Init(void)
{
    RC_Ptr = get_remote_control_point();

    // 绑定电机反馈。大 Yaw 为 DM4310；小 Yaw 为 GM6020，反馈 ID 0x206，位于 0x1FF 控制帧第二路。
    LargeYaw.measure = CAN_Cmd.GimbalLargeYaw.Get_DM_Motor_Measure_Pointer();
    SmallYaw.measure = CAN_Cmd.GimbalSmallYaw.Get_Motor_Measure_Pointer(1);
    Pitch.measure = CAN_Cmd.GimbalPitch.Get_DM_Motor_Measure_Pointer();
    Fric1.measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(0);
    Fric2.measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(1);
    Trigger.measure = CAN_Cmd.Trigger.Get_Motor_Measure_Pointer(0);

    //双yaw对准底盘前发的编码值
    LargeYawZeroRad = GIMBAL_LARGE_YAW_ZERO_RAD;
    SmallYawZeroEcd = GIMBAL_SMALL_YAW_ZERO_ECD;

    // 大 Yaw 根据小 Yaw 相对偏角回中，速度环输出力矩。
    PID.Init(&LargeYaw.position_pid, POSITION,
             GIMBAL_LARGE_YAW_POSITION_KP, GIMBAL_LARGE_YAW_POSITION_KI,
             GIMBAL_LARGE_YAW_POSITION_KD, GIMBAL_LARGE_YAW_POSITION_MAX_OUT,
             GIMBAL_LARGE_YAW_POSITION_MAX_IOUT, GIMBAL_LARGE_YAW_POSITION_BAND_I);
    PID.Init(&LargeYaw.speed_pid, POSITION,
             GIMBAL_LARGE_YAW_SPEED_KP, GIMBAL_LARGE_YAW_SPEED_KI,
             GIMBAL_LARGE_YAW_SPEED_KD, GIMBAL_LARGE_YAW_SPEED_MAX_OUT,
             GIMBAL_LARGE_YAW_SPEED_MAX_IOUT, GIMBAL_LARGE_YAW_SPEED_BAND_I);
    // 小 Yaw 使用 IMU 角度/角速度串级闭环，保持对地指向稳定。
    PID.Init(&SmallYaw.position_pid, POSITION,
             GIMBAL_SMALL_YAW_POSITION_KP, GIMBAL_SMALL_YAW_POSITION_KI,
             GIMBAL_SMALL_YAW_POSITION_KD, GIMBAL_SMALL_YAW_POSITION_MAX_OUT,
             GIMBAL_SMALL_YAW_POSITION_MAX_IOUT, GIMBAL_SMALL_YAW_POSITION_BAND_I);
    PID.Init(&SmallYaw.speed_pid, POSITION,
             GIMBAL_SMALL_YAW_SPEED_KP, GIMBAL_SMALL_YAW_SPEED_KI,
             GIMBAL_SMALL_YAW_SPEED_KD, GIMBAL_SMALL_YAW_SPEED_MAX_OUT,
             GIMBAL_SMALL_YAW_SPEED_MAX_IOUT, GIMBAL_SMALL_YAW_SPEED_BAND_I);
    // Pitch 同样使用 IMU 串级闭环，输出在 Control() 中叠加重力补偿。
    PID.Init(&Pitch.position_pid, POSITION,
             GIMBAL_PITCH_POSITION_KP, GIMBAL_PITCH_POSITION_KI,
             GIMBAL_PITCH_POSITION_KD, GIMBAL_PITCH_POSITION_MAX_OUT,
             GIMBAL_PITCH_POSITION_MAX_IOUT, GIMBAL_PITCH_POSITION_BAND_I);
    PID.Init(&Pitch.speed_pid, POSITION,
             GIMBAL_PITCH_SPEED_KP, GIMBAL_PITCH_SPEED_KI,
             GIMBAL_PITCH_SPEED_KD, GIMBAL_PITCH_SPEED_MAX_OUT,
             GIMBAL_PITCH_SPEED_MAX_IOUT, GIMBAL_PITCH_SPEED_BAND_I);

    // 发射机构两个摩擦轮转向相反，拨弹盘连续转动。
    PID.Init(&Fric1.speed_pid, POSITION,
             FRIC1_SPEED_PID_KP, FRIC1_SPEED_PID_KI, FRIC1_SPEED_PID_KD,
             FRIC1_SPEED_PID_MAX_OUT, FRIC1_SPEED_PID_MAX_IOUT, FRIC1_SPEED_PID_BAND_I);
    PID.Init(&Fric2.speed_pid, POSITION,
             FRIC2_SPEED_PID_KP, FRIC2_SPEED_PID_KI, FRIC2_SPEED_PID_KD,
             FRIC2_SPEED_PID_MAX_OUT, FRIC2_SPEED_PID_MAX_IOUT, FRIC2_SPEED_PID_BAND_I);
    PID.Init(&Trigger.speed_pid, POSITION,
             TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD,
             TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_BAND_I);

    // DM 电机上电后需要使能；DJI 小 Yaw 无需使能帧。
    CAN_Cmd.DM_Motor_Enable(&CAN_Cmd.GimbalLargeYaw);
    CAN_Cmd.DM_Motor_Enable(&CAN_Cmd.GimbalPitch);

    Mode = GIMBAL_NO_MOVE;
    Last_Mode = GIMBAL_NO_MOVE;
    Initialized = false;
    LargeYaw.speed_set = 0.0f;
    SmallYawLimited = false;
    RelativeYawRad = 0.0f;
    FricReadyCount = 0u;
    FricKeyLatched = false;
    FricKeyWasPressed = false;
    Flags.Fric_Flag = false;
    Flags.Shoot_Flag = false;
    Flags.Fric_Ready_Flag = false;
    Flags.Heat_Allow_Flag = true;

    Feedback_Update();
}

void Gimbal_Ctrl::Feedback_Update(void)
{

	
	
    // 大 Yaw 反馈使用 DM4310 自身编码器，去零点后转为角度。
    LargeYaw.angle = RelativeDMToDeg(LargeYaw.measure->POS.fdata, LargeYawZeroRad);
    LargeYaw.speed = LargeYaw.measure->VEl.fdata;

    // relative_ecd 只表示小 Yaw 相对大 Yaw 的机械偏角；控制角度使用 IMU 绝对 Yaw。
    SmallYaw.relative_ecd = WrapEcd((float)SmallYaw.measure->ecd - (float)SmallYawZeroEcd);
    SmallYaw.angle = Message.GimbalGyro.Yaw_angle;
    SmallYaw.speed = Message.GimbalGyro.Yaw_speed;

	
    //IMU 的 Pitch 正方向取反。
    Pitch.angle = -Message.GimbalGyro.Pitch_angle;
    Pitch.speed = -Message.GimbalGyro.Pitch_speed;

    // 发射电机反馈均为 DJI 转子 RPM；失联保护在 Launcher_Control() 中处理。
    if (Message.FrictionFeedbackReady())
    {
        Fric1.speed = Fric1.measure->speed_rpm;
        Fric2.speed = Fric2.measure->speed_rpm;
    }
    if (Message.TriggerFeedbackReady())
    {
        Trigger.speed = Trigger.measure->speed_rpm;
    }

    // 最终朝向 = 大 Yaw 相对底盘角 + 小 Yaw 相对大 Yaw 角。
    // 该值供底盘做坐标旋转，从而保证以云台朝向为基准。
    const float large_relative_rad = GIMBAL_LARGE_YAW_DIRECTION * LargeYaw.angle * DEG_TO_RAD;
    const float small_relative_rad = GIMBAL_SMALL_YAW_DIRECTION * SmallYaw.relative_ecd * ECD_TO_RAD;
    RelativeYawRad = rad_format(large_relative_rad + small_relative_rad);

    // 首次四路反馈就绪时以当前姿态为目标，防止上电突跳。
    if (!Initialized && Message.GimbalFeedbackReady())
    {
        SmallYaw.angle_set = SmallYaw.angle;
        Pitch.angle_set = Pitch.angle;
        Initialized = true;
    }
}

void Gimbal_Ctrl::Behaviour_Mode(void)
{
    //记录上一次模式
    Last_Mode = Mode;
    // 云台任一必要反馈失联时停止输出，恢复后重新建立目标。
    if (!Message.GimbalFeedbackReady())
    {
        Initialized = false;
        Mode = GIMBAL_NO_MOVE;
    }
    else if (RC_data_is_error(RC_Ptr) || switch_is_down(RC_Ptr->rc.s[GIMBAL_RIGHT_SWITCH])
        || Chassis.KeyboardNoForce)
    {
        Mode = GIMBAL_NO_MOVE;
    }
    else
    {
        Mode = GIMBAL_REMOTE_CONTROL;
    }

    // 模式跳变时重置目标与 PID 历史量，避免恢复输出时突然转动。
    if (Mode != Last_Mode)
    {
        SmallYaw.angle_set = SmallYaw.angle;
        Pitch.angle_set = Pitch.angle;
        SmallYawLimited = false;
        LargeYaw.speed_set = 0.0f;
        PID.Clear(&LargeYaw.position_pid);
        PID.Clear(&LargeYaw.speed_pid);
        PID.Clear(&SmallYaw.position_pid);
        PID.Clear(&SmallYaw.speed_pid);
        PID.Clear(&Pitch.position_pid);
        PID.Clear(&Pitch.speed_pid);
    }

    Launcher_Behaviour();
}

void Gimbal_Ctrl::Control(void)
{
    int16_t mouse_dx = 0;
    int16_t mouse_dy = 0;
    RC_take_mouse_delta(&mouse_dx, &mouse_dy);

    //无力模式下不输出力矩
    if (Mode == GIMBAL_NO_MOVE)
    {
        LargeYaw.speed_set = 0.0f;
        LargeYaw.torque_set = 0.0f;
        SmallYaw.give_current = 0;
        Pitch.torque_set = 0.0f;
        Launcher_Reset();
        return;
    }

    // 双拨杆向上为键鼠模式；鼠标位移每接收帧只积分一次。
    const bool mouse_mode = switch_is_up(RC_Ptr->rc.s[GIMBAL_RIGHT_SWITCH])
        && switch_is_up(RC_Ptr->rc.s[GIMBAL_LEFT_SWITCH]);
    const float yaw_delta = mouse_mode
        ? fp32_constrain((float)mouse_dx, -GIMBAL_MOUSE_MAX_DELTA, GIMBAL_MOUSE_MAX_DELTA)
            * GIMBAL_MOUSE_YAW_SENSITIVITY
        : Deadband(RC_Ptr->rc.ch[GIMBAL_YAW_CHANNEL], GIMBAL_RC_DEADLINE)
            * GIMBAL_YAW_RC_SENSITIVITY;
    const float pitch_delta = mouse_mode
        ? -fp32_constrain((float)mouse_dy, -GIMBAL_MOUSE_MAX_DELTA, GIMBAL_MOUSE_MAX_DELTA)
            * GIMBAL_MOUSE_PITCH_SENSITIVITY
        : -Deadband(RC_Ptr->rc.ch[GIMBAL_PITCH_CHANNEL], GIMBAL_RC_DEADLINE)
            * GIMBAL_PITCH_RC_SENSITIVITY;

    Pitch.angle_set -= pitch_delta;
    //约束pitch角度
    Pitch.angle_set = fp32_constrain(Pitch.angle_set, GIMBAL_PITCH_MIN_ANGLE, GIMBAL_PITCH_MAX_ANGLE);

    // 小 Yaw 软限幅采用滞回：超过 1000 ECD（约 44°）冻结目标，
    // 待大 Yaw 将其拉回 500 ECD（约 22°）内再重新接受遥控输入。
    if (!SmallYawLimited && fabsf(SmallYaw.relative_ecd) > GIMBAL_SMALL_YAW_LIMIT_ECD)
    {
        SmallYawLimited = true;
        SmallYaw.angle_set = SmallYaw.angle;
    }
    else if (SmallYawLimited && fabsf(SmallYaw.relative_ecd) < GIMBAL_SMALL_YAW_RELEASE_ECD)
    {
        SmallYawLimited = false;
        // 从冻结的目标继续接收增量，避免释放时跳到瞬时 IMU 角度。
    }
    if (!SmallYawLimited)
    {
        SmallYaw.angle_set -= yaw_delta;
    }

    // 大 Yaw：小 Yaw 机械偏角 -> 大 Yaw 目标速度 -> DM4310 力矩。
    PID.Calc(&LargeYaw.position_pid, SmallYaw.relative_ecd * ECD_TO_DEG, 0.0f);
    // 叠加底盘旋转速度前馈后，最终目标限制在 DM 速度协议范围内。
    LargeYaw.speed_set = fp32_constrain(LargeYaw.position_pid.out
        + Chassis.Velocity.wz_set * GIMBAL_CHASSIS_WZ_FEEDFORWARD,
        DM_4310_V_MIN, DM_4310_V_MAX);
    PID.Calc(&LargeYaw.speed_pid, LargeYaw.speed, LargeYaw.speed_set);
    LargeYaw.torque_set = LargeYaw.speed_pid.out;

    // 小 Yaw：IMU 绝对角 -> 目标角速度 -> DJI 电流。
    PID.Calc(&SmallYaw.position_pid, SmallYaw.angle, SmallYaw.angle_set);
    PID.Calc(&SmallYaw.speed_pid, SmallYaw.speed, SmallYaw.position_pid.out);
    SmallYaw.give_current = (int16_t)fp32_constrain(
        SmallYaw.speed_pid.out, -MAX_MOTOR_6020_CAN_CURRENT, MAX_MOTOR_6020_CAN_CURRENT);

    // Pitch：IMU 角度 -> 目标角速度 -> DM 力矩。
    PID.Calc(&Pitch.position_pid, Pitch.angle, Pitch.angle_set);
    PID.Calc(&Pitch.speed_pid, Pitch.speed, Pitch.position_pid.out);
    Pitch.torque_set = Pitch.speed_pid.out + shit;

    Launcher_Control();

}

void Gimbal_Ctrl::Send(void)
{
    // 无力模式发送显式零指令，避免保留上一周期输出。
    if (Mode == GIMBAL_NO_MOVE)
    {
        CAN_Cmd.DM_SendData(&CAN_Cmd.GimbalLargeYaw, 0, 0, 0, 0, 0);
        CAN_Cmd.SendData(&CAN_Cmd.GimbalSmallYaw, 0, 0);
        CAN_Cmd.DM_SendData(&CAN_Cmd.GimbalPitch, 0, 0, 0, 0, 0);
        CAN_Cmd.SendData(&CAN_Cmd.Fric, 0, 0);
        CAN_Cmd.SendData(&CAN_Cmd.Trigger, 0);
        return;
    }

    // DM4310 大 Yaw/Pitch 使用 MIT 力矩前馈；GM6020 小 Yaw 位于 0x1FF 帧的第二个电流槽。
    CAN_Cmd.DM_SendData(&CAN_Cmd.GimbalLargeYaw, 0, 0, 0, 0, LargeYaw.torque_set);
    CAN_Cmd.SendData(&CAN_Cmd.GimbalSmallYaw, 0, SmallYaw.give_current);
    CAN_Cmd.DM_SendData(&CAN_Cmd.GimbalPitch, 0, 0, 0, 0, Pitch.torque_set);
    CAN_Cmd.SendData(&CAN_Cmd.Fric, Fric1.give_current, Fric2.give_current);
    CAN_Cmd.SendData(&CAN_Cmd.Trigger, Trigger.give_current);
}

void Gimbal_Ctrl::Launcher_Behaviour(void)
{
    Flags.Fric_Flag = false;
    Flags.Shoot_Flag = false;

    const bool mouse_mode = switch_is_up(RC_Ptr->rc.s[GIMBAL_RIGHT_SWITCH])
        && switch_is_up(RC_Ptr->rc.s[GIMBAL_LEFT_SWITCH]);
    const bool f_pressed = (RC_Ptr->key.v & KEY_PRESSED_OFFSET_F) != 0;
    if (Mode != GIMBAL_REMOTE_CONTROL || !mouse_mode)
    {
        FricKeyLatched = false;
    }
    else if (f_pressed && !FricKeyWasPressed)
    {
        FricKeyLatched = !FricKeyLatched;
    }
    FricKeyWasPressed = f_pressed;

    if (Mode != GIMBAL_REMOTE_CONTROL)
    {
        return;
    }
    if (mouse_mode)
    {
        Flags.Fric_Flag = FricKeyLatched;
        Flags.Shoot_Flag = FricKeyLatched && RC_Ptr->mouse.press_l != 0;
        return;
    }
    if (!switch_is_mid(RC_Ptr->rc.s[GIMBAL_RIGHT_SWITCH]))
    {
        return;
    }

    // 左中：只开摩擦轮；左上：保持摩擦轮并请求拨弹；左下：全部关闭。
    if (switch_is_mid(RC_Ptr->rc.s[GIMBAL_LEFT_SWITCH]))
    {
        Flags.Fric_Flag = true;
    }
    else if (switch_is_up(RC_Ptr->rc.s[GIMBAL_LEFT_SWITCH]))
    {
        Flags.Fric_Flag = true;
        Flags.Shoot_Flag = true;
    }
}

void Gimbal_Ctrl::Launcher_Control(void)
{
    // CAN3 通信状态：
    // fric_feedback_ready 要求 0x201、0x202 两个摩擦轮反馈均未超过 100 ms；
    // trigger_feedback_ready 要求拨弹电机 0x205 反馈未超过 100 ms。
    // 这两个变量只表示收到了电机的反馈数据，不表示摩擦轮已经达到目标转速。
    const bool fric_feedback_ready = Message.FrictionFeedbackReady();
    const bool trigger_feedback_ready = Message.TriggerFeedbackReady();

    if (Flags.Fric_Flag && fric_feedback_ready)
    {
        Fric1.speed_set = FRIC_SPEED_SET_RPM;
        Fric2.speed_set = -FRIC_SPEED_SET_RPM;
    }
    else
    {
        Fric1.speed_set = 0.0f;
        Fric2.speed_set = 0.0f;
        FricReadyCount = 0u;
        Flags.Fric_Ready_Flag = false;
    }

    if (fric_feedback_ready)
    {
        PID.Calc(&Fric1.speed_pid, Fric1.speed, Fric1.speed_set);
        PID.Calc(&Fric2.speed_pid, Fric2.speed, Fric2.speed_set);
        Fric1.give_current = (int16_t)fp32_constrain(
            Fric1.speed_pid.out, -FRIC1_SPEED_PID_MAX_OUT, FRIC1_SPEED_PID_MAX_OUT);
        Fric2.give_current = (int16_t)fp32_constrain(
            Fric2.speed_pid.out, -FRIC2_SPEED_PID_MAX_OUT, FRIC2_SPEED_PID_MAX_OUT);
    }
    else
    {
        PID.Clear(&Fric1.speed_pid);
        PID.Clear(&Fric2.speed_pid);
        Fric1.give_current = 0;
        Fric2.give_current = 0;
    }

    // 锁存式启动联锁：关闭摩擦轮或反馈失联时解除锁存。
    // 启动阶段要求两轮连续到速100 ms；锁存后不因弹丸通过造成的瞬时转速跌落而中断拨弹。
    if (!Flags.Fric_Flag || !fric_feedback_ready)
    {
        FricReadyCount = 0u;
        Flags.Fric_Ready_Flag = false;
    }
    else if (!Flags.Fric_Ready_Flag)
    {
        if (fabsf(Fric1.speed_set - Fric1.speed) <= FRIC_READY_TOLERANCE_RPM
            && fabsf(Fric2.speed_set - Fric2.speed) <= FRIC_READY_TOLERANCE_RPM)
        {
            if (FricReadyCount < FRIC_READY_STABLE_TIME_MS)
            {
                FricReadyCount++;
            }
            if (FricReadyCount >= FRIC_READY_STABLE_TIME_MS)
            {
                Flags.Fric_Ready_Flag = true;
            }
        }
        else
        {
            FricReadyCount = 0u;
        }
    }

    // 最终拨弹许可由五道条件共同决定：
    // 1. Fric_Flag，已经打开摩擦轮；
    // 2. Shoot_Flag，把左拨杆拨到上挡，请求发射；
    // 3. Fric_Ready_Flag，两颗摩擦轮已经达到目标速度并连续稳定 100 ms；
    // 4. Heat_Allow_Flag：预留的裁判系统热量许可。当前没有裁判系统，初始化后保持 true；
    // 5. trigger_feedback_ready，拨弹盘电机在线标志位；
    if (Flags.Fric_Flag && Flags.Shoot_Flag && Flags.Fric_Ready_Flag
        && Flags.Heat_Allow_Flag && trigger_feedback_ready)
    {
        Trigger.speed_set = TRIGGER_MOTOR_REVERSE * TRIGGER_SHOOT_FREQUENCY
                          * 60.0f / TRIGGER_ONCE_SHOOT_NUM * TRIGGER_REDUCTION_RATIO;
    }
    else
    {
        Trigger.speed_set = 0.0f;
    }

    if (trigger_feedback_ready)
    {
        PID.Calc(&Trigger.speed_pid, Trigger.speed, Trigger.speed_set);
        Trigger.give_current = (int16_t)fp32_constrain(
            Trigger.speed_pid.out, -TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_OUT);
    }
    else
    {
        PID.Clear(&Trigger.speed_pid);
        Trigger.give_current = 0;
    }
}

void Gimbal_Ctrl::Launcher_Reset(void)
{
    Fric1.speed_set = 0.0f;
    Fric2.speed_set = 0.0f;
    Trigger.speed_set = 0.0f;
    Fric1.give_current = 0;
    Fric2.give_current = 0;
    Trigger.give_current = 0;
    FricReadyCount = 0u;
    Flags.Fric_Ready_Flag = false;
    PID.Clear(&Fric1.speed_pid);
    PID.Clear(&Fric2.speed_pid);
    PID.Clear(&Trigger.speed_pid);
}

float Gimbal_Ctrl::GetRelativeYawRad(void) const
{
    // 反馈未就绪时返回 0，底盘同时会被 Initialized 安全门禁止输出。
    if (Initialized)
    {
        return RelativeYawRad;
    }
    return 0.0f;
}

float Gimbal_Ctrl::Deadband(int16_t input, int16_t deadband)
{
    if (input > -deadband && input < deadband)
    {
        return 0.0f;
    }
    return (float)input;
}

float Gimbal_Ctrl::WrapEcd(float ecd)
{
    // 8192 线编码器跨零点时，选择绝对值最小的相对路径。
    if (ecd > 4096.0f)
    {
        ecd -= 8192.0f;
    }
    else if (ecd < -4096.0f)
    {
        ecd += 8192.0f;
    }
    return ecd;
}

float Gimbal_Ctrl::RelativeDMToDeg(float angle, float zero)
{
    // DM 位置反馈为弧度，先去零点并包装到 [-pi, pi]，再转为度。
    float relative = angle - zero;
    if (relative > PI)
    {
        relative -= 2.0f * PI;
    }
    else if (relative < -PI)
    {
        relative += 2.0f * PI;
    }
    return relative * RAD_TO_DEG;
}
