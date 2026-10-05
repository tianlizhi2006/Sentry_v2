#include "tasks.h" /*大部分Class的.h文件引入在这*/
#include "arm_math.h"

/*
 *
 *                        ::
 *                       :;J7, :,                        ::;7:
 *                       ,ivYi, ,                       ;LLLFS:
 *                       :iv7Yi                       :7ri;j5PL
 *                      ,:ivYLvr                    ,ivrrirrY2X,
 *                      :;r@Wwz.7r:                :ivu@kexianli.
 *                     :iL7::,:::iiirii:ii;::::,,irvF7rvvLujL7ur
 *                    ri::,:,::i:iiiiiii:i:irrv177JX7rYXqZEkvv17
 *                 ;i:, , ::::iirrririi:i:::iiir2XXvii;L8OGJr71i
 *               :,, ,,:   ,::ir@mingyi.irii:i:::j1jri7ZBOS7ivv,
 *                  ,::,    ::rv77iiiriii:iii:i::,rvLq@huhao.Li
 *              ,,      ,, ,:ir7ir::,:::i;ir:::i:i::rSGGYri712:
 *            :::  ,v7r:: ::rrv77:, ,, ,:i7rrii:::::, ir7ri7Lri
 *           ,     2OBBOi,iiir;r::        ,irriiii::,, ,iv7Luur:
 *         ,,     i78MBBi,:,:::,:,  :7FSL: ,iriii:::i::,,:rLqXv::
 *         :      iuMMP: :,:::,:ii;2GY7OBB0viiii:i:iii:i:::iJqL;::
 *        ,     ::::i   ,,,,, ::LuBBu BBBBBErii:i:i:i:i:i:i:r77ii
 *       ,       :       , ,,:::rruBZ1MBBqi, :,,,:::,::::::iiriri:
 *      ,               ,,,,::::i:  @arqiao.       ,:,, ,:::ii;i7:
 *     :,       rjujLYLi   ,,:::::,:::::::::,,   ,:i,:,,,,,::i:iii
 *     ::      BBBBBBBBB0,    ,,::: , ,:::::: ,      ,,,, ,,:::::::
 *     i,  ,  ,8BMMBBBBBBi     ,,:,,     ,,, , ,   , , , :,::ii::i::
 *     :      iZMOMOMBBM2::::::::::,,,,     ,,,,,,:,,,::::i:irr:i:::,
 *     i   ,,:;u0MBMOG1L:::i::::::  ,,,::,   ,,, ::::::i:i:iirii:i:i:
 *     :    ,iuUuuXUkFu7i:iii:i:::, :,:,: ::::::::i:i:::::iirr7iiri::
 *     :     :rk@Yizero.i:::::, ,:ii:::::::i:::::i::,::::iirrriiiri::,
 *      :      5BMBBBBBBSr:,::rv2kuii:::iii::,:i:,, , ,,:,:i@petermu.,
 *           , :r50EZ8MBBBBGOBBBZP7::::i::,:::::,: :,:,::i;rrririiii::
 *               :jujYY7LS0ujJL7r::,::i::,::::::::::::::iirirrrrrrr:ii:
 *            ,:  :@kevensun.:,:,,,::::i:i:::::,,::::::iir;ii;7v77;ii;i,
 *            ,,,     ,,:,::::::i:iiiii:i::::,, ::::iiiir@xingjief.r;7:i,
 *         , , ,,,:,,::::::::iiiiiiiiii:,:,:::::::::iiir;ri7vL77rrirri::
 *          :,, , ::::::::i:::i:::i:i::,,,,,:,::i:i:::iir;@Secbone.ii:::
 *
 * 四全向轮底盘，俯视时 ID1~ID4 逆时针排列。
 * 底盘中心指向 ID1 的方向临时定义为车头 (+X)，+Y 向左，+WZ 为逆时针。
 * 四个电机同时给正电流时，底盘向 +WZ 方向旋转。
 */

Chassis_Ctrl Chassis;

// VOFA+ JustFloat: four little-endian float32 values followed by 00 00 80 7F.
// Keep the buffer alive until the UART10 transmit interrupt completes.
static void Send_Chassis_Power_VOFA(void)
{
	static uint8_t frame[4U * 4U + 4U];
	static uint32_t last_tick = 0U;
	const uint32_t now = xTaskGetTickCount();
	if ((now - last_tick) < pdMS_TO_TICKS(CHASSIS_POWER_VOFA_PERIOD_MS)
		|| huart10.gState != HAL_UART_STATE_READY)
	{
		return;
	}
	last_tick = now;

	const bool feedback_ready = Message.PowerBoardFeedbackReady();
	const float channels[4] = {
		feedback_ready ? Message.PowerBoardR.power_out : 0.0f,
		feedback_ready ? Message.PowerBoardR.power : 0.0f,
		feedback_ready ? (float)Message.PowerBoardR.power_limit : 0.0f,
		feedback_ready ? 1.0f : 0.0f
	};
	memcpy(frame, channels, sizeof(channels));
	frame[16] = 0x00;
	frame[17] = 0x00;
	frame[18] = 0x80;
	frame[19] = 0x7f;
	(void)HAL_UART_Transmit_IT(&huart10, frame, sizeof(frame));
}

void Chassis_Task(void *argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	Chassis.Chassis_Init();

	/* Infinite loop */
	for (;;)
	{
		//状态机切换
		Chassis.Behaviour_Mode();
		//外设数据反馈更新
		Chassis.Feedback_Update();
		//速度矢量参考系转换，获得底盘中心目标速度
		Chassis.Control();
		//逆解算与PID计算
		Chassis.Control_loop();
		


		if(Chassis.Mode == CHASSIS_NO_MOVE)
		{
			CAN_Cmd.SendData(&CAN_Cmd.Chassis, 0, 0, 0, 0);
		}
		else
		{
			CAN_Cmd.SendData(&CAN_Cmd.Chassis,
				Chassis.Motor[0].give_current,
				Chassis.Motor[1].give_current,
				Chassis.Motor[2].give_current,
				Chassis.Motor[3].give_current);
		}
		Send_Chassis_Power_VOFA();

		
		
		Chassis.Chassis_Task_DWT_dt = DWT_GetDeltaT(&Chassis.Chassis_Task_DWT_Count);

		xQueueSend(Message_Queue, &ID_Data[ChassisData], 0);
		Chassis.Statistic_Update(xTaskGetTickCount());

		osDelay(CHASSIS_CONTROL_TIME_MS);
	}
	/* USER CODE END StartDefaultTask */
}

// 底盘初始化
void Chassis_Ctrl::Chassis_Init(void)
{
	RC_Ptr = get_remote_control_point();
	Mode = CHASSIS_NO_MOVE;
	KeyboardLittleTop = false;
	KeyboardNoForce = false;
	LastCPressed = false;
	LastEPressed = false;

	Chassis_Task_DWT_dt = 0;
	Chassis_Task_DWT_Count = 0;
	
	Feed_Back_Count = 0;
	Feed_Back_dt = 0;


	{
		Motor[0].chassis_motor_measure = CAN_Cmd.Chassis.Get_Motor_Measure_Pointer(0);
		Motor[1].chassis_motor_measure = CAN_Cmd.Chassis.Get_Motor_Measure_Pointer(1);
		Motor[2].chassis_motor_measure = CAN_Cmd.Chassis.Get_Motor_Measure_Pointer(2);
		Motor[3].chassis_motor_measure = CAN_Cmd.Chassis.Get_Motor_Measure_Pointer(3);

		for (uint8_t i = 0; i < 4; ++i)
		{
			PID.Init(&Motor_Speed_Pid[i], POSITION,
				CHASSIS_3508_SPEED_PID_KP,
				CHASSIS_3508_SPEED_PID_KI,
				CHASSIS_3508_SPEED_PID_KD,
				CHASSIS_3508_SPEED_PID_MAX_OUT,
				CHASSIS_3508_SPEED_PID_MAX_IOUT,
				CHASSIS_3508_SPEED_PID_BAND_I);
		}
	}
	
	// 最大 最小速度
	Velocity.vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
	Velocity.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	Velocity.vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
	Velocity.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	

	
	
	Feedback_Update();
}

// 数据更新
void Chassis_Ctrl::Feedback_Update(void)
{
	uint8_t i = 0;
	Velocity.Speed = 0;
	
	for(i = 0; i < 4; i++)
	{
		// 3508 反馈为转子 RPM，按 19:1 减速比和 0.075m 轮半径换算为轮缘线速度。
		Motor[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Motor[i].chassis_motor_measure->speed_rpm;
		Velocity.Speed += abs(Motor[i].speed);
	}
	
	Velocity.Speed /= 4;
	
	// 四全向轮正运动学，数组 0~3 分别对应 ID1~ID4。
	Velocity.vx = (-Motor[1].speed + Motor[3].speed) * OMNI_TRANSLATION_FEEDBACK_SCALE;
	Velocity.vy = ( Motor[0].speed - Motor[2].speed) * OMNI_TRANSLATION_FEEDBACK_SCALE;
	Velocity.wz = ( Motor[0].speed + Motor[1].speed + Motor[2].speed + Motor[3].speed)
		* OMNI_ROTATION_FEEDBACK_SCALE / MOTOR_DISTANCE_TO_CENTER;

	IMU_Data.AngleX = -Message.MPU_DataXY.AngleX.int_16 * FP32_MPU_RAD * ANGLE_TO_RAD;
	IMU_Data.AngleY = Message.MPU_DataXY.AngleY.int_16 * FP32_MPU_RAD * ANGLE_TO_RAD;
	IMU_Data.AngleZ = Message.MPU_DataZ.AngleZ.int_16 * FP32_MPU_RAD * ANGLE_TO_RAD;
	IMU_Data.Speed_X = -Message.MPU_DataXY.Speed_X.int_16 * BMI088_GYRO_2000_SEN;
	IMU_Data.Speed_Y = Message.MPU_DataXY.Speed_Y.int_16 * BMI088_GYRO_2000_SEN;
	IMU_Data.Speed_Z = Message.MPU_DataZ.Speed_Z.int_16 * BMI088_GYRO_2000_SEN;
	IMU_Data.Acce_X = (float)Message.MPU_DataZ.Acce_X.int_16 / 1000.0f;
	IMU_Data.Acce_Z = (float)Message.MPU_DataZ.Acce_Z.int_16 / 1000.0f;
	
	Feed_Back_dt = DWT_GetDeltaT(&Feed_Back_Count);
}

// 底盘行为状态设置
void Chassis_Ctrl::Behaviour_Mode(void)
{
	// 双拨杆向上进入键鼠模式；C 切换小陀螺，E 切换整车无力。
	const bool rc_error = RC_data_is_error(RC_Ptr);
	const bool keyboard_mode = !rc_error
		&& switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT])
		&& switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]);
	const bool c_pressed = (RC_Ptr->key.v & KEY_PRESSED_OFFSET_C) != 0;
	const bool e_pressed = (RC_Ptr->key.v & KEY_PRESSED_OFFSET_E) != 0;
	if (!keyboard_mode)
	{
		KeyboardLittleTop = false;
		KeyboardNoForce = false;
	}
	else
	{
		const bool e_toggle = e_pressed && !LastEPressed;
		if (e_toggle)
		{
			KeyboardNoForce = !KeyboardNoForce;
			if (KeyboardNoForce)
			{
				KeyboardLittleTop = false;
			}
		}
		if (c_pressed && !LastCPressed && !KeyboardNoForce && !e_toggle)
		{
			KeyboardLittleTop = !KeyboardLittleTop;
		}
	}
	LastCPressed = c_pressed;
	LastEPressed = e_pressed;

	// 实体无力拨杆和键盘无力均优先于运动模式。
	if (rc_error || switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]) || KeyboardNoForce)
	{
		Mode = CHASSIS_NO_MOVE;
	}
	else if (switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT])
		&& (!switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]) || KeyboardLittleTop))
	{
		Mode = CHASSIS_LITTLE_TOP;
	}
	else
	{
		Mode = CHASSIS_NORMAL_MODE;
	}
	Flag_Behaviour_Control();
}

void Chassis_Ctrl::Flag_Behaviour_Control()
{
	if (Mode == CHASSIS_NO_MOVE)
	{
		for (uint8_t i = 0; i < 4; ++i)
		{
			PID.Clear(&Motor_Speed_Pid[i]);
		}
	}
}

// 遥控器的数据处理成底盘的前进vx速度，vy速度
void Chassis_Ctrl::RC_to_Control(fp32 *vx_set, fp32 *vy_set)
{

	//当两边拨杆同时在最上方的话，进入由键盘控制的普通模式

	if (switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
	{
		*vx_set = 0.0f;
		*vy_set = 0.0f;

		// 按实车方向修正 WASD 输入：W 前、S 后、A 左、D 右。
		if ((RC_Ptr->key.v & KEY_PRESSED_OFFSET_W) && !(RC_Ptr->key.v & KEY_PRESSED_OFFSET_S))
		{
			*vx_set = -CHASSIS_RC_MAX_VX_MPS;
		}
		else if ((RC_Ptr->key.v & KEY_PRESSED_OFFSET_S) && !(RC_Ptr->key.v & KEY_PRESSED_OFFSET_W))
		{
			*vx_set = CHASSIS_RC_MAX_VX_MPS;
		}

		if ((RC_Ptr->key.v & KEY_PRESSED_OFFSET_A) && !(RC_Ptr->key.v & KEY_PRESSED_OFFSET_D))
		{
			*vy_set = -CHASSIS_RC_MAX_VY_MPS;
		}
		else if ((RC_Ptr->key.v & KEY_PRESSED_OFFSET_D) && !(RC_Ptr->key.v & KEY_PRESSED_OFFSET_A))
		{
			*vy_set = CHASSIS_RC_MAX_VY_MPS;
		}
		return;
	}

	int16_t vx_channel;
	int16_t vy_channel;
	rc_deadline_limit(RC_Ptr->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
	rc_deadline_limit(RC_Ptr->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

	const fp32 vx_set_channel = vx_channel * CHASSIS_RC_MAX_VX_MPS / 660.0f;
	const fp32 vy_set_channel = vy_channel * CHASSIS_RC_MAX_VY_MPS / 660.0f;

	// DR16 左摇杆：ch3 向前推时的原始符号与底盘 +X 相反，因此前后输入在这里取反。
	// ch2 直接作为云台坐标系的左右平移量
	*vx_set = -vx_set_channel;
	*vy_set = vy_set_channel;
}

void Chassis_Ctrl::Behaviour_Control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set)
{
	if (Mode == CHASSIS_NO_MOVE) // 无力
	{
		*vx_set = 0.0f;
		*vy_set = 0.0f;
		*angle_set = 0.0f;
	}
	else if (Mode == CHASSIS_NORMAL_MODE)
	{
		RC_to_Control(vx_set, vy_set);
		*angle_set = 0.0f;
	}
	else if (Mode == CHASSIS_LITTLE_TOP)
	{
		const fp32 turn_weight = 1.0f
			- (abs(RC_Ptr->rc.ch[CHASSIS_X_CHANNEL]) + abs(RC_Ptr->rc.ch[CHASSIS_Y_CHANNEL])) / 1320.0f;//shit_001
		RC_to_Control(vx_set, vy_set);
		*angle_set = CHASSIS_LITTLE_TOP_MAX_WZ_RADPS * turn_weight;
	}
}

// 控制
void Chassis_Ctrl::Control(void)
{
	fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	Behaviour_Control(&vx_set, &vy_set, &angle_set);
	if (Mode == CHASSIS_NO_MOVE || !Gimbal.Initialized)
	{
		Velocity.vx_set = 0.0f;
		Velocity.vy_set = 0.0f;
		Velocity.wz_set = 0.0f;
		return;
	}

	// 遥控器的平移指令属于云台坐标系，转换到底盘 ID1 径向为 +X 的坐标系。
	const fp32 gimbal_yaw = Gimbal.GetRelativeYawRad();
	const fp32 sin_yaw = arm_sin_f32(gimbal_yaw);
	const fp32 cos_yaw = arm_cos_f32(gimbal_yaw);
	const fp32 vx_body = cos_yaw * vx_set - sin_yaw * vy_set;
	const fp32 vy_body = sin_yaw * vx_set + cos_yaw * vy_set;

	Velocity.vx_set = vx_body;
	Velocity.vy_set = vy_body;
	Velocity.wz_set = angle_set;
	if (Mode == CHASSIS_LITTLE_TOP)
	{
		Velocity.vx_set = fp32_constrain(Velocity.vx_set, Velocity.vx_min_speed, Velocity.vx_max_speed);
		Velocity.vy_set = fp32_constrain(Velocity.vy_set, Velocity.vy_min_speed, Velocity.vy_max_speed);
	}
}

// 四全向轮逆运动学。每个轮子正速度的驱动方向均为当地逆时针切向。
void Chassis_Ctrl::Vector_to_Wheel_Speed(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{

	fp32 vx_temp = *vx_set;
	fp32 vy_temp = *vy_set;
	fp32 wz_temp = *wz_set;

	Motor[0].speed_set =  vy_temp + MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Motor[1].speed_set = -vx_temp + MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Motor[2].speed_set = -vy_temp + MOTOR_DISTANCE_TO_CENTER * wz_temp;
	Motor[3].speed_set =  vx_temp + MOTOR_DISTANCE_TO_CENTER * wz_temp;

	fp32 max_speed = 0.0f;
	for (uint8_t i = 0; i < 4; ++i)
	{
		const fp32 wheel_speed = fabsf(Motor[i].speed_set);
		if (wheel_speed > max_speed)
		{
			max_speed = wheel_speed;
		}
	}
	if (max_speed > MAX_WHEEL_SPEED)
	{
		const fp32 scale = MAX_WHEEL_SPEED / max_speed;
		for (uint8_t i = 0; i < 4; ++i)
		{
			Motor[i].speed_set *= scale;
		}
	}

}

// 底盘控制计算
void Chassis_Ctrl::Control_loop(void)
{
	uint8_t i = 0;

	Vector_to_Wheel_Speed(&Velocity.vx_set, &Velocity.vy_set, &Velocity.wz_set);

	// 3508 轮速 PID，输出为 C620 电流指令。
	for(i = 0; i < 4; i++)
	{
		PID.Calc(&Motor_Speed_Pid[i], Motor[i].speed, Motor[i].speed_set);
		Motor[i].give_current = (int16_t)fp32_constrain(
			Motor_Speed_Pid[i].out,
		   -CHASSIS_3508_SPEED_PID_MAX_OUT,
		    CHASSIS_3508_SPEED_PID_MAX_OUT);
	}
}

Chassis_Ctrl *get_chassis_ctrl_pointer(void)
{
	return &Chassis;
}
