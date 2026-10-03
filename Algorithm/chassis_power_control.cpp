#include "chassis_power_control.h"
#include "app_preference.h"
#include "tasks.h"

PowerClass::PowerClass()
{
	Power_K.K1 = POWER_K1;
	Power_K.K2 = POWER_K2;
	Power_K.constant = POWER_CONSTANT;
	Power_K.Cap_power_open = CAP_POWER_OPEN;
	Power_K.Cap_power_close = CAP_POWER_CLOSE;
	Power_limit.Chassis_Max_power = CHASSIS_POWER_LIMIT_W;
	TN_K = TOQUE_COEFFICIENT;
	cap_state = false;
	cap_lack = 1;
	power_buffer_set = 60;
}

void PowerClass::Power_Feedback_Update()
{
	Power_limit.Max_input_power = CHASSIS_POWER_LIMIT_W;
	if (Power_limit.Max_input_power <= 90.0f)
	{
		Power_limit.Max_input_power -= 5.0f;
	}

	for (uint8_t i = 0; i < 4; ++i)
	{
		Power_calc.chassis_speed_radps[i] = Chassis.Motor[i].speed / OMNI_WHEEL_RADIUS;
	}
}

void PowerClass::Power_Calc()
{
	if (Message.SuperCapR.energy > 6)
	{
		if (Message.SuperCapR.energy > 10)
		{
			cap_lack = 0;
		}
		Power_limit.Chassis_Max_power = (cap_state && !cap_lack)
			? Power_limit.Max_input_power + Power_K.Cap_power_open
			: Power_limit.Max_input_power;
	}
	else
	{
		cap_lack = 1;
		Power_limit.Chassis_Max_power = Power_limit.Max_input_power;
	}

	Power_calc.initial_total_power = 0.0f;
	for (uint8_t i = 0; i < 4; ++i)
	{
		float torque = Power_calc.chassis_torque_Nm[i];
		float speed = Power_calc.chassis_speed_radps[i];
		Power_calc.initial_give_power[i] = torque * speed
			+ Power_K.K1 * torque * torque
			+ Power_K.K2 * speed * speed
			+ POWER_CONSTANT;
		if (Power_calc.initial_give_power[i] < 0.0f)
		{
			Power_calc.initial_give_power[i] = 0.0f;
		}
		Power_calc.initial_total_power += Power_calc.initial_give_power[i];
	}

	if (Power_calc.initial_total_power <= Power_limit.Chassis_Max_power)
	{
		return;
	}

	float power_scale = Power_limit.Chassis_Max_power / Power_calc.initial_total_power;
	for (uint8_t i = 0; i < 4; ++i)
	{
		if (Power_calc.initial_give_power[i] == 0.0f)
		{
			continue;
		}

		Power_calc.scaled_give_power[i] = Power_calc.initial_give_power[i] * power_scale;
		float a = Power_K.K1;
		float b = Power_calc.chassis_speed_radps[i];
		float c = Power_K.K2 * b * b - Power_calc.scaled_give_power[i] + POWER_CONSTANT;
		float discriminant = b * b - 4.0f * a * c;
		if (discriminant < 0.0f)
		{
			continue;
		}

		float root = sqrtf(discriminant);
		Power_calc.chassis_torque_Nm[i] = Power_calc.chassis_torque_Nm[i] >= 0.0f
			? (-b + root) / (2.0f * a)
			: (-b - root) / (2.0f * a);
	}
}
