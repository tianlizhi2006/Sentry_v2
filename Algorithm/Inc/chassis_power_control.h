#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H

#include "arm_math.h"
#include "algorithm_pid.h"

#define TOQUE_COEFFICIENT 1.99688994e-6f
#define POWER_K1          0.65f
#define POWER_K2          0.004f
#define POWER_CONSTANT    1.55f
#define CAP_POWER_OPEN    100.0f
#define CAP_POWER_CLOSE   10.0f

typedef struct
{
	float K1;
	float K2;
	float constant;
	float Cap_power_open;
	float Cap_power_close;
} Chassis_Power_K;

typedef struct
{
	float power_buffer_set;
	float power_buffer_out;
	uint16_t grade_power_limit;
	float Max_input_power;
	float Chassis_Max_power;
} Chassis_Power_limit;

typedef struct
{
	float chassis_speed_radps[4];
	float chassis_torque_Nm[4];
	float initial_total_power;
	float initial_give_power[4];
	float scaled_give_power[4];
	float scaled_total_power;
} Chassis_Power_calc;

class PowerClass
{
public:
	Chassis_Power_K Power_K;
	Chassis_Power_limit Power_limit;
	Chassis_Power_calc Power_calc;
	float TN_K;
	float power_buffer_set;
	bool cap_state;
	uint8_t cap_lack;

	PowerClass();
	void Power_Feedback_Update(void);
	void Power_Calc(void);
};

#endif
