#ifndef PID_H
#define PID_H

#include "dev_system.h"
#include "algorithm_user_lib.h"


#ifdef __cplusplus
extern "C" {
#endif

	
#ifdef __cplusplus
}
#endif



//#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

#define LimitBand(input, max, min)   \
    {                                \
        if (input > max)             \
        {                            \
            input = max;             \
        }                            \
        else if (input < min)        \
        {                            \
            input = min;             \
        }                            \
    }

/******************************* PID CONTROL *********************************/
typedef enum pid_Improvement_e
{
    none= 0X00,                        //0000 0000
    Integral_Limit = 0x01,              //0000 0001
    Derivative_On_Measurement = 0x02,   //0000 0010
    Trapezoid_Intergral = 0x04,         //0000 0100
    Proportional_On_Measurement = 0x08, //0000 1000
    OutputFilter = 0x10,                //0001 0000
    ChangingIntegrationRate = 0x20,     //0010 0000
    DerivativeFilter = 0x40,            //0100 0000
    ErrorHandle = 0x80,                 //1000 0000
} PID_Improvement_e;										  
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef __packed struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;
		



typedef enum
{
    INIT = 0x00,
    POSITION,//位置式
    DELTA//增量式
}PidMode;

class sPidTypeDef
{
public:
    sPidTypeDef(PidMode mode_, fp32 Kp_, fp32 Ki_, fp32 Kd_, fp32 max_out_ = 30000, fp32 max_Iout_ = 3000, fp32 band_I_ = 3000)
        : mode(mode_), Kp(Kp_), Ki(Ki_), Kd(Kd_), max_out(max_out_), max_Iout(max_Iout_), band_I(band_I_)
    {
    }
    sPidTypeDef() {};
    PidMode mode;

    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  // 最大输出
    fp32 max_Iout; // 最大积分输出
    fp32 band_I;

    fp32 set;
    fp32 ref;

    fp32 out;
    fp32 out_single;
    fp32 P_out;
    fp32 I_out;
    fp32 D_out;

    fp32 last_in;
    fp32 T; // ms

    fp32 Dbuf[3];  // 微分项 0最新 1上一次 2上上次
    fp32 error[3]; // 误差项 0最新 1上一次 2上上次

    float Forwardfeed(float i);

    ~sPidTypeDef() {}
};
class PID_Ctrl
{
public:
    void Init(sPidTypeDef *pid, PidMode mode, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out, fp32 max_Iout, fp32 band_I);
    fp32 Calc(sPidTypeDef *pid, fp32 ref, fp32 set);
    void Clear(sPidTypeDef *pid);
};

typedef __packed struct pid_t
{
    float Kp;
    float Ki;
    float Kd;
    float Fd_K;

    float Ref;
    float Measure;
    float Err;

    float Pout;
    float Iout;
    float Dout;
    float Forwardfeed_out;
    float ITerm;

    float Output;

    float Last_Measure;
    float Last_Err;
    float Last_ITerm;
    float Last_Output;
    float Last_Dout;
    float Last_in;
    float Last_Ref;
    float Last_Forwardfeed_out;
    float Ref_err;

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ControlPeriod;
    float CoefA;         // For Changing Integral
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;

    uint16_t OLS_Order;
    Ordinary_Least_Squares_t OLS;

    uint32_t DWT_CNT;
    float dt;

    PID_ErrorHandler_t ERRORHandler;

    uint8_t Improve;

    void (*User_Func1_f)(struct pid_t *pid);
    void (*User_Func2_f)(struct pid_t *pid);
} PID_t;

class PID_T
{
public:
    void Init(PID_t *pid, float max_out, float intergral_limit, float deadband, float kp, float Ki, float Kd, float A, float B, float output_lpf_rc, float derivative_lpf_rc, uint16_t ols_order, uint8_t improve);
    float Calc(PID_t *pid, float measure, float ref);
    /*自己加的*/
    static float Forwardfeed(PID_t *pid);
    void Clear(PID_t *pid);
};

static void f_Trapezoid_Intergral(PID_t *pid);
static void f_Changing_Integration_Rate(PID_t *pid);
static void f_Integral_Limit(PID_t *pid);
static void f_Derivative_On_Measurement(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Derivative_Filter(PID_t *pid);
static void f_Output_Limit(PID_t *pid);
static void f_Proportion_Limit(PID_t *pid);
static void f_PID_ErrorHandle(PID_t *pid);

extern PID_T    PID_CT;
extern PID_Ctrl PID;

#endif

