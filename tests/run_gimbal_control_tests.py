"""Host regression tests for the normal dual-yaw controller.

Compiles actual Gimbal_Task.cpp and algorithm_pid.cpp with hardware/RTOS stubs.
Covers RC and mouse control, feedback loss, recenter speed limits,
soft-limit hysteresis, F-key switching, and launcher interlocks.
This does not validate physical motor stability.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

PLATFORM = r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include "app_preference.h"
#define __packed
typedef float fp32;
#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f
#define ECD_TO_DEG 0.0439453125f
#define DM_4310_T_MIN (-10.0f)
#define DM_4310_T_MAX 10.0f
#define DM_4310_V_MIN (-30.0f)
#define DM_4310_V_MAX 30.0f
extern uint32_t host_tick;
inline uint32_t xTaskGetTickCount() { return host_tick; }
inline void osDelay(uint32_t ms) { host_tick += ms; }
inline void vTaskDelayUntil(uint32_t *last, uint32_t ms) { *last += ms; host_tick=*last; }
inline float DWT_GetDeltaT(uint32_t *) { return 0.001f; }
class Statistic {};
class ValidData {};
struct RC_ctrl_t {
    struct { int16_t ch[5]; uint8_t s[2]; } rc;
    struct { int16_t x, y; uint8_t press_l, press_r; } mouse;
    struct { uint16_t v; } key;
};
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
extern RC_ctrl_t host_rc;
extern bool host_rc_bad;
extern int16_t host_mouse_pending_x, host_mouse_pending_y;
inline void RC_take_mouse_delta(int16_t *dx, int16_t *dy) {
    *dx=host_mouse_pending_x; *dy=host_mouse_pending_y;
    host_mouse_pending_x=0; host_mouse_pending_y=0;
}
inline RC_ctrl_t *get_remote_control_point() { return &host_rc; }
inline bool RC_data_is_error(const RC_ctrl_t *) { return host_rc_bad; }
inline bool switch_is_down(uint8_t s) { return s==2; }
inline bool switch_is_mid(uint8_t s) { return s==3; }
inline bool switch_is_up(uint8_t s) { return s==1; }
struct DM_Motor_measure_t {
    struct { float fdata; } POS, VEl, Torque;
    uint8_t state;
};
struct motor_measure_t { uint16_t ecd; int16_t speed_rpm; };
struct HostDM {
    DM_Motor_measure_t data = {};
    float sent = 0;
    bool enable_succeeds = true;
    bool clear_pending = false;
    unsigned clear_count = 0, enable_count = 0, ordered_enable_count = 0;
    const DM_Motor_measure_t *Get_DM_Motor_Measure_Pointer() { return &data; }
};
struct HostDJI {
    motor_measure_t data[4] = {};
    int16_t sent[4] = {};
    const motor_measure_t *Get_Motor_Measure_Pointer(int i) { return &data[i]; }
};
struct HostCAN {
    HostDM GimbalLargeYaw, GimbalPitch;
    HostDJI GimbalSmallYaw, Fric, Trigger;
    void DM_Motor_clear_error(HostDM *m) { ++m->clear_count; m->clear_pending=true; }
    void DM_Motor_Enable(HostDM *m) {
        ++m->enable_count;
        if(m->clear_pending) ++m->ordered_enable_count;
        m->clear_pending=false;
        if(m->enable_succeeds) m->data.state=1;
    }
    void DM_SendData(HostDM *m, float, float, float, float, float t) { m->sent=t; }
    void SendData(HostDJI *m, int16_t a, int16_t b=0) { m->sent[0]=a; m->sent[1]=b; }
};
extern HostCAN CAN_Cmd;
struct HostMessage {
    struct { float Yaw_angle, Yaw_speed, Pitch_angle, Pitch_speed; } GimbalGyro = {};
    bool all_ready=true;
    bool GimbalFeedbackReady() const { return all_ready; }
    bool FrictionFeedbackReady() const { return true; }
    bool TriggerFeedbackReady() const { return true; }
};
extern HostMessage Message;
struct HostChassis { struct { float wz_set; } Velocity; bool KeyboardNoForce=false; };
extern HostChassis Chassis;

'''

TEST = r'''
#include "Gimbal_Task.h"
#include <stdio.h>
#include <string.h>

uint32_t host_tick=0;
RC_ctrl_t host_rc={};
bool host_rc_bad=false;
int16_t host_mouse_pending_x=0, host_mouse_pending_y=0;
HostCAN CAN_Cmd;
HostMessage Message;
HostChassis Chassis={};
static unsigned checks=0, cycles=0;
static FILE *replay=NULL;
#define CHECK(x) do { ++checks; if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static bool Near(float a,float b) { return fabsf(a-b)<0.00001f; }
fp32 fp32_constrain(fp32 x,fp32 lo,fp32 hi) { return x<lo?lo:(x>hi?hi:x); }
fp32 loop_fp32_constrain(fp32 x,fp32 lo,fp32 hi) {
    while(x>hi) x-=hi-lo;
    while(x<lo) x+=hi-lo;
    return x;
}
static void Cycle() {
    ++host_tick; ++cycles;
    Gimbal.Feedback_Update(); Gimbal.Behaviour_Mode(); Gimbal.Control(); Gimbal.Send();
    if(replay) {
        const float row[]={
            (float)host_tick,(float)Gimbal.Mode,(float)Gimbal.Initialized,(float)Gimbal.SmallYawLimited,
            Gimbal.RelativeYawRad,Gimbal.LargeYaw.angle,Gimbal.LargeYaw.speed,Gimbal.LargeYaw.speed_set,
            Gimbal.LargeYaw.torque_set,Gimbal.LargeYaw.position_pid.P_out,Gimbal.LargeYaw.position_pid.I_out,
            Gimbal.LargeYaw.position_pid.D_out,Gimbal.LargeYaw.position_pid.out,
            Gimbal.LargeYaw.speed_pid.P_out,Gimbal.LargeYaw.speed_pid.I_out,Gimbal.LargeYaw.speed_pid.D_out,
            Gimbal.SmallYaw.angle,Gimbal.SmallYaw.angle_set,Gimbal.SmallYaw.relative_ecd,Gimbal.SmallYaw.speed,
            (float)Gimbal.SmallYaw.give_current,Gimbal.SmallYaw.position_pid.out,
            Gimbal.SmallYaw.speed_pid.P_out,Gimbal.SmallYaw.speed_pid.I_out,Gimbal.SmallYaw.speed_pid.D_out,
            Gimbal.Pitch.angle,Gimbal.Pitch.angle_set,Gimbal.Pitch.speed,Gimbal.Pitch.torque_set,
            CAN_Cmd.GimbalLargeYaw.sent,CAN_Cmd.GimbalPitch.sent,
            (float)CAN_Cmd.GimbalSmallYaw.sent[1],(float)CAN_Cmd.Fric.sent[0],(float)CAN_Cmd.Fric.sent[1],
            (float)CAN_Cmd.Trigger.sent[0],(float)Gimbal.Flags.Fric_Ready_Flag,
            (float)Gimbal.Flags.Fric_Flag,(float)Gimbal.Flags.Shoot_Flag
        };
        CHECK(fwrite(row,sizeof(row),1,replay)==1);
    }
}
static void CheckStopped() {
    CHECK(Gimbal.Mode==GIMBAL_NO_MOVE);
    CHECK(Gimbal.LargeYaw.speed_set==0 && CAN_Cmd.GimbalLargeYaw.sent==0);
    CHECK(CAN_Cmd.GimbalSmallYaw.sent[1]==0 && CAN_Cmd.GimbalPitch.sent==0);
    CHECK(CAN_Cmd.Fric.sent[0]==0 && CAN_Cmd.Fric.sent[1]==0 && CAN_Cmd.Trigger.sent[0]==0);
}
int main(int argc,char **argv) {
    if(argc>1) { replay=fopen(argv[1],"wb"); CHECK(replay!=NULL); }
    host_rc.rc.s[0]=3; host_rc.rc.s[1]=2;
    CAN_Cmd.GimbalSmallYaw.data[1].ecd=GIMBAL_SMALL_YAW_ZERO_ECD;
    CAN_Cmd.GimbalLargeYaw.data.POS.fdata=GIMBAL_LARGE_YAW_ZERO_RAD;
    Message.GimbalGyro.Pitch_angle=10;
    Gimbal.Init(); Cycle();
    CHECK(CAN_Cmd.GimbalLargeYaw.ordered_enable_count==1);
    CHECK(CAN_Cmd.GimbalPitch.ordered_enable_count==1);
    CHECK(Near(Gimbal.LargeYaw.position_pid.max_out,DM_4310_V_MAX));
    CHECK(Gimbal.Mode==GIMBAL_REMOTE_CONTROL);
    CHECK(Near(Gimbal.LargeYaw.position_pid.Kp,GIMBAL_LARGE_YAW_POSITION_KP));
    CHECK(Near(Gimbal.LargeYaw.speed_pid.Kp,GIMBAL_LARGE_YAW_SPEED_KP));
    CHECK(Near(Gimbal.LargeYaw.speed_pid.Ki,GIMBAL_LARGE_YAW_SPEED_KI));
    CHECK(Near(Gimbal.LargeYaw.speed_pid.max_Iout,GIMBAL_LARGE_YAW_SPEED_MAX_IOUT));
    host_rc.rc.ch[0]=100; Cycle();
    CHECK(Near(Gimbal.SmallYaw.angle_set,-100.0f*GIMBAL_YAW_RC_SENSITIVITY));
    CHECK(CAN_Cmd.GimbalSmallYaw.sent[1]!=0); // IMU yaw tracking is active again.
    host_rc.rc.ch[0]=0;
    CAN_Cmd.GimbalSmallYaw.data[1].ecd=GIMBAL_SMALL_YAW_ZERO_ECD+100;
    Chassis.Velocity.wz_set=0.3f;
    Cycle(); Cycle(); // Let the position difference term settle.
    CHECK(Near(Gimbal.LargeYaw.speed_set,Gimbal.LargeYaw.position_pid.out+
        Chassis.Velocity.wz_set*GIMBAL_CHASSIS_WZ_FEEDFORWARD));
    CHECK(Near(Gimbal.LargeYaw.speed_set,-GIMBAL_LARGE_YAW_POSITION_KP*100.0f*ECD_TO_DEG-0.3f));
    // Choose an offset demanding at least 3 rad/s at the configured gain, so
    // this tests the limiter rather than depending on one particular PID gain.
    const int cap_test_ecd=(int)ceilf(3.0f/(GIMBAL_LARGE_YAW_POSITION_KP*ECD_TO_DEG));
    CHECK(cap_test_ecd>0 && cap_test_ecd<4096);
    Chassis.Velocity.wz_set=0;
    CAN_Cmd.GimbalSmallYaw.data[1].ecd=(uint16_t)((GIMBAL_SMALL_YAW_ZERO_ECD+cap_test_ecd)%8192);
    Cycle(); Cycle();
    CHECK(Gimbal.LargeYaw.speed_set < -2.5f);
    CHECK(Near(Gimbal.LargeYaw.speed_set,-GIMBAL_LARGE_YAW_POSITION_KP*cap_test_ecd*ECD_TO_DEG));

    CHECK(fabsf(CAN_Cmd.GimbalLargeYaw.sent)<=GIMBAL_LARGE_YAW_SPEED_MAX_OUT);
    CAN_Cmd.GimbalSmallYaw.data[1].ecd=(uint16_t)(((int)GIMBAL_SMALL_YAW_ZERO_ECD-cap_test_ecd+8192)%8192);
    Cycle(); Cycle();
    CHECK(Gimbal.LargeYaw.speed_set > 2.5f);
    CHECK(Near(Gimbal.LargeYaw.speed_set,GIMBAL_LARGE_YAW_POSITION_KP*cap_test_ecd*ECD_TO_DEG));

    CHECK(fabsf(CAN_Cmd.GimbalLargeYaw.sent)<=GIMBAL_LARGE_YAW_SPEED_MAX_OUT);
    // Extreme feedforward must still respect the final DM speed range.
    CAN_Cmd.GimbalSmallYaw.data[1].ecd=GIMBAL_SMALL_YAW_ZERO_ECD;
    Chassis.Velocity.wz_set=100; Cycle(); Cycle();
    CHECK(Near(Gimbal.LargeYaw.speed_set,DM_4310_V_MIN));
    Chassis.Velocity.wz_set=-100; Cycle(); Cycle();
    CHECK(Near(Gimbal.LargeYaw.speed_set,DM_4310_V_MAX));
    CHECK(fabsf(CAN_Cmd.GimbalLargeYaw.sent)<=GIMBAL_LARGE_YAW_SPEED_MAX_OUT);
    CAN_Cmd.GimbalSmallYaw.data[1].ecd=GIMBAL_SMALL_YAW_ZERO_ECD+100;
    Chassis.Velocity.wz_set=0.3f; Cycle(); Cycle();
    // The small-yaw target must remain frozen throughout a latched soft limit,
    // and resume from that target (not jump to the measured IMU angle) on release.
    const int limit_ecd=(int)GIMBAL_SMALL_YAW_LIMIT_ECD;
    const int release_ecd=(int)GIMBAL_SMALL_YAW_RELEASE_ECD;
    const int directions[]={1,-1};
    for(unsigned n=0;n<2;++n) {
        const int direction=directions[n];
        host_rc.rc.s[0]=2; host_rc.rc.ch[0]=0;
        CAN_Cmd.GimbalSmallYaw.data[1].ecd=GIMBAL_SMALL_YAW_ZERO_ECD;
        Message.GimbalGyro.Yaw_angle=10.0f*direction;
        Cycle(); host_rc.rc.s[0]=3; Cycle();
        host_rc.rc.ch[0]=(int16_t)(660*direction);
        CAN_Cmd.GimbalSmallYaw.data[1].ecd=(uint16_t)(GIMBAL_SMALL_YAW_ZERO_ECD+direction*limit_ecd);
        Cycle(); CHECK(!Gimbal.SmallYawLimited); // Strict entry threshold.
        CAN_Cmd.GimbalSmallYaw.data[1].ecd=(uint16_t)(GIMBAL_SMALL_YAW_ZERO_ECD+direction*(limit_ecd+1));
        Cycle(); CHECK(Gimbal.SmallYawLimited);
        const float held_target=Gimbal.SmallYaw.angle_set;
        CHECK(Near(held_target,Message.GimbalGyro.Yaw_angle));
        Message.GimbalGyro.Yaw_angle=held_target+2.0f*direction;
        for(int i=0;i<50;++i) Cycle();
        CHECK(Gimbal.SmallYawLimited && Gimbal.SmallYaw.angle_set==held_target);
        CAN_Cmd.GimbalSmallYaw.data[1].ecd=(uint16_t)(GIMBAL_SMALL_YAW_ZERO_ECD+direction*(limit_ecd+release_ecd)/2);
        Cycle(); CHECK(Gimbal.SmallYawLimited && Gimbal.SmallYaw.angle_set==held_target);
        CAN_Cmd.GimbalSmallYaw.data[1].ecd=(uint16_t)(GIMBAL_SMALL_YAW_ZERO_ECD+direction*release_ecd);
        Cycle(); CHECK(Gimbal.SmallYawLimited && Gimbal.SmallYaw.angle_set==held_target);
        CAN_Cmd.GimbalSmallYaw.data[1].ecd=(uint16_t)(GIMBAL_SMALL_YAW_ZERO_ECD+direction*(release_ecd-1));
        Cycle(); CHECK(!Gimbal.SmallYawLimited);
        CHECK(Near(Gimbal.SmallYaw.angle_set,held_target-660.0f*direction*GIMBAL_YAW_RC_SENSITIVITY));
    }
    host_rc.rc.s[0]=2; host_rc.rc.ch[0]=0; Message.GimbalGyro.Yaw_angle=0;
    CAN_Cmd.GimbalSmallYaw.data[1].ecd=GIMBAL_SMALL_YAW_ZERO_ECD+100;
    Cycle(); host_rc.rc.s[0]=3; Cycle(); Cycle();
    // Normal launcher switch logic is restored, while the upper right switch inhibits firing.
    host_rc.rc.s[1]=1; Cycle();
    CHECK(Gimbal.Flags.Fric_Flag && Gimbal.Flags.Shoot_Flag);
    CAN_Cmd.Fric.data[0].speed_rpm=(int16_t)FRIC_SPEED_SET_RPM;
    CAN_Cmd.Fric.data[1].speed_rpm=-(int16_t)FRIC_SPEED_SET_RPM;
    for(unsigned i=0;i<FRIC_READY_STABLE_TIME_MS;++i) Cycle();
    CHECK(Gimbal.Flags.Fric_Ready_Flag && Gimbal.Trigger.speed_set!=0);
    host_rc.rc.s[0]=1; Cycle(); CHECK(Gimbal.Mode==GIMBAL_REMOTE_CONTROL);
    CHECK(!Gimbal.Flags.Fric_Flag && !Gimbal.Flags.Shoot_Flag && Gimbal.Trigger.speed_set==0);
    host_rc.rc.s[0]=2; Cycle(); CheckStopped();
    host_rc.rc.s[1]=2;
    Message.all_ready=false; Cycle(); CheckStopped();
    Message.all_ready=true; host_rc.rc.s[0]=3; Cycle();
    // Keyboard/mouse mode consumes each received mouse delta once, including
    // when the 1 kHz control loop runs several times between input frames.
    host_rc.rc.s[0]=1; host_rc.rc.s[1]=1;
    CAN_Cmd.GimbalSmallYaw.data[1].ecd=GIMBAL_SMALL_YAW_ZERO_ECD;
    Cycle();
    const float yaw_before_mouse=Gimbal.SmallYaw.angle_set;
    const float pitch_before_mouse=Gimbal.Pitch.angle_set;
    host_mouse_pending_x=25; host_mouse_pending_y=-10;
    Cycle();
    CHECK(Near(Gimbal.SmallYaw.angle_set,
        yaw_before_mouse-25.0f*GIMBAL_MOUSE_YAW_SENSITIVITY));
    CHECK(Near(Gimbal.Pitch.angle_set,
        pitch_before_mouse-10.0f*GIMBAL_MOUSE_PITCH_SENSITIVITY));
    const float yaw_after_mouse=Gimbal.SmallYaw.angle_set;
    const float pitch_after_mouse=Gimbal.Pitch.angle_set;
    Cycle();
    CHECK(Near(Gimbal.SmallYaw.angle_set,yaw_after_mouse));
    CHECK(Near(Gimbal.Pitch.angle_set,pitch_after_mouse));
    host_mouse_pending_x=1000; host_mouse_pending_y=1000;
    Cycle();
    CHECK(Near(Gimbal.SmallYaw.angle_set,
        yaw_after_mouse-GIMBAL_MOUSE_MAX_DELTA*GIMBAL_MOUSE_YAW_SENSITIVITY));
    CHECK(Near(Gimbal.Pitch.angle_set,
        pitch_after_mouse+GIMBAL_MOUSE_MAX_DELTA*GIMBAL_MOUSE_PITCH_SENSITIVITY));

    // Left mouse alone cannot fire. F toggles on only at a new press edge;
    // releasing the switch or losing RC control clears the latch.
    host_rc.mouse.press_l=1; Cycle();
    CHECK(!Gimbal.Flags.Fric_Flag && !Gimbal.Flags.Shoot_Flag);
    host_rc.key.v=KEY_PRESSED_OFFSET_F; Cycle();
    CHECK(Gimbal.Flags.Fric_Flag && Gimbal.Flags.Shoot_Flag);
    Cycle(); CHECK(Gimbal.Flags.Fric_Flag);
    host_rc.key.v=0; Cycle(); CHECK(Gimbal.Flags.Fric_Flag);
    CAN_Cmd.Fric.data[0].speed_rpm=(int16_t)FRIC_SPEED_SET_RPM;
    CAN_Cmd.Fric.data[1].speed_rpm=-(int16_t)FRIC_SPEED_SET_RPM;
    for(unsigned i=0;i<FRIC_READY_STABLE_TIME_MS;++i) Cycle();
    CHECK(Gimbal.Flags.Fric_Ready_Flag && Gimbal.Trigger.speed_set!=0);
    host_rc.mouse.press_l=0; Cycle();
    CHECK(!Gimbal.Flags.Shoot_Flag && Gimbal.Trigger.speed_set==0);
    host_rc.key.v=KEY_PRESSED_OFFSET_F; Cycle();
    CHECK(!Gimbal.Flags.Fric_Flag && !Gimbal.Flags.Shoot_Flag);
    host_rc.key.v=0; Cycle();
    host_rc.key.v=KEY_PRESSED_OFFSET_F; Cycle();
    CHECK(Gimbal.Flags.Fric_Flag);
    host_rc.key.v=0;
    host_rc.rc.s[0]=2; Cycle(); CheckStopped();
    host_rc.rc.s[0]=1; Cycle();
    CHECK(!Gimbal.Flags.Fric_Flag && !Gimbal.Flags.Shoot_Flag);
    Chassis.KeyboardNoForce=true;
    host_rc.mouse.press_l=1; Cycle(); CheckStopped();
    Chassis.KeyboardNoForce=false;
    host_rc.mouse.press_l=0; Cycle();
    CHECK(Gimbal.Mode==GIMBAL_REMOTE_CONTROL && !Gimbal.Flags.Fric_Flag);
    // Lost enable state: retry clear-error then enable at a bounded rate.
    HostDM &large=CAN_Cmd.GimbalLargeYaw;
    HostDM &pitch=CAN_Cmd.GimbalPitch;
    large.data.state=0; pitch.data.state=0;
    large.enable_succeeds=false; pitch.enable_succeeds=false;
    const unsigned large_before=large.enable_count, pitch_before=pitch.enable_count;
    Cycle();
    CHECK(large.enable_count==large_before+1 && pitch.enable_count==pitch_before+1);
    CHECK(large.sent==0 && pitch.sent==0);
    for(unsigned i=1;i<GIMBAL_DM_ENABLE_RETRY_MS;++i) Cycle();
    CHECK(large.enable_count==large_before+1 && pitch.enable_count==pitch_before+1);
    Cycle();
    CHECK(large.enable_count==large_before+2 && pitch.enable_count==pitch_before+2);
    large.enable_succeeds=true; pitch.enable_succeeds=true;
    for(unsigned i=0;i<GIMBAL_DM_ENABLE_RETRY_MS;++i) Cycle();
    CHECK(large.data.state==1 && pitch.data.state==1);
    CHECK(large.enable_count==large.ordered_enable_count);
    CHECK(pitch.enable_count==pitch.ordered_enable_count);

    // Re-enable while Pitch is above its range: hold its current angle, then
    // accept only inward commands and never pull it back outward as it moves.
    host_rc.rc.s[0]=2; host_rc.rc.s[1]=2; host_rc.rc.ch[1]=0;
    Message.GimbalGyro.Pitch_angle=-40.0f; Cycle();
    host_rc.rc.s[0]=3; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,40.0f));
    host_rc.rc.ch[1]=660; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,40.0f));
    host_rc.rc.ch[1]=-660; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,40.0f-660.0f*GIMBAL_PITCH_RC_SENSITIVITY));
    host_rc.rc.ch[1]=0;
    Message.GimbalGyro.Pitch_angle=-39.8f; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,39.8f));

    // The same one-way behavior applies below the lower stop.
    host_rc.rc.s[0]=2; Message.GimbalGyro.Pitch_angle=30.0f; Cycle();
    host_rc.rc.s[0]=3; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,-30.0f));
    host_rc.rc.ch[1]=-660; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,-30.0f));
    host_rc.rc.ch[1]=660; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,-30.0f+660.0f*GIMBAL_PITCH_RC_SENSITIVITY));
    host_rc.rc.ch[1]=0;
    Message.GimbalGyro.Pitch_angle=29.8f; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,-29.8f));

    // Once inside the range, the normal target limits still apply.
    host_rc.rc.s[0]=2; Message.GimbalGyro.Pitch_angle=-29.99f; Cycle();
    host_rc.rc.s[0]=3; Cycle();
    host_rc.rc.ch[1]=660; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,GIMBAL_PITCH_MAX_ANGLE));
    host_rc.rc.s[0]=2; host_rc.rc.ch[1]=0;
    Message.GimbalGyro.Pitch_angle=19.99f; Cycle();
    host_rc.rc.s[0]=3; Cycle();
    host_rc.rc.ch[1]=-660; Cycle();
    CHECK(Near(Gimbal.Pitch.angle_set,GIMBAL_PITCH_MIN_ANGLE));
    if(replay) CHECK(fclose(replay)==0);
    printf("Normal gimbal: %u checks passed; %u control cycles.\n",checks,cycles);
}

'''

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--cxx", default=shutil.which("g++"))
parser.add_argument("--record-replay", type=Path)
args = parser.parse_args()
if not args.cxx:
    parser.error("Supply --cxx with a host C++ compiler.")
project = Path(__file__).resolve().parent.parent
with tempfile.TemporaryDirectory(prefix="shaobing_gimbal_control_") as folder:
    work = Path(folder)
    (work / "host_platform.h").write_text(PLATFORM, encoding="utf-8")
    for name in ("cmsis_os2.h", "app_motor.h", "drivers_statistic.h", "protocol_dbus.h",
                 "tasks.h", "arm_math.h", "dev_system.h", "bsp_dwt.h"):
        (work / name).write_text('#include "host_platform.h"\n', encoding="utf-8")
    (work / "test.cpp").write_text(TEST, encoding="utf-8")
    binary = work / "gimbal_control.exe"
    cmd = [args.cxx, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
           "-Wno-unknown-pragmas", "-Wno-unused-function"]
    cmd += ["-I" + str(p) for p in (work, project / "Task/Inc", project / "App/Inc", project / "Algorithm/Inc")]
    cmd += [str(project / "Task/Gimbal_Task.cpp"), str(project / "Algorithm/algorithm_pid.cpp")]
    cmd += [str(work / "test.cpp"), "-o", str(binary)]
    subprocess.run(cmd, check=True)
    run = [str(binary)]
    if args.record_replay:
        run.append(str(args.record_replay.resolve()))
    subprocess.run(run, check=True)
