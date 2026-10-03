#ifndef __PROTOCOL_DBUS_H
#define __PROTOCOL_DBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "drivers_dma.h"
	
#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u
#define CUSTOM_KEYMOUSE_FRAME_LENGTH 27u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/** @name           rc_deadline_limit(input, output, dealine) 
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
  */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

	
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Custom Key-Mouse Bitmap (128-bit) ----------------- */
#define KM_BIT_W                      22u
#define KM_BIT_S                      18u
#define KM_BIT_A                       0u
#define KM_BIT_D                       3u
#define KM_BIT_SHIFT_L                26u
#define KM_BIT_SHIFT_R                27u
#define KM_BIT_CTRL_L                 28u
#define KM_BIT_CTRL_R                 29u
#define KM_BIT_Q                      16u
#define KM_BIT_E                       4u
#define KM_BIT_R                      17u
#define KM_BIT_F                       5u
#define KM_BIT_G                       6u
#define KM_BIT_Z                      25u
#define KM_BIT_X                      23u
#define KM_BIT_C                       2u
#define KM_BIT_V                      21u
#define KM_BIT_B                       1u
#define KM_BIT_SPACE                  53u
#define KM_BIT_ENTER                  54u
#define KM_BIT_BACKSPACE              55u
#define KM_BIT_TAB                    56u
#define KM_BIT_ESC                    57u
#define KM_BIT_CAPSLOCK               58u
#define KM_BIT_NUMLOCK                59u
#define KM_BIT_LALT                   60u
#define KM_BIT_RALT                   61u
#define KM_BIT_LWIN                   30u
#define KM_BIT_RWIN                   62u
#define KM_BIT_MENU                   31u
#define KM_BIT_UP                    112u
#define KM_BIT_DOWN                  113u
#define KM_BIT_LEFT                  114u
#define KM_BIT_RIGHT                 115u
#define KM_BIT_INSERT                116u
#define KM_BIT_DELETE                117u
#define KM_BIT_HOME                  118u
#define KM_BIT_END                   119u
#define KM_BIT_PAGEUP                120u
#define KM_BIT_PAGEDOWN              121u
#define KM_BIT_NUMPAD0                96u
#define KM_BIT_NUMPAD1                97u
#define KM_BIT_NUMPAD2                98u
#define KM_BIT_NUMPAD3                99u
#define KM_BIT_NUMPAD4               100u
#define KM_BIT_NUMPAD5               101u
#define KM_BIT_NUMPAD6               102u
#define KM_BIT_NUMPAD7               103u
#define KM_BIT_NUMPAD8               104u
#define KM_BIT_NUMPAD9               105u
#define KM_BIT_NUMPAD_MUL            106u
#define KM_BIT_NUMPAD_ADD            107u
#define KM_BIT_NUMPAD_SUB            108u
#define KM_BIT_NUMPAD_DIV            109u
#define KM_BIT_NUMPAD_DOT            110u
#define KM_BIT_NUMPAD_ENTER          111u
#define KM_BIT_F1                     64u
#define KM_BIT_F2                     65u
#define KM_BIT_F3                     66u
#define KM_BIT_F4                     67u
#define KM_BIT_F5                     68u
#define KM_BIT_F6                     69u
#define KM_BIT_F7                     70u
#define KM_BIT_F8                     71u
#define KM_BIT_F9                     72u
#define KM_BIT_F10                    73u
#define KM_BIT_F11                    74u
#define KM_BIT_F12                    75u
#define KM_BIT_GRAVE                  44u
#define KM_BIT_MINUS                  42u
#define KM_BIT_EQUAL                  43u
#define KM_BIT_LBRACKET               45u
#define KM_BIT_RBRACKET               46u
#define KM_BIT_BACKSLASH              47u
#define KM_BIT_SEMICOLON              48u
#define KM_BIT_QUOTE                  49u
#define KM_BIT_COMMA                  50u
#define KM_BIT_DOT                    51u
#define KM_BIT_SLASH                  52u
#define KM_BIT_0                      41u
#define KM_BIT_1                      33u
#define KM_BIT_2                      34u
#define KM_BIT_3                      35u
#define KM_BIT_4                      36u
#define KM_BIT_5                      37u
#define KM_BIT_6                      38u
#define KM_BIT_7                      39u
#define KM_BIT_8                      40u
#define KM_BIT_9                      32u
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct
{
        __packed struct
        {
                int16_t ch[5];
                char s[3];
        } rc;
		__packed struct 
		{
			char pause;
			char fn_1;
			char fn_2;
			char trigger;
		}rc_ker;
        __packed struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
				uint8_t press_m;
        } mouse;
        __packed struct
        {
                uint16_t v;
                uint8_t map[16];   // 128位键盘位图 (自定义客户端协议)
        } key;
        uint8_t func_ctrl;         // 功能控制码 (自定义客户端协议)

} RC_ctrl_t;

typedef __packed struct   // 图传链路数据结构体 （现在以校验作用为主）	
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}remote_data_t;

/* ----------------------- Internal Data ----------------------------------- */
extern RC_ctrl_t RC_ctrl;
static int16_t RC_abs(int16_t value);
extern void RC_restart(uint16_t dma_buf_num);
// 每收到一帧遥控数据时更新时间戳，供失联超时保护使用。
extern void RC_mark_frame_received(void);
extern uint8_t RC_data_is_error(RC_ctrl_t *rc_ctrl);
// 原子取出自上次调用以来收到的鼠标位移；每帧位移只消费一次。
extern void RC_take_mouse_delta(int16_t *dx, int16_t *dy);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
void serial_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
void custom_keymouse_to_rc(volatile const uint8_t *buf, RC_ctrl_t *rc_ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_DBUS_H */
