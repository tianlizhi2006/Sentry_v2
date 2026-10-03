#include "protocol_dbus.h"
#include "dev_serial.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task.h"

RC_ctrl_t RC_ctrl;
remote_data_t remote;

//遥控器出错数据上限
#define RC_CHANNAL_ERROR_VALUE 700
// DR16正常约每7 ms发送一帧；连续100 ms没有收到数据即判定失联。
#define RC_LOST_TIMEOUT_MS 100u

static volatile uint32_t rc_last_frame_tick = 0u;
static volatile bool rc_frame_received = false;
static int16_t mouse_pending_x = 0;
static int16_t mouse_pending_y = 0;
static volatile uint32_t keymouse_override_last_tick = 0u;
static volatile bool keymouse_override_seen = false;

static int16_t clamp_mouse_delta(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static void RC_queue_mouse_delta(int16_t dx, int16_t dy)
{
    taskENTER_CRITICAL();
    mouse_pending_x = clamp_mouse_delta((int32_t)mouse_pending_x + dx);
    mouse_pending_y = clamp_mouse_delta((int32_t)mouse_pending_y + dy);
    taskEXIT_CRITICAL();
}

void RC_take_mouse_delta(int16_t *dx, int16_t *dy)
{
    if (dx == NULL || dy == NULL) return;
    taskENTER_CRITICAL();
    *dx = mouse_pending_x;
    *dy = mouse_pending_y;
    mouse_pending_x = 0;
    mouse_pending_y = 0;
    taskEXIT_CRITICAL();
}

static void RC_clear_mouse_delta(void)
{
    taskENTER_CRITICAL();
    mouse_pending_x = 0;
    mouse_pending_y = 0;
    taskEXIT_CRITICAL();
}

void RC_mark_frame_received(void)
{
    rc_last_frame_tick = HAL_GetTick();
    rc_frame_received = true;
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
	
/* CRC16 初始值 */
static const uint16_t crc16_init = 0xffff;

/* CRC16 查找表 (多项式 0x1021, 初始值 0xFFFF, 不反转) */
static const uint16_t crc16_tab[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief 计算 CRC16 校验值（链式调用）
 * @param p_msg 待校验数据指针
 * @param len   数据长度（字节）
 * @param crc16 初始 CRC 值（首次调用通常为 0xFFFF）
 * @return uint16_t 计算后的 CRC16 值
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if (p_msg == NULL) {
        return 0xffff;
    }

    while (len--) {
        data = *p_msg++;
        crc16 = (crc16 >> 8) ^ crc16_tab[(crc16 ^ data) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief 校验 CRC16 是否正确
 * @param p_msg 完整数据包指针（包含末尾的 2 字节 CRC）
 * @param len   数据包总长度（字节）
 * @return true  校验通过
 * @return false 校验失败（空指针或长度不足或 CRC 不匹配）
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected;

    if ((p_msg == NULL) || (len <= 2)) {
        return false;
    }

    // 计算除最后 2 字节（CRC 字段）外数据的 CRC 值
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    // CRC 字段在数据包中按小端序存储：低字节在前，高字节在后
    return ((w_expected & 0xff) == p_msg[len - 2] &&
            ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
}
	
	
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
	
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    const bool keyboard_mode = switch_is_up(rc_ctrl->rc.s[0]) && switch_is_up(rc_ctrl->rc.s[1]);
    if (!keyboard_mode)
    {
        keymouse_override_seen = false;
    }
    // 独立键鼠帧有效时，基础 DBUS 帧不能用空按键覆盖按下状态。
    const bool use_sbus_keymouse = !keyboard_mode || !keymouse_override_seen
        || (uint32_t)(HAL_GetTick() - keymouse_override_last_tick) > RC_LOST_TIMEOUT_MS;
    if (use_sbus_keymouse)
    {
        rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);
        rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);
        rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);
        rc_ctrl->mouse.press_l = sbus_buf[12];
        rc_ctrl->mouse.press_r = sbus_buf[13];
        rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);
        RC_queue_mouse_delta(rc_ctrl->mouse.x, rc_ctrl->mouse.y);
    }
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
		
}

void serial_to_rc(volatile const uint8_t *buf, RC_ctrl_t *rc_ctrl)
{

    if (buf == NULL || rc_ctrl == NULL) {
        return;
    }

    /* 帧头校验 */
    if (buf[0] != 0xA9 || buf[1] != 0x53) {
        return;  // 帧头错误，丢弃
    }

	 // 通道0：偏移16，长度11
    remote.ch_0 = (int16_t)((buf[2] | ((buf[3] & 0x07) << 8)) & 0x07FF);
    // 通道1：偏移27，长度11
    remote.ch_1 = (int16_t)((((buf[4] & 0x3F) << 5) | (buf[3] >> 3)) & 0x07FF);
    // 通道2：偏移38，长度11
    remote.ch_2 = (int16_t)(( ((buf[6] & 0x01) << 10) | ((uint16_t)buf[5] << 2) | (buf[4] >> 6) ) & 0x07FF);
    // 通道3：偏移49，长度11
    remote.ch_3 = (int16_t)(( ((buf[7] & 0x0F) << 7) | ((buf[6] >> 1) & 0x7F) ) & 0x07FF);

    // 挡位开关：偏移60，长度2
    remote.mode_sw = (buf[7] >> 4) & 0x03;
    // 暂停按键：偏移62，长度1
    remote.pause = (buf[7] >> 6) & 0x01;
    // 自定义按键(左)：偏移63，长度1
    remote.fn_1 = (buf[7] >> 7);
    // 自定义按键(右)：偏移64，长度1
    remote.fn_2 = (buf[8] & 0x01);

    // 拨轮：偏移65，长度11
    remote.wheel = (int16_t)((((buf[9] & 0x0F) << 7) | ((buf[8] >> 1) & 0x7F)) & 0x07FF);

    // 扳机键：偏移76，长度1
    remote.trigger = (buf[9] >> 4) & 0x01;

    // 3. 鼠标移动数据（有符号16位，小端序）
    remote.mouse_x = (int16_t)(buf[10] | (buf[11] << 8));
    remote.mouse_y = (int16_t)(buf[12] | (buf[13] << 8));
    remote.mouse_z = (int16_t)(buf[14] | (buf[15] << 8));

    // 4. 鼠标按键（各2位，只取其低位作为布尔状态）
    remote.mouse_left   = buf[16] & 0x01;
    remote.mouse_right  = (buf[16] >> 2) & 0x01;
    remote.mouse_middle = (buf[16] >> 4) & 0x01;

    // 5. 键盘数据（16位掩码）
    remote.key = (uint16_t)(buf[17] | (buf[18] << 8));

    // 6. CRC16 字段（可选：保存到结构体中供参考）
    remote.crc16 = (uint16_t)(buf[19] | (buf[20] << 8));
	
	if (verify_crc16_check_sum((uint8_t *)buf, 21))
	{
		if( rc_ctrl->rc.s[0] == 1 && rc_ctrl->rc.s[1] == 1 ){
	
		/* ---------- 摇杆通道（11位，无符号，需减中值） ---------- */
		// 通道0: 右摇杆水平，偏移16
		rc_ctrl->rc.ch[0] = (int16_t)((buf[2] | ((buf[3] & 0x07) << 8)) & 0x07FF);
		// 通道1: 右摇杆竖直，偏移27
		rc_ctrl->rc.ch[1] = (int16_t)((((buf[4] & 0x3F) << 5) | (buf[3] >> 3)) & 0x07FF);
		// 通道2: 左摇杆竖直，偏移38
		rc_ctrl->rc.ch[2] = (int16_t)(( ((buf[6] & 0x01) << 10) | ((uint16_t)buf[5] << 2) | (buf[4] >> 6) ) & 0x07FF);
		// 通道3: 左摇杆水平，偏移49
		rc_ctrl->rc.ch[3] = (int16_t)(( ((buf[7] & 0x0F) << 7) | ((buf[6] >> 1) & 0x7F) ) & 0x07FF);
	
		/* ---------- 拨轮（作为第5通道），偏移65 ---------- */
		rc_ctrl->rc.ch[4] = (int16_t)((((buf[9] & 0x0F) << 7) | ((buf[8] >> 1) & 0x7F)) & 0x07FF);
	
		/* ---------- 挡位切换开关（2位），偏移60 ---------- */
		rc_ctrl->rc.s[2] = (char)((buf[7] >> 4) & 0x03);   // 0:C, 1:N, 2:S
	
		/* 注意：暂停按键（偏移62）、自定义按键（偏移63/64）、扳机键（偏移76）
		在RC_ctrl_t结构体中无对应字段，此处丢弃 */
		
		rc_ctrl->rc_ker.pause  = (char)((buf[7] >> 6) & 0x01);
		rc_ctrl->rc_ker.fn_1   = (char)(buf[7] >> 7);
		rc_ctrl->rc_ker.fn_2   = (char)(buf[8] & 0x01);
		rc_ctrl->rc_ker.trigger = (char)((buf[9] >> 4) & 0x01);
	
		/* ---------- 鼠标移动数据（16位有符号），偏移80/96/112 ---------- */
		// 鼠标X轴
		uint16_t mx = (buf[10] | (buf[11] << 8));
		rc_ctrl->mouse.x = (int16_t)mx;
		// 鼠标Y轴
		uint16_t my = (buf[12] | (buf[13] << 8));
		rc_ctrl->mouse.y = (int16_t)my;
		// 鼠标Z轴（滚轮）
		uint16_t mz = (buf[14] | (buf[15] << 8));
		rc_ctrl->mouse.z = (int16_t)mz;
	
		/* ---------- 鼠标按键（各2位，取最低位），偏移128/130/132 ---------- */
		// 左键：位于字节16的位0-1
		rc_ctrl->mouse.press_l = buf[16] & 0x01;
		// 右键：位于字节16的位2-3
		rc_ctrl->mouse.press_r = (buf[16] >> 2) & 0x01;
		// 中键：位于字节16的位4-5
		rc_ctrl->mouse.press_m = (buf[16] >> 4) & 0x01;

		/* ---------- 键盘数据（16位掩码），偏移136 ---------- */
		rc_ctrl->key.v = (buf[17] | (buf[18] << 8));

		/* ---------- CRC校验（可选，此处忽略） ---------- */
		// CRC位于字节19-20，偏移152-168

		/* ---------- 摇杆/拨轮减中值，转换为有符号数 ---------- */
		for (int i = 0; i < 5; i++) 
		{
			rc_ctrl->rc.ch[i] -= RC_CH_VALUE_OFFSET;
		}
		RC_queue_mouse_delta(rc_ctrl->mouse.x, rc_ctrl->mouse.y);
		keymouse_override_last_tick = HAL_GetTick();
		keymouse_override_seen = true;
	}
		
    }
	else
	{
		rc_ctrl->rc.ch[0] = 0;
		rc_ctrl->rc.ch[1] = 0;
		rc_ctrl->rc.ch[2] = 0;
		rc_ctrl->rc.ch[3] = 0;
		rc_ctrl->rc.ch[4] = 0;
		rc_ctrl->mouse.x = 0;
		rc_ctrl->mouse.y = 0;
		rc_ctrl->mouse.z = 0;
		rc_ctrl->mouse.press_l = 0;
		rc_ctrl->mouse.press_r = 0;
		rc_ctrl->mouse.press_m = 0;
		rc_ctrl->key.v = 0;
		rc_ctrl->func_ctrl = 0;
		for (uint8_t i = 0; i < 16; i++) rc_ctrl->key.map[i] = 0;
	}
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(RC_ctrl_t *rc_ctrl)
{
    // 数值仍可能保持在最后一次合法状态，因此必须独立检查接收时间。
    if (!rc_frame_received || (uint32_t)(HAL_GetTick() - rc_last_frame_tick) > RC_LOST_TIMEOUT_MS)
    {
        goto error;
    }
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl->rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl->rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl->rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl->rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl->rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl->rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    keymouse_override_seen = false;
    RC_clear_mouse_delta();
    rc_ctrl->rc.ch[0] = 0;
    rc_ctrl->rc.ch[1] = 0;
    rc_ctrl->rc.ch[2] = 0;
    rc_ctrl->rc.ch[3] = 0;
    rc_ctrl->rc.ch[4] = 0;
    rc_ctrl->rc.s[0] = 0;
    rc_ctrl->rc.s[1] = 0;
    rc_ctrl->mouse.x = 0;
    rc_ctrl->mouse.y = 0;
    rc_ctrl->mouse.z = 0;
    rc_ctrl->mouse.press_l = 0;
    rc_ctrl->mouse.press_r = 0;
    rc_ctrl->key.v = 0;
    rc_ctrl->func_ctrl = 0;
    for (uint8_t i = 0; i < 16; i++) rc_ctrl->key.map[i] = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart5);
    __HAL_DMA_DISABLE(&hdma_uart5_rx);

    ((DMA_Stream_TypeDef*)hdma_uart5_rx.Instance)->NDTR  = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_uart5_rx);
    __HAL_UART_ENABLE(&huart5);

}

//取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/* ===== 128位位图 -> 传统16位key.v 映射表 ===== */
static const uint8_t km_bit_to_key_v[16] = {
	22, 18, 0, 3, 26, 28, 16, 4, 17, 5, 6, 25, 23, 2, 21, 1
};

#define KM_BIT(n)       (((buf[8 + (n) / 8]) >> ((n) % 8)) & 0x01)
#define KM_OR(a, b)     (KM_BIT(a) || KM_BIT(b))

void custom_keymouse_to_rc(volatile const uint8_t *buf, RC_ctrl_t *rc_ctrl)
{
	if (buf == NULL || rc_ctrl == NULL) return;

	if (buf[0] != 0x5A || buf[1] != 0xA5) return;

	uint16_t crc_calc = get_crc16_check_sum((uint8_t *)buf, 25, crc16_init);
		uint16_t k = 0;
	uint16_t crc_recv = ((uint16_t)buf[25] << 8) | buf[26];
	if (crc_calc != crc_recv) goto zero_all;

	if (!(rc_ctrl->rc.s[0] == RC_SW_UP && rc_ctrl->rc.s[1] == RC_SW_UP)) return;

	rc_ctrl->mouse.x = (int16_t)(buf[2] | (buf[3] << 8));
	rc_ctrl->mouse.y = (int16_t)(buf[4] | (buf[5] << 8));
	rc_ctrl->mouse.z = (int8_t)buf[6];
	rc_ctrl->mouse.press_l = buf[7] & 0x01;
	rc_ctrl->mouse.press_r = (buf[7] >> 1) & 0x01;
	rc_ctrl->mouse.press_m = (buf[7] >> 2) & 0x01;

	for (uint8_t i = 0; i < 16; i++) rc_ctrl->key.map[i] = buf[8 + i];

	if (KM_BIT(km_bit_to_key_v[0]))  k |= (1 << 0);
	if (KM_BIT(km_bit_to_key_v[1]))  k |= (1 << 1);
	if (KM_BIT(km_bit_to_key_v[2]))  k |= (1 << 2);
	if (KM_BIT(km_bit_to_key_v[3]))  k |= (1 << 3);
	if (KM_OR(26, 27))               k |= (1 << 4);
	if (KM_OR(28, 29))               k |= (1 << 5);
	if (KM_BIT(km_bit_to_key_v[6]))  k |= (1 << 6);
	if (KM_BIT(km_bit_to_key_v[7]))  k |= (1 << 7);
	if (KM_BIT(km_bit_to_key_v[8]))  k |= (1 << 8);
	if (KM_BIT(km_bit_to_key_v[9]))  k |= (1 << 9);
	if (KM_BIT(km_bit_to_key_v[10])) k |= (1 << 10);
	if (KM_BIT(km_bit_to_key_v[11])) k |= (1 << 11);
	if (KM_BIT(km_bit_to_key_v[12])) k |= (1 << 12);
	if (KM_BIT(km_bit_to_key_v[13])) k |= (1 << 13);
	if (KM_BIT(km_bit_to_key_v[14])) k |= (1 << 14);
	if (KM_BIT(km_bit_to_key_v[15])) k |= (1 << 15);
	rc_ctrl->key.v = k;

	rc_ctrl->func_ctrl = buf[24];

	for (int i = 0; i < 5; i++) rc_ctrl->rc.ch[i] = 0;
	RC_queue_mouse_delta(rc_ctrl->mouse.x, rc_ctrl->mouse.y);
	keymouse_override_last_tick = HAL_GetTick();
	keymouse_override_seen = true;
	return;

zero_all:
	RC_clear_mouse_delta();
	rc_ctrl->rc.ch[0] = 0; rc_ctrl->rc.ch[1] = 0; rc_ctrl->rc.ch[2] = 0;
	rc_ctrl->rc.ch[3] = 0; rc_ctrl->rc.ch[4] = 0;
	rc_ctrl->mouse.x = 0; rc_ctrl->mouse.y = 0; rc_ctrl->mouse.z = 0;
	rc_ctrl->mouse.press_l = 0; rc_ctrl->mouse.press_r = 0; rc_ctrl->mouse.press_m = 0;
	rc_ctrl->key.v = 0; rc_ctrl->func_ctrl = 0;
	for (uint8_t i = 0; i < 16; i++) rc_ctrl->key.map[i] = 0;
}

#undef KM_BIT
#undef KM_OR
