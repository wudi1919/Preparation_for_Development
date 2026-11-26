//
// Created by Ken on 2026/11/26.
//

#include "Mo_Contral_DM.h"
#include "fdcan.h"
#include "bsp_fdcan.h"

/* ==================== 协议实例 ====================
 * 1E / 2E 帧结构完全相同，差异只有三项：
 *   1) 编码器数量（位置是否可跨圈）
 *   2) 是否支持力位混控 0x300
 *   3) 量程 p_max / v_max / t_max（以调试助手为准，不同型号不同）
 * 量程留 0 表示未标定，标定后反馈反算和 MIT 打包才能用。 */

const DM_Protocol_t DM_Protocol_1E = {
    .name           = "DM-1EC",
    .enc            = DM_ENC_1E,
    .multi_turn     = 0,

    .bar_mit        = 0x000,
    .bar_pos_vel    = 0x100,
    .bar_vel        = 0x200,
    .bar_force_pos  = 0x300,
    .has_force_pos  = 0,

    .cmd_enable     = 0xFC,
    .cmd_disable    = 0xFD,
    .cmd_set_zero   = 0xFE,
    .cmd_clear_err  = 0xFB,

    .param_id       = 0x7FF,
    .cmd_param_read = 0x33,
    .cmd_param_write = 0x55,
    .cmd_param_save = 0xAA,

    .fb_id_default  = 0x00,
    .fb_id_mask     = 0x0F,
    .fb_err_shift   = 4,

    .bits_pos       = 16,
    .bits_vel       = 12,
    .bits_tor       = 12,
    .bits_kp        = 12,
    .bits_kd        = 12,

    .kp_min         = 0.0f,
    .kp_max         = 500.0f,
    .kd_min         = 0.0f,
    .kd_max         = 5.0f,

    .p_max          = 0.0f,   /* 未标定 */
    .v_max          = 0.0f,
    .t_max          = 0.0f,
};

const DM_Protocol_t DM_Protocol_2E = {
    .name           = "DM-2EC",
    .enc            = DM_ENC_2E,
    .multi_turn     = 1,

    .bar_mit        = 0x000,
    .bar_pos_vel    = 0x100,
    .bar_vel        = 0x200,
    .bar_force_pos  = 0x300,
    .has_force_pos  = 1,

    .cmd_enable     = 0xFC,
    .cmd_disable    = 0xFD,
    .cmd_set_zero   = 0xFE,
    .cmd_clear_err  = 0xFB,

    .param_id       = 0x7FF,
    .cmd_param_read = 0x33,
    .cmd_param_write = 0x55,
    .cmd_param_save = 0xAA,

    .fb_id_default  = 0x00,
    .fb_id_mask     = 0x0F,
    .fb_err_shift   = 4,

    .bits_pos       = 16,
    .bits_vel       = 12,
    .bits_tor       = 12,
    .bits_kp        = 12,
    .bits_kd        = 12,

    .kp_min         = 0.0f,
    .kp_max         = 500.0f,
    .kd_min         = 0.0f,
    .kd_max         = 5.0f,

    .p_max          = 0.0f,   /* 未标定 */
    .v_max          = 0.0f,
    .t_max          = 0.0f,
};

/* 按编码器类型索引协议 */
const DM_Protocol_t *DM_Get_Protocol(DM_Encoder_Type enc)
{
    switch (enc) {
        case DM_ENC_1E: return &DM_Protocol_1E;
        case DM_ENC_2E: return &DM_Protocol_2E;
        default:        return &DM_Protocol_2E;
    }
}

/* 按 CAN 通道号取句柄 */
static FDCAN_HandleTypeDef *DM_Handle(uint8_t can_ch)
{
    switch (can_ch) {
        case 1:  return &hfdcan1;
        case 2:  return &hfdcan2;
        case 3:  return &hfdcan3;
        default: return 0;
    }
}

/* 使能电机：帧 ID = 电机 CAN ID，D[0..6]=0xFF，D[7]=协议里的使能命令。
 * 收到反馈帧 ERR=1 表示使能成功。 */
void DM_Motor_Enable(uint8_t can_ch, uint16_t can_id, DM_Encoder_Type enc)
{
    FDCAN_HandleTypeDef *h = DM_Handle(can_ch);
    if (h == 0) return;

    const DM_Protocol_t *p = DM_Get_Protocol(enc);

    uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, p->cmd_enable };
    fdcanx_send_data(h, can_id, buf, 8);
}

void DM_Enable_Motor(uint8_t can_ch, uint16_t can_id)
{
    /* 兼容旧调用：1E/2E 的使能命令相同，默认按 2E 取协议 */
    DM_Motor_Enable(can_ch, can_id, DM_ENC_2E);
}
