//
// Created by Ken on 2026/11/26.
//

#include "Mo_Contral_DM.h"
#include "fdcan.h"
#include "bsp_fdcan.h"
#include "bsp_Can_Callback.h"
#include "PID_Calculator.h"
#include <math.h>
#include <string.h>

/* 单位换算：对外 API 统一用角度制（度 / RPM），内部与电机交互用 rad / rad·s⁻¹ / N·m */
#define DM_DEG2RAD   0.0174532925f
#define DM_RAD2DEG   57.29577951f
#define DM_RADS2RPM  9.549296586f    /* 60 / (2π) */
#define DM_PI        3.14159265f

/* ==================== 协议实例 ====================
 * 1E / 2E 帧结构完全相同，差异只有三项：
 *   1) 编码器数量（位置是否可跨圈）
 *   2) 是否支持力位混控 0x300
 *   3) 量程 p_max / v_max / t_max（存放在 DM_Mo_Data 里，按电机单独标定）
 * 这里的量程字段保留为 0，实际数值走 DM_Mo_Setting_Init() 传入。 */

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

    .p_max          = 0.0f,   /* 未标定，实际值见 DM_Mo_Data */
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

    .p_max          = 0.0f,   /* 未标定，实际值见 DM_Mo_Data */
    .v_max          = 0.0f,
    .t_max          = 0.0f,
};

/* 全部电机状态：索引约定见头文件（[CAN 通道][电机下标]） */
DM_Mo_Data Mo_Setting[3][8];

/* MIT 发送缓冲：每路 8 台 × 8 字节，模仿 DJI 的“先写缓冲、再统一发送” */
static uint8_t CAN_DM[3][8][8];
/* 未收到反馈时的使能重发计数（每 25 个周期 = 500ms 重发一次使能帧） */
static uint16_t dm_enable_try[3][8];

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

/* ==================== 定点数换算 ==================== */
/* 浮点 → 无符号整数（线性映射，超量程截断） */
static uint16_t DM_F2U(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    if (span <= 0.0f) return 0;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * (float)((1u << bits) - 1u) / span);
}

/* MIT 帧打包（8 字节，大端位域）：
 *   D0,D1   : p_des 16bit（高位在前）
 *   D2,D3   : v_des[11:4] ，v_des[3:0] | Kp[11:8]
 *   D4      : Kp[7:0]
 *   D5,D6   : Kd[11:4] ，Kd[3:0] | t_ff[11:8]
 *   D7      : t_ff[7:0] */
static void DM_Pack_MIT(const DM_Mo_Data *m, float p_des, float v_des,
                        float kp, float kd, float t_ff, uint8_t *buf)
{
    uint16_t p_int  = DM_F2U(p_des, -m->p_max,   m->p_max,   16);
    uint16_t v_int  = DM_F2U(v_des, -m->v_max,   m->v_max,   12);
    uint16_t kp_int = DM_F2U(kp,    m->proto->kp_min, m->proto->kp_max, 12);
    uint16_t kd_int = DM_F2U(kd,    m->proto->kd_min, m->proto->kd_max, 12);
    uint16_t t_int  = DM_F2U(t_ff,  -m->t_max,   m->t_max,   12);

    buf[0] = (uint8_t)(p_int >> 8);
    buf[1] = (uint8_t)(p_int & 0xFF);
    buf[2] = (uint8_t)(v_int >> 4);
    buf[3] = (uint8_t)(((v_int & 0x0F) << 4) | (kp_int >> 8));
    buf[4] = (uint8_t)(kp_int & 0xFF);
    buf[5] = (uint8_t)(kd_int >> 4);
    buf[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (t_int >> 8));
    buf[7] = (uint8_t)(t_int & 0xFF);
}

/* ==================== 使能 / 特殊命令 ==================== */
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

/* ==================== 配置 ==================== */
/* 【配置函数】初始化一个电机槽位 */
void DM_Mo_Setting_Init(uint8_t can_ch, uint8_t idx, Mo_DM_Type type, DM_Encoder_Type enc,
                        float p_max, float v_max, float t_max)
{
    if (can_ch == 0 || can_ch > 3 || idx > 7) return;

    DM_Mo_Data *m = &Mo_Setting[can_ch - 1][idx];
    memset(m, 0, sizeof(*m));
    memset(&CAN_DM[can_ch - 1][idx][0], 0, 8);

    m->PID_Data = (Mo_PID_Data){0};
    m->Work_mode = MTI;
    m->Mo_Type   = type;

    m->proto     = DM_Get_Protocol(enc);
    m->can_id    = (uint16_t)(idx + 1);          /* CAN ID = 下标 + 1 */
    m->master_id = (uint16_t)(m->can_id + 0x10); /* 反馈 ID = CAN ID + 0x10 */

    m->p_max = p_max;
    m->v_max = v_max;
    m->t_max = t_max;

    m->kp_mit = 0.0f;   /* 默认纯力矩接口，PID 全部在 MCU 算 */
    m->kd_mit = 0.0f;

    /* 默认进入速度模式，速度环输出限幅 = 力矩量程 */
    m->PID_Data.CtrlMode = DJI_CTRL_SPEED;
    m->PID_Data.Speed.OutMax = (t_max > 0.0f) ? t_max : 1.0f;
    m->PID_Data.Speed.IntMax = m->PID_Data.Speed.OutMax * 3.0f;
}

/* 重力前馈：t_ff = k * sin(角度 + phase_deg)，k 单位 N·m */
void DM_Set_Gravity_FF(uint8_t id, float k, float phase_deg, uint8_t can_ch)
{
    if (can_ch == 0 || can_ch > 3 || id == 0 || id > 8) return;
    Mo_PID_Data *st = &Mo_Setting[can_ch - 1][id - 1].PID_Data;
    st->GravityK = k;
    st->GravityPhase = phase_deg;
}

/* 修改反馈帧 ID（电机 Master ID 与 can_id+0x10 不一致时用） */
void DM_Set_MasterID(uint8_t id, uint16_t master_id, uint8_t can_ch)
{
    if (can_ch == 0 || can_ch > 3 || id == 0 || id > 8) return;
    Mo_Setting[can_ch - 1][id - 1].master_id = master_id;
}

/* MIT 内置位置环增益（一般保持 0） */
void DM_Set_MIT_Gain(uint8_t id, float kp, float kd, uint8_t can_ch)
{
    if (can_ch == 0 || can_ch > 3 || id == 0 || id > 8) return;
    Mo_Setting[can_ch - 1][id - 1].kp_mit = kp;
    Mo_Setting[can_ch - 1][id - 1].kd_mit = kd;
}

/* ==================== 运行时控制 ==================== */
static Mo_PID_Data *DM_Get_PID(uint8_t id, uint8_t can_ch)
{
    if (can_ch == 0 || can_ch > 3 || id == 0 || id > 8) return NULL;
    if (Mo_Setting[can_ch - 1][id - 1].proto == NULL) return NULL;
    return &Mo_Setting[can_ch - 1][id - 1].PID_Data;
}

/* 把角度归一到 ±p_max：MIT 的 p_des 只能表示 ±p_max，控制必须在同一表示域里做，
 * 否则多圈累计值会超出表示范围、被截断后目标永远追不上（表现为“一直转/一直顶”）。 */
static float DM_WrapPMax(float deg, float lim)
{
    float span = 2.0f * lim;
    float x = fmodf(deg + lim, span);
    if (x < 0.0f) x += span;
    return x - lim;
}

/* 速度闭环：目标转速 RPM（直接交给电机内部速度环，帧 0x200+ID） */
void DM_PID_Speed_Control(uint8_t id, int16_t speed_rpm, uint8_t can_ch)
{
    Mo_PID_Data *st = DM_Get_PID(id, can_ch);
    if (st == NULL) return;
    st->Speed.Target = (float)speed_rpm;
    st->CtrlMode = DJI_CTRL_SPEED;
}

/* 【配置函数】轨迹参数：v_lim_dps = 斜坡最大速度（度/秒），kp/kd = 电机内部位置环增益。
 * 定角度/增量模式只靠它把目标做成斜坡，位置保持完全由电机内部 Kp/Kd 完成。 */
void DM_Motion_Config(uint8_t id, float v_lim_dps, float kp, float kd, uint8_t can_ch)
{
    if (can_ch == 0 || can_ch > 3 || id == 0 || id > 8) return;
    DM_Mo_Data *m = &Mo_Setting[can_ch - 1][id - 1];
    if (m->proto == NULL) return;
    m->PID_Data.Pos.OutMax = (v_lim_dps > 0.0f) ? v_lim_dps : 60.0f;
    m->kp_mit = kp;
    m->kd_mit = kd;
}

/* 绝对位置：转到 angle（0~360°），取最短路径，起点 = 当前角度 */
void DM_PID_Position_Control(uint8_t id, float angle_deg, uint8_t can_ch)
{
    Mo_PID_Data *st = DM_Get_PID(id, can_ch);
    if (st == NULL) return;

    DM_Mo_Data *m = &Mo_Setting[can_ch - 1][id - 1];
    float lim = m->p_max * DM_RAD2DEG;
    float cur = DM_WrapPMax(m->pos_multiturn_rad * DM_RAD2DEG, lim);

    float t = fmodf(angle_deg, 360.0f);
    if (t < 0.0f) t += 360.0f;

    float d = t - fmodf(cur, 360.0f);
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;

    float tgt = cur + d;
    if (tgt >  lim) tgt =  lim;
    if (tgt < -lim) tgt = -lim;

    st->IncStartDeg  = cur;          /* 打印/查询用 */
    st->PosTargetDeg = tgt - cur;
    st->Pos.Out      = cur;          /* 斜坡从当前位置起步，不跳变 */
    st->Pos.Target   = tgt;
    st->CtrlMode     = DJI_CTRL_POSITION;
}

/* 相对增量角度：从当前位置再转 delta_deg（可超 360°、多圈） */
void DM_PID_Increment_Control(uint8_t id, float delta_deg, uint8_t can_ch)
{
    Mo_PID_Data *st = DM_Get_PID(id, can_ch);
    if (st == NULL) return;

    DM_Mo_Data *m = &Mo_Setting[can_ch - 1][id - 1];
    float lim = m->p_max * DM_RAD2DEG;
    float cur = DM_WrapPMax(m->pos_multiturn_rad * DM_RAD2DEG, lim);

    float tgt = cur + delta_deg;
    if (tgt >  lim) tgt =  lim;
    if (tgt < -lim) tgt = -lim;

    st->IncStartDeg  = cur;
    st->PosTargetDeg = tgt - cur;
    st->Pos.Out      = cur;
    st->Pos.Target   = tgt;
    st->CtrlMode     = DJI_CTRL_INCREMENT;
}

void DM_Set_CtrlMode(uint8_t id, uint8_t mode, uint8_t can_ch)
{
    Mo_PID_Data *st = DM_Get_PID(id, can_ch);
    if (st == NULL || mode > DJI_CTRL_INCREMENT) return;
    st->CtrlMode = mode;
}

float DM_Pos_MultiTurn_Deg(uint8_t id, uint8_t can_ch)
{
    if (can_ch == 0 || can_ch > 3 || id == 0 || id > 8) return 0.0f;
    return Mo_Setting[can_ch - 1][id - 1].pos_multiturn_rad * DM_RAD2DEG;
}

/* ==================== 周期任务：轨迹斜坡 + 打包发送 ==================== */
/* 斜坡一步：目标角按 v_lim 匀速逼近，输出 p_des / v_des（rad、rad/s）。
 * MIT 的 p_des 只有 ±p_max 的表示范围，超出就截断并清零 v_des（否则会一直顶着）。 */
static void DM_Traj_Step(DM_Mo_Data *m, float dt, float *p_des, float *v_des)
{
    Mo_PID_Data *st = &m->PID_Data;
    float lim_deg = m->p_max * DM_RAD2DEG;

    float step = st->Pos.OutMax * dt;          /* 本周期最多走多少度 */
    float d = st->Pos.Target - st->Pos.Out;
    if (d >  step) d =  step;
    if (d < -step) d = -step;
    st->Pos.Out += d;

    float p = st->Pos.Out;
    float v = d / dt;

    if (p >  lim_deg) { p =  lim_deg; v = 0.0f; }
    if (p < -lim_deg) { p = -lim_deg; v = 0.0f; }

    *p_des = p * DM_DEG2RAD;
    *v_des = v * DM_DEG2RAD;
}

/* 【周期回调】TIM17 里调用，周期必须与 PID dt（20ms）一致。
 * 速度模式 → 发 0x200+ID 的速度帧（电机内部速度环）；
 * 定角度/增量 → MCU 只做斜坡，MIT 帧里带电机内部 Kp/Kd 做位置保持。 */
void Mo_Time_Callback_DM(void)
{
    const float dt = 0.02f;

    for (uint8_t ci = 0; ci < 3; ci++) {
        if (!DM_Can_IsEnabled((uint8_t)(ci + 1))) continue;   /* 该通道未使能 */

        FDCAN_HandleTypeDef *h = DM_Handle((uint8_t)(ci + 1));
        if (h == 0) continue;

        for (uint8_t i = 0; i < 8; i++) {
            DM_Mo_Data *m = &Mo_Setting[ci][i];
            if (m->proto == NULL) continue;
            if (m->p_max <= 0.0f || m->v_max <= 0.0f || m->t_max <= 0.0f) continue; /* 量程未标定 */

            if (!m->fb_valid) {
                /* 尚未收到反馈：交替发“零力矩 MIT”和“使能帧”。
                 * 达妙电机收到帧才回反馈；若上电瞬间使能帧丢了（总线毛刺/电机未上电），
                 * 只发零力矩会让电机一直保持失能、永远不反馈，所以每 500ms 补发一次使能。 */
                if (++dm_enable_try[ci][i] >= 25) {
                    dm_enable_try[ci][i] = 0;
                    DM_Enable_Motor((uint8_t)(ci + 1), m->can_id);
                } else {
                    DM_Pack_MIT(m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, CAN_DM[ci][i]);
                    fdcanx_send_data(h, (uint16_t)(m->proto->bar_mit + m->can_id), CAN_DM[ci][i], 8);
                }
                continue;
            }

            if (m->PID_Data.CtrlMode == DJI_CTRL_SPEED) {
                /* 定速：v_des 直接给电机内部速度环，单位 rad/s */
                float v = m->PID_Data.Speed.Target * (6.2831853f / 60.0f);
                if (v >  m->v_max) v =  m->v_max;
                if (v < -m->v_max) v = -m->v_max;

                union { float f; uint8_t b[4]; } u;
                u.f = v;                                   /* Cortex-M7 小端，可直接字节拷贝 */
                CAN_DM[ci][i][0] = u.b[0];
                CAN_DM[ci][i][1] = u.b[1];
                CAN_DM[ci][i][2] = u.b[2];
                CAN_DM[ci][i][3] = u.b[3];

                fdcanx_send_data(h, (uint16_t)(m->proto->bar_vel + m->can_id), CAN_DM[ci][i], 4);
            } else {
                /* 定角度 / 增量：只把目标做成斜坡，位置保持由电机内部 Kp/Kd 做 */
                float p_des, v_des;
                DM_Traj_Step(m, dt, &p_des, &v_des);

                float t_ff = m->PID_Data.GravityK *
                             sinf(p_des + m->PID_Data.GravityPhase * DM_DEG2RAD);

                DM_Pack_MIT(m, p_des, v_des, m->kp_mit, m->kd_mit, t_ff, CAN_DM[ci][i]);
                fdcanx_send_data(h, (uint16_t)(m->proto->bar_mit + m->can_id), CAN_DM[ci][i], 8);
            }
        }
    }
}
