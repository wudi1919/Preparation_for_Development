//
// Created by Ken on 2026/11/22.
//

#include "Mo_Contral_DJI.h"
#include "tim.h"
#include"HW_Config.h"
#include"Overall_Struct.h"
#include "bsp_fdcan.h"
#include "bsp_Can_Callback.h"
#include "PID_Calculator.h"
#include <math.h>

/* 控制周期（秒），必须等于 TIM17 周期 */
#define MOTOR_CTRL_DT 0.02f
/* C610/GM6020 角度差分测速的一阶低通系数 */
#define SPEED_DIFF_LPF_ALPHA 0.25f

/* CAN 总线负载：每路最多 8 个大疆电机，id 1~4 或 5~8 需按通道单独配置 */
static uint8_t CAN_GM6020[3][8];
static uint8_t CAN_C620_and_C610[3][8];



Mo_PID_Data GM6020_stat[3][8];
Mo_PID_Data C620_stat[3][8];
Mo_PID_Data C610_stat[3][8];
uint8_t CAN3__C620_and_C610_Type_select[3];
/* 电机控制周期回调：由 My_Tim.c 的 TIM17 中断调用，做 PID 计算 + CAN 发送。
 * 所有 CAN 通道的电机统一在这里回调，其他通道配置见下方注释。 */
void Mo_Time_Callback_DJI(void) {
    /* TIM17 每 20ms 触发一次（CubeMX 配好 PSC=239/ARR=9999），
     * 只处理已通过 DJI_Can_Enabel() 开启的通道，
     * 未开启的通道不做 PID 计算、不发报文。 */
    for (uint8_t ch = 1; ch <= 3; ch++) {
        if (!DJI_Can_IsEnabled(ch)) continue;
        Mo_PID_Callbakc_DJI(ch, 1, 2);
    }
    /* ============================================================
     *  Mo_PID_Callbakc_DJI(Can_Ch, GM6020_Ch, C620_And_C610_Ch) 参数含义：
     *    Can_Ch           CAN 外设 1/2/3（对应 hfdcan1/2/3）
     *    GM6020_Ch        1=发 0x1FE(ID1~4)   2=发 0x2FE(ID5~8)
     *    C620_And_C610_Ch 1=发 0x200(ID1~4)   2=发 0x1FF(ID5~8)
     *  某一路的电机编号段不同时，需要单独改这里的第 2/3 个参数。
     * ============================================================ */
}


void Mo_Select_DJI(uint8_t Can_Ch,uint8_t type) {
    CAN3__C620_and_C610_Type_select[Can_Ch-1] = type;
}

void Mo_Set_current_DJI(uint8_t type,uint8_t id,int16_t current,uint8_t can_Ch) {
    /* 越界/非法保护 */
    if (can_Ch == 0 || can_Ch > 3 || id == 0 || id > 8) return;

    uint8_t *buf;
    if (type == GM6020) {
        buf = CAN_GM6020[can_Ch - 1];
    } else if (type == C610 || type == C620) {
        buf = CAN_C620_and_C610[can_Ch - 1];
    } else {
        return;
    }

    /* 每帧 8 字节 = 4 个电机（int16 × 4）。
     * id 1~4 走 CH1（槽 0~3），id 5~8 走 CH2（也映射到槽 0~3）。
     * 实际使用只会用到 CH1 或 CH2 其中一个，所以共用一个缓冲不冲突。 */
    uint8_t slot = (id - 1) & 0x03;                       /* 0~3 */
    buf[slot * 2]     = (uint8_t)((uint16_t)current >> 8);  /* 高字节在前（大端） */
    buf[slot * 2 + 1] = (uint8_t)current;                   /* 低字节在后 */
}

void Mo_Callback_DJI(uint8_t Can_Ch,uint8_t GM6020_Ch,uint8_t C620_And_C610_Ch) {
    if (Can_Ch < 1 || Can_Ch > 3) return;

    FDCAN_HandleTypeDef *hfdcan;
    switch (Can_Ch) {
        case 1: hfdcan = &hfdcan1; break;
        case 2: hfdcan = &hfdcan2; break;
        default: hfdcan = &hfdcan3; break;
    }

    uint8_t ch = Can_Ch - 1;

    /* GM6020：CH1 0x1FE / CH2 0x2FE */
    switch (GM6020_Ch) {
        case 1: fdcanx_send_data(hfdcan, GM6020_IDE_CH1, CAN_GM6020[ch], 8); break;
        case 2: fdcanx_send_data(hfdcan, GM6020_IDE_CH2, CAN_GM6020[ch], 8); break;
    }

    /* C610/C620：CH1 0x200 / CH2 0x1FF */
    switch (C620_And_C610_Ch) {
        case 1: fdcanx_send_data(hfdcan, C610_IDE_CH1, CAN_C620_and_C610[ch], 8); break;
        case 2: fdcanx_send_data(hfdcan, C610_IDE_CH2, CAN_C620_and_C610[ch], 8); break;
    }
}

/* ---------------- 精确增量运动：底层统一实现 ---------------- */
/* 上层只要给「绝对多圈目标 + 限速」，规划、到位锁定、堵转保护都在这里做 */
#define MO_MOVE_ARRIVE_DEG   10.0f     /* 到位死区（电机侧角度） */
#define MO_MOVE_DECEL      2000.0f     /* 规划减速度 deg/s^2，决定刹车距离 */
#define MO_MOVE_VMIN         15.0f     /* 末段最低转速，防止爬不动 */
#define MO_MOVE_STALL_MS     500U      /* 饱和不动持续多久判定堵转 */
#define MO_MOVE_STALL_DEG    20.0f     /* 判定“没在前进”的角度门限 */

typedef struct {
    float    target_deg;    /* 绝对多圈目标 */
    float    v_limit;       /* 本段限速 RPM */
    float    last_remain;   /* 上一周期剩余角度 */
    uint16_t stall_ms;      /* 饱和不动累计时间 */
    uint8_t  state;         /* Mo_MoveState_t */
} Mo_Move_t;

static Mo_Move_t mo_move[3][8];

/* 每个控制周期调用一次：规划速度上限 → 写进位置环 Output 限幅 */
static void Mo_Move_Tick(Mo_PID_Data *stat, uint8_t id, uint8_t can_Ch)
{
    Mo_Move_t *m = &mo_move[can_Ch - 1][id - 1];
    if (m->state != MO_MOVE_RUN) return;

    float mt     = DJI_Angle_MultiTurn_Deg(can_Ch, id);
    float remain = m->target_deg - mt;

    /* 到位：锁在“当前实际角度”，不往指令终点再顶，避免堵转 */
    if (fabsf(remain) < MO_MOVE_ARRIVE_DEG) {
        float a = fmodf(mt, 360.0f);
        if (a < 0.0f) a += 360.0f;
        stat->PosTargetDeg = a;
        stat->Pos.OutMax   = m->v_limit;   /* 恢复保持时的速度上限 */
        stat->CtrlMode = DJI_CTRL_POSITION;
        m->state = MO_MOVE_HOLD;
        return;
    }

    /* 梯形规划：v = min(限速, sqrt(2·a·剩余))，末段自动减速 */
    float v = sqrtf(2.0f * MO_MOVE_DECEL * fabsf(remain));
    if (v > m->v_limit) v = m->v_limit;
    if (v < MO_MOVE_VMIN) v = MO_MOVE_VMIN;
    stat->Pos.OutMax = v;

    /* 增量目标改成“绝对目标”：IncStart = target, PosTarget = 0 */
    stat->IncStartDeg = m->target_deg;
    stat->PosTargetDeg = 0.0f;

    /* 堵转检测：速度环输出饱和且剩余角基本不再减小 */
    if (stat->Speed.Saturated) {
        if (fabsf(remain) > fabsf(m->last_remain) - MO_MOVE_STALL_DEG) {
            m->stall_ms += (uint16_t)(MOTOR_CTRL_DT * 1000.0f);
            if (m->stall_ms >= MO_MOVE_STALL_MS) {
                stat->IncStartDeg = mt;     /* 就地卸力 */
                stat->PosTargetDeg = 0.0f;
                m->state = MO_MOVE_STALL;
            }
        } else {
            m->stall_ms = 0;
        }
    } else {
        m->stall_ms = 0;
    }
    m->last_remain = remain;
}

static void Mo_PID_Speed_Calc_DJI(Mo_PID_Data *stat, uint8_t type, uint8_t id, uint8_t can_Ch) {
    /* 0. 精确增量运动状态机：规划速度、到位锁定、堵转保护 */
    Mo_Move_Tick(stat, id, can_Ch);

    /* 1. 测速：C610/GM6020 用多圈角度差分 → RPM，叠一阶 LPF。
     *    角度差分精度 ±0.37 RPM，远优于电调直报速度。C620 直报速度已在 CAN 回调低通。 */
    if (type == C610 || type == GM6020) {
        float turn_now = DJI_Angle_MultiTurn_Deg(can_Ch, id);
        float diff_deg = turn_now - stat->LastTurnDeg;
        stat->LastTurnDeg = turn_now;
        /* 度 → RPM：deg / 360 * 60 / dt */
        float raw_rpm = diff_deg * (60.0f / (360.0f * MOTOR_CTRL_DT));
        stat->Speed.Actual += SPEED_DIFF_LPF_ALPHA * (raw_rpm - stat->Speed.Actual);
    }

    /* 2. 位置环（串级外环，增量式 PID）：输出目标转速，覆盖速度环 Target */
    if (stat->CtrlMode == DJI_CTRL_POSITION || stat->CtrlMode == DJI_CTRL_INCREMENT) {
        float actual_deg = DJI_Angle_MultiTurn_Deg(can_Ch, id);
        float target_deg;
        if (stat->CtrlMode == DJI_CTRL_POSITION) {
            /* 绝对位置：把 0~360° 目标映射到离当前累计角度最近的等价角（最短路径） */
            float diff = fmodf(stat->PosTargetDeg - actual_deg, 360.0f);
            if (diff >  180.0f) diff -= 360.0f;
            if (diff < -180.0f) diff += 360.0f;
            target_deg = actual_deg + diff;
        } else {
            /* 增量角度：起点累计角度 + 增量（可超 360°） */
            target_deg = stat->IncStartDeg + stat->PosTargetDeg;
        }
        PID_Incremental_Calc(&stat->Pos, target_deg, actual_deg);
        stat->Speed.Target = stat->Pos.Out;
    }

    /* 3. 速度环（位置式 PID）→ 力矩 */
    PID_Calculator(&stat->Speed);

    /* 4. 重力前馈（仅位置/增量模式）：加在力矩上，补偿重力负载稳态误差 */
    float out = stat->Speed.Out;
    if (stat->CtrlMode == DJI_CTRL_POSITION || stat->CtrlMode == DJI_CTRL_INCREMENT) {
        float actual_deg = DJI_Angle_MultiTurn_Deg(can_Ch, id);
        float a = fmodf(actual_deg, 360.0f);
        if (a < 0.0f) a += 360.0f;
        out += stat->GravityK * sinf((a + stat->GravityPhase) * (3.14159265f / 180.0f));
    }

    /* 总输出限幅（速度环 PID + 前馈，统一按速度环 OutMax 限） */
    if (out >  stat->Speed.OutMax) out =  stat->Speed.OutMax;
    if (out < -stat->Speed.OutMax) out = -stat->Speed.OutMax;

    /* 5. 写电流/电压指令 */
    Mo_Set_current_DJI(type, id, (int16_t)out, can_Ch);
}

void Mo_PID_Callbakc_DJI(uint8_t Can_Ch,uint8_t GM6020_Ch,uint8_t C620_And_C610_Ch) {
    if (Can_Ch < 1 || Can_Ch > 3) return;

    uint8_t ch = Can_Ch - 1;
    uint8_t gm_base = (GM6020_Ch == 2) ? 5 : 1;         /* GM6020 起始电机号：CH1=1, CH2=5 */
    uint8_t c_base  = (C620_And_C610_Ch == 2) ? 5 : 1;  /* C610/C620 起始电机号 */

    for (uint8_t i = 0; i < 4; i++) {
        uint8_t gid = gm_base + i;
        Mo_PID_Speed_Calc_DJI(&GM6020_stat[ch][gid - 1], GM6020, gid, Can_Ch);

        uint8_t cid = c_base + i;
        if (CAN3__C620_and_C610_Type_select[ch] == C610) {
            Mo_PID_Speed_Calc_DJI(&C610_stat[ch][cid - 1], C610, cid, Can_Ch);
        } else if (CAN3__C620_and_C610_Type_select[ch] == C620) {
            Mo_PID_Speed_Calc_DJI(&C620_stat[ch][cid - 1], C620, cid, Can_Ch);
        }
    }

    /* 发送 */
    Mo_Callback_DJI(Can_Ch, GM6020_Ch, C620_And_C610_Ch);
}
void Mo_PID_Speed_Control_DJI(uint8_t type,uint8_t id,int16_t Speed,uint8_t can_Ch) {
    if (can_Ch == 0 || can_Ch > 3 || id == 0 || id > 8) return;

    Mo_PID_Data *stat;
    switch (type) {
        case GM6020: stat = &GM6020_stat[can_Ch - 1][id - 1]; break;
        case C610:   stat = &C610_stat[can_Ch - 1][id - 1];   break;
        case C620:   stat = &C620_stat[can_Ch - 1][id - 1];   break;
        default: return;
    }

    /* 只负责设置目标转速（PID 计算在 Mo_PID_Callbakc_DJI 里统一做），并切到速度环模式 */
    stat->Speed.Target = Speed;
    stat->CtrlMode = DJI_CTRL_SPEED;
}

/* ---------------- 位置 / 增量角度控制（串级外环） ---------------- */

/* 取电机状态指针（带越界保护），供位置/角度接口复用 */
static Mo_PID_Data* Mo_Get_Stat(uint8_t type, uint8_t id, uint8_t can_Ch) {
    if (can_Ch == 0 || can_Ch > 3 || id == 0 || id > 8) return NULL;
    switch (type) {
        case GM6020: return &GM6020_stat[can_Ch - 1][id - 1];
        case C610:   return &C610_stat[can_Ch - 1][id - 1];
        case C620:   return &C620_stat[can_Ch - 1][id - 1];
        default: return NULL;
    }
}

void Mo_Set_CtrlMode_DJI(uint8_t type, uint8_t id, uint8_t mode, uint8_t can_Ch) {
    Mo_PID_Data *stat = Mo_Get_Stat(type, id, can_Ch);
    if (stat == NULL || mode > DJI_CTRL_INCREMENT) return;
    stat->CtrlMode = mode;
}

/* 绝对位置控制：转到 0~360° 目标角（最短路径），进入位置环模式 */
void Mo_PID_Position_Control_DJI(uint8_t type, uint8_t id, float angle_deg, uint8_t can_Ch) {
    Mo_PID_Data *stat = Mo_Get_Stat(type, id, can_Ch);
    if (stat == NULL) return;
    /* 归一到 0~360 */
    float t = fmodf(angle_deg, 360.0f);
    if (t < 0.0f) t += 360.0f;
    /* 从其它模式切入时清积分，避免旧积分把起始输出顶偏 */
    if (stat->CtrlMode != DJI_CTRL_POSITION) {
        PID_Reset(&stat->Speed);
    }
    stat->PosTargetDeg = t;
    stat->CtrlMode = DJI_CTRL_POSITION;
}

/* 相对增量角度控制：以当前位置为起点转 delta_deg（可超 360°），进入增量模式 */
void Mo_PID_Increment_Control_DJI(uint8_t type, uint8_t id, float delta_deg, uint8_t can_Ch) {
    Mo_PID_Data *stat = Mo_Get_Stat(type, id, can_Ch);
    if (stat == NULL) return;
    /* 记录起点累计角度，目标 = 起点 + 增量（增量可任意大小，含多圈） */
    if (stat->CtrlMode != DJI_CTRL_INCREMENT) {
        PID_Reset(&stat->Speed);   /* 切模式清积分，防止旧积分污染起始输出 */
    }
    stat->IncStartDeg = DJI_Angle_MultiTurn_Deg(can_Ch, id);
    stat->PosTargetDeg = delta_deg;
    stat->CtrlMode = DJI_CTRL_INCREMENT;
}

/* ---------------- 精确增量运动 API ---------------- */

void Mo_Move_Absolute(uint8_t type, uint8_t id, float target_deg, float v_limit_RPM, uint8_t can_Ch) {
    Mo_PID_Data *stat = Mo_Get_Stat(type, id, can_Ch);
    if (stat == NULL) return;

    Mo_Move_t *m = &mo_move[can_Ch - 1][id - 1];

    /* 通道未使能（DJI_Can_Enabel 没开）：不进入运动状态，
     * 否则永远等不到周期回调，状态会一直卡在“运动中” */
    if (!DJI_Can_IsEnabled(can_Ch)) {
        m->state = MO_MOVE_IDLE;
        return;
    }

    m->target_deg  = target_deg;
    m->v_limit     = (v_limit_RPM <= 0.0f) ? stat->Pos.OutMax : v_limit_RPM;
    m->last_remain = target_deg - DJI_Angle_MultiTurn_Deg(can_Ch, id);
    m->stall_ms    = 0;
    m->state       = MO_MOVE_RUN;

    /* 先切到增量模式，目标由 Mo_Move_Tick 每周期刷新 */
    stat->IncStartDeg  = target_deg;
    stat->PosTargetDeg = 0.0f;
    stat->CtrlMode     = DJI_CTRL_INCREMENT;
}

void Mo_Move_Relative(uint8_t type, uint8_t id, float delta_deg, float v_limit_RPM, uint8_t can_Ch) {
    float cur = DJI_Angle_MultiTurn_Deg(can_Ch, id);
    Mo_Move_Absolute(type, id, cur + delta_deg, v_limit_RPM, can_Ch);
}

uint8_t Mo_Move_Status(uint8_t type, uint8_t id, float *remain_deg, uint8_t can_Ch) {
    if (can_Ch == 0 || can_Ch > 3 || id == 0 || id > 8) return MO_MOVE_IDLE;
    Mo_Move_t *m = &mo_move[can_Ch - 1][id - 1];
    if (!DJI_Can_IsEnabled(can_Ch)) {          /* 通道被关掉：状态复位，避免下次开启时残留旧运动 */
        m->state = MO_MOVE_IDLE;
        if (remain_deg != NULL) *remain_deg = 0.0f;
        return MO_MOVE_IDLE;
    }
    if (remain_deg != NULL) {
        *remain_deg = m->target_deg - DJI_Angle_MultiTurn_Deg(can_Ch, id);
    }
    return m->state;
}

/* 配置位置环（增量式 PID），OutMax 为目标转速上限（RPM） */
void Mo_PID_Position_Config_DJI(uint8_t type, uint8_t id, float kp, float ki, float kd, float out_max, uint8_t can_Ch) {
    Mo_PID_Data *stat = Mo_Get_Stat(type, id, can_Ch);
    if (stat == NULL) return;
    PID_Incremental_Config(&stat->Pos, kp, ki, kd, out_max);
}

/* 配置重力前馈：F = k * sin(角度 + phase_deg)（度），加在力矩上 */
void Mo_Set_Gravity_FF_DJI(uint8_t type, uint8_t id, float k, float phase_deg, uint8_t can_Ch) {
    Mo_PID_Data *stat = Mo_Get_Stat(type, id, can_Ch);
    if (stat == NULL) return;
    stat->GravityK = k;
    stat->GravityPhase = phase_deg;
}