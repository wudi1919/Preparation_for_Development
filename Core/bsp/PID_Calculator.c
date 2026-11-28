//
// Created by Ken on 2026/10/18.
//

#include "PID_Calculator.h"
#include "Overall_Struct.h"
#include <math.h>



/* ================================================================
 *  控制器周期：TIM17 在 APB2(120MHz) 上，PSC=239, ARR=9999
 *  120MHz / (239+1) / (9999+1) = 120MHz / 240 / 10000 = 50Hz = 20ms
 *  ⚠️ 之前硬编码 0.01f 是错的！已修正为 0.02f
 * ================================================================ */
#define PID_SAMPLE_TIME_S 0.02f

/* 积分分离默认带宽（RPM）。实际值优先取 PID_Data.IActivateBand，
 * 为 0 时回退到本默认值；建议配置为该环最大目标值的 1.2 倍。
 * 带宽若小于速度目标，启动瞬间积分会被冻结，重载下无法起转。
 * 抗饱和由 Out 饱和判断 + IntMax 双限幅兜底。 */
#define PID_I_ACTIVATE_BAND  200.0f  // RPM

/**
 * @brief  位置式 PID 计算（速度环单环）
 * @param  p 指向 PID_Data 的指针（不能为 NULL）
 * @note   内部完成：积分分离 + 抗饱和 + 双限幅。
 *         D 项基于滤波后的 Actual（CAN 回调里已做 LPF），噪声天然受控。
 */
void PID_Calculator(PID_Data * p) {
    if (p == NULL) return;

    /* 本次误差 + 误差差（D 项用） */
    float err_raw = p->Target - p->Actual;
    float d_err   = err_raw - p->Error0;          // Δe = e[k] - e[k-1]
    p->Error1     = p->Error0;
    p->Error0     = err_raw;

    /* ---------- 积分分离：只在误差小时才积分，防大误差超调 ----------
     * 带宽优先取结构体字段（每台电机可独立配置），为 0 时回退到默认值。 */
    float i_band = (p->IActivateBand > 0.0f) ? p->IActivateBand : PID_I_ACTIVATE_BAND;
    if (fabsf(err_raw) < i_band) {
        int sat_hi = (p->Out >=  p->OutMax) && (err_raw > 0);
        int sat_lo = (p->Out <= -p->OutMax) && (err_raw < 0);
        if (!sat_hi && !sat_lo) {
            p->ErrorInt += err_raw * PID_SAMPLE_TIME_S;
        }
    }

    /* 积分限幅：限的是“积分对输出的贡献” Ki·ErrorInt ≤ IntMax。
     * 这样 IntMax 的含义与 Ki 解耦，调 Ki 不会顺带改变抗饱和能力。 */
    if (p->Ki > 1e-6f) {
        float i_raw_max = p->IntMax / p->Ki;
        if (p->ErrorInt >  i_raw_max) p->ErrorInt =  i_raw_max;
        if (p->ErrorInt < -i_raw_max) p->ErrorInt = -i_raw_max;
    } else {
        p->ErrorInt = 0.0f;   /* Ki = 0 时积分无意义，直接清零 */
    }

    /* ---------- PID 计算 ----------
     * 注意：不要加固定摩擦前馈——摩擦随温度变化 60~140（实测），
     * 固定前馈会在热机时过补偿，反而注入振荡。交给积分自适应即可。 */
    float out_raw = p->Kp * err_raw
                  + p->Ki * p->ErrorInt
                  + p->Kd * d_err / PID_SAMPLE_TIME_S;

    /* 输出限幅（±OutMax），同时给出饱和标志供堵转检测使用 */
    p->Saturated = (out_raw >= p->OutMax || out_raw <= -p->OutMax) ? 1U : 0U;
    p->Out = out_raw;
    if (p->Out >  p->OutMax) p->Out =  p->OutMax;
    if (p->Out < -p->OutMax) p->Out = -p->OutMax;
}

/**
 * @brief  清空 PID 运行状态，保留参数
 * @note   切换控制模式（位置/速度/增量）时调用，避免旧积分与误差历史污染新模式的起始输出
 */
void PID_Reset(PID_Data * p) {
    if (p == NULL) return;
    p->Error0    = 0.0f;
    p->Error1    = 0.0f;
    p->ErrorInt  = 0.0f;
    p->Out       = 0.0f;
    p->Saturated = 0U;
}

/**
 * @brief  设置积分分离带宽（RPM / 与目标同单位）
 * @note   0 表示使用默认值 PID_I_ACTIVATE_BAND；一般取该环最大目标值的 1.2 倍，
 *         否则速度目标高于带宽时启动瞬间积分冻结，重载下无法起转。
 */
void PID_Set_IBand(PID_Data * p, float band) {
    if (p == NULL) return;
    p->IActivateBand = (band > 0.0f) ? band : 0.0f;
}

/**
 * @brief  配置 PID 参数（Kp/Ki/Kd 及输出、积分限幅）
 * @param  p 指向 PIO_Data 的指针（不能为 NULL）
 */

void PID_config(PID_Data * p, float kp, float ki, float kd, float OutMax, float IntMax) {
    if (p == NULL) return;   // 空指针防护，防止死机
    p->Kp     = kp;
    p->Ki     = ki;
    p->Kd     = kd;
    p->OutMax = OutMax;
    p->IntMax = IntMax;
    p->ErrorInt = 0.0f;      /* 换参数时清零积分，防止旧积分污染新参数的起始响应 */
}

/* ================================================================
 *  增量式 PID（位置环，串级外环）
 *  公式：Δu = Kp*(e-e1) + Ki*e + Kd*(e-2e1+e2)
 *        Out += Δu，再对 Out 限幅
 *  特点：
 *    - 输出为增量累加，目标跳变不会造成输出突变，适合位置环；
 *    - 稳态时 e=0 → Δu=0，Out 保持，天然抗积分饱和（输出不继续漂移）；
 *    - 重力等持续负载造成的稳态误差由 Ki*e 项逐步消除。
 *  dt 固定 20ms（TIM17），已折入 Ki/Kd，本函数不显式乘 dt。
 * ================================================================ */
void PID_Incremental_Calc(IncPID_Data * p, float target, float actual) {
    if (p == NULL) return;

    float e0 = target - actual;
    float delta = p->Kp * (e0 - p->Error1)
                + p->Ki * e0
                + p->Kd * (e0 - 2.0f * p->Error1 + p->Error2);

    p->Out += delta;
    if (p->Out >  p->OutMax) p->Out =  p->OutMax;
    if (p->Out < -p->OutMax) p->Out = -p->OutMax;

    /* 递推误差历史 */
    p->Error2 = p->Error1;
    p->Error1 = e0;
    p->Error0 = e0;
}

void PID_Incremental_Config(IncPID_Data * p, float kp, float ki, float kd, float OutMax) {
    if (p == NULL) return;
    p->Kp = kp;
    p->Ki = ki;
    p->Kd = kd;
    p->OutMax = OutMax;
    /* 换参数清空历史，防止旧状态污染新参数起始响应 */
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->Error2 = 0.0f;
    p->Out    = 0.0f;
}
