# 电机控制使用说明（大疆 C610 / GM6020 + 达妙 DM）

本工程对两类电机提供独立但一致的接口：

- **大疆**（C610 / C620 / GM6020）：MCU 侧串级 PID
  「位置环（增量式）→ 目标转速 → 速度环（位置式）→ 电流/电压」
- **达妙**（DM 系列，MIT 协议）：
  - 定速 → 发 `0x200+ID` 速度帧，**用电机内部速度环**
  - 定角度 / 增量 → MCU **只做轨迹斜坡**，位置保持由**电机内部 Kp/Kd** 完成

> 核心调用模型：你只在任意函数里**设置目标/参数**；真正的计算与 CAN 发送由
> `TIM17` 中断（20ms）里的 `Mo_Time_Callback_DJI()` / `Mo_Time_Callback_DM()` 自动完成，
> **不要手动发 CAN**。
> 所有配置（选型、PID、量程、目标）都写在 [HW_Config.c](Core/Hardware/HW_Config.c)，
> 主函数只调 `Mo_Init()` 一次。

---

## 1. 电机类型与指令单位

| 类型 | 常量 | 控制量 | OutMax 建议 |
|------|------|--------|-------------|
| C610 (M2006) | `C610` | 电流指令 | `1000`（约 ±1A，保守） |
| C620 | `C620` | 电流指令 | `1000` |
| GM6020 | `GM6020` | 电压指令 | `30000`（约 ±30V 满量程） |

- C610/C620 用 `Mo_Select_DJI(can_ch, C610)` 指定某路 CAN 上的类型。
- 速度环 PID 输出限幅（`OutMax`）必须按上表设置，否则扭矩/电压量级不对。

---

## 2. 控制模式

```c
typedef enum {
    DJI_CTRL_POSITION  = 0,   // 绝对位置环（0~360°，取最短路径，默认）
    DJI_CTRL_SPEED     = 1,   // 速度环
    DJI_CTRL_INCREMENT = 2    // 相对增量角度环（可超 360°，多圈累计）
} DJI_CtrlMode;
```

---

## 3. API 一览

```c
// 【配置函数】电机初始化 + PID 参数配置统一入口，主函数只调一次
void Mo_Init_DJI(void);

// 电机控制周期回调：由 TIM17 中断调用，做 PID 计算 + CAN 发送
void Mo_Time_Callback_DJI(void);

// 指定某路 CAN 上 C610/C620 的类型（每路只会是其中一种）
void Mo_Select_DJI(uint8_t can_ch, uint8_t type);

// 速度环目标（RPM，带符号），进入速度环模式
void Mo_PID_Speed_Control_DJI(uint8_t type, uint8_t id, int16_t speed, uint8_t can_ch);

// 绝对位置目标（0~360°，度），进入位置环模式
void Mo_PID_Position_Control_DJI(uint8_t type, uint8_t id, float angle_deg, uint8_t can_ch);

// 相对增量角度（度，可超 360），以当前位置为起点，进入增量模式
void Mo_PID_Increment_Control_DJI(uint8_t type, uint8_t id, float delta_deg, uint8_t can_ch);

/* ---- 精确增量运动（推荐）：梯形规划 + 到位锁定 + 堵转保护 ---- */
// 绝对多圈目标（以多圈计数为基准，可超 360°，与当前误差无关，天然不累积）
void Mo_Move_Absolute(uint8_t type, uint8_t id, float target_deg, float v_limit_RPM, uint8_t can_ch);
// 相对运动：内部换算成「当前多圈角 + delta」
void Mo_Move_Relative(uint8_t type, uint8_t id, float delta_deg, float v_limit_RPM, uint8_t can_ch);
// 查询状态：0=空闲 1=运动中 2=到位保持 3=堵转卸力；remain 返回剩余角度（可传 NULL）
uint8_t Mo_Move_Status(uint8_t type, uint8_t id, float *remain_deg, uint8_t can_ch);

// 配置位置环（增量式 PID），out_max 为目标转速上限（RPM）
void Mo_PID_Position_Config_DJI(uint8_t type, uint8_t id, float kp, float ki, float kd, float out_max, uint8_t can_ch);

// 重力前馈：F = k * sin(角度 + phase_deg)，直接加在力矩上
void Mo_Set_Gravity_FF_DJI(uint8_t type, uint8_t id, float k, float phase_deg, uint8_t can_ch);

// 手动切模式
void Mo_Set_CtrlMode_DJI(uint8_t type, uint8_t id, uint8_t mode, uint8_t can_ch);

// 速度环参数（位置式 PID）：kp, ki, kd, out_max, int_max
void PID_config(PID_Data *pid, float kp, float ki, float kd, float out_max, float int_max);
```

电机编号约定：GM6020 / C610 每路 CAN 最多 8 个，`id` 为 1~8；
`id 1~4` 走 CH1，`id 5~8` 走 CH2。

---

## 4. 例程

### 4.0 初始化与周期回调

`Mo_Init_DJI()`（配置函数）在 main 里调一次，`Mo_Time_Callback_DJI()`（回调函数）由 TIM17 中断自动调用：

```c
Mo_Init();       // 电机全部初始化：DJI + 达妙 + 通道使能（main 里只调这一个）
```

`Mo_Init()` 依次调用 `Mo_Init_DJI()` / `Mo_Init_DM()`，再按电机表自动打开对应 CAN 通道。
具体选型、PID、量程、初始目标全部写在 [HW_Config.c](Core/Hardware/HW_Config.c)，主函数不参与配置。

**使用规则**：
- 换电机 / 改参数 / 改初始目标，都在 `HW_Config.c` 的配置表里改，**不要往 `main.c` 里堆初始化**。
- 多路 CAN 电机：在表里加一行（`can_ch` 填 2 或 3），通道使能会自动打开。
- GM6020：`OutMax` 改 `30000`（电压指令）、数组用 `GM6020_stat`，其余同理。
- 上电不想让电机就动：把表中 `mode` 设为 `DJI_CTRL_SPEED`、`target = 0`。
- 主循环只保留遥测打印与串口调参，不再放控制逻辑。

**① `Mo_Init_DJI()` / `Mo_Init_DM()`** —— 各自遍历配置表（C610 电调范例在 `HW_Config.c`）：

```c
// 大疆表：一行一台
{ .can_ch = 1, .id = 5, .type = C610,
  .spd_kp = 3.0f, .spd_ki = 2.0f, .spd_kd = 0.45f,
  .spd_out_max = 1000.0f, .spd_int_max = 3000.0f,
  .pos_kp = 2.0f, .pos_ki = 0.3f, .pos_kd = 0.0f, .pos_rpm_max = 120.0f,
  .grav_k = 0.0f, .grav_phase = 0.0f,
  .mode = DJI_CTRL_POSITION, .target = 90.0f },   // 上电转到 90°
```

**② `Mo_Time_Callback_DJI()` / `Mo_Time_Callback_DM()`** —— 周期回调（TIM17 中断里调用）：

```c
void Mo_Time_Callback_DJI(void) {
    Mo_PID_Callbakc_DJI(1, 1, 2);   // 其他 CAN 通道见函数内注释
}
void Mo_Time_Callback_DM(void) { /* 遍历使能通道，定速/定位分别发帧 */ }
```

> 回调在 `My_Tim.c` 的 TIM17 分支里，默认是注释状态；要跑电机需取消注释。

---

### 4.1 C610 速度环

初始化（选型 + PID）已在 `Mo_Init_DJI()` 里完成，运行时只需设目标：

```c
Mo_PID_Speed_Control_DJI(C610, 5, 120, 1);   // CAN1, ID5, 120 RPM
```

### 4.2 C610 绝对位置（0~360°）

初始化已在 `Mo_Init_DJI()` 里完成，运行时只需设目标：

```c
Mo_PID_Position_Control_DJI(C610, 5, 90.0f, 1);   // 转到 90°
```

### 4.3 C610/C620 相对增量角度（可超 360°，多圈）

```c
// 从当前位置再转 2 圈（720°），多圈累计不丢圈
Mo_PID_Increment_Control_DJI(C610, 5, 720.0f, 1);
```

#### 需要精确圈数 + 停住保持时，用 `Mo_Move_*`

`Mo_PID_Increment_Control_DJI` 只负责“以当前实际位置为起点发一段增量”，
到位判定和保持要上层自己做，多次往返会因到位残差逐次累积。底层另有
一套完整的运动接口，适合“转 N 圈 → 精确停住 → 保持”的场合：

```c
// 转到绝对多圈角 24480°（68 圈），限速 240 RPM
Mo_Move_Absolute(C620, 5, 24480.0f, 240.0f, 1);

// 或相对当前再转 3.4 圈（12240°）
Mo_Move_Relative(C620, 5, 12240.0f, 240.0f, 1);

// 查状态：0 空闲 / 1 运动中 / 2 到位保持 / 3 堵转卸力
float remain;
if (Mo_Move_Status(C620, 5, &remain, 1) == 2) { /* 已到位并锁定 */ }
```

底层每个控制周期自动完成四件事：

| 环节 | 行为 |
|---|---|
| 梯形规划 | 速度上限 = min(设定限速, √(2·a·剩余角))，末段自动减速，不会冲过头 |
| 到位锁定 | 剩余 < 10° 时把目标改成**当前实际角度**并切位置环，不往指令终点顶，因此不会堵转 |
| 堵转保护 | 输出饱和且剩余角 500 ms 内减少不到 20° → 就地卸力，状态置 3 |
| 误差不累积 | 目标始终是**绝对多圈角**，不用“当前 + 增量”反复重算 |

所有 DJI 电机（GM6020 / C610 / C620）语义一致，只换第一个参数即可。
应用层就退化成“给目标 + 查状态”，`Lift_Task.c` 之前的原点记忆、到位判定、
保持锁定都可以删掉。

### 4.4 GM6020 位置环

初始化里改用 `GM6020_stat`、`OutMax=30000`（电压指令），运行时设目标：

```c
Mo_PID_Position_Control_DJI(GM6020, 1, 90.0f, 1);   // 转到 90°
```

### 4.5 机械臂夹取任务（状态机示例）

```c
typedef enum { MOVE_TO_GRIP, GRIP, LIFT, DONE } TaskState;
static TaskState st = MOVE_TO_GRIP;

void Arm_Task(void) {
    switch (st) {
    case MOVE_TO_GRIP:
        Mo_PID_Position_Control_DJI(GM6020, 1, 30.0f, 1);   // 关节1 → 30°
        Mo_PID_Position_Control_DJI(GM6020, 2, 90.0f, 1);   // 关节2 → 90°
        if (fabsf(GM6020_stat[0][0].Pos.Error0) < 2.0f &&
            fabsf(GM6020_stat[0][1].Pos.Error0) < 2.0f) {
            st = GRIP;
        }
        break;
    case GRIP:
        Gripper_Close();      // 夹爪：舵机(PWM)或气动(IO)，不是这套 CAN 速度环
        st = LIFT;
        break;
    case LIFT:
        Mo_PID_Position_Control_DJI(GM6020, 2, 120.0f, 1); // 关节2 → 120°
        if (fabsf(GM6020_stat[0][1].Pos.Error0) < 2.0f) st = DONE;
        break;
    case DONE:
        break;
    }
}
```

把 `Arm_Task()` 放进主循环 `while(1)` 即可（只设目标/读误差，不碰 CAN）。

---

## 5. 到位判断

位置环的当前角度误差存在 `stat->Pos.Error0`（度，= 目标 − 实际）：

```c
// 到位：|误差| < 阈值（如 2°）
if (fabsf(C610_stat[0][4].Pos.Error0) < 2.0f) { /* 到位 */ }
```

> 该值在 TIM17 中断里每 20ms 更新一次，对机械臂到位判断足够。

---

## 6. 重力前馈标定（GM6020 机械臂）

位置环积分项能兜底消除稳态误差，但动态响应慢。标定前馈后响应更快：

```c
// F = k * sin(角度 + phase_deg)，加在电压指令上
Mo_Set_Gravity_FF_DJI(GM6020, 1, k, phase_deg, 1);
```

标定步骤（建议）：
1. `k=0, phase=0` 先跑位置环，记录稳定后的 `Pos.Error0`。
2. 逐步增大 `k`，直到静止时 `Pos.Error0` 接近 0 且不振荡。
3. `phase` 用于对齐「机构机械零点」与「电机电气零点」的相位差，按实际零点偏差整定。

---

## 7. 串口调参协议（速度环）

- 帧格式：`0x55 0xAA + Kp_hi Kp_lo + Ki_hi Ki_lo + Kd_hi Kd_lo`（8 字节）
- 三个 `int16` 大端，值为参数 ×100，范围 0~50
- 成功回复 `OK Kp=... Ki=... Kd=...`

---

## 8. 注意事项

- 速度环 `OutMax` 必须匹配电机类型（C610=1000 电流 / GM6020=30000 电压），否则量级错乱。
- 位置环参数（`Mo_PID_Position_Config_DJI`）和重力前馈（`Mo_Set_Gravity_FF_DJI`）当前为保守占位值，**需按实际负载/机构实测标定**。
- 不要在主循环和中断里同时调用 `Mo_PID_Callbakc_DJI`，会造成重入和重复发送。
- 增量角度环天然支持多圈（超 360°），由内部累计角度实现，不会丢圈。

---

## 9. 达妙电机（DM 系列 / MIT 协议）

### 9.1 三种模式

| 模式 | 下发帧 | 谁在闭环 | 适用 |
|------|--------|----------|------|
| 定速 | `0x200 + CAN ID`（float v_des） | **电机内部速度环** | 匀速转动、轮子 |
| 定角度 | `CAN ID`（MIT：p_des/v_des/Kp/Kd/t_ff） | **电机内部位置 PD** | 关节定位 |
| 增量角度 | 同上 | 同上 | 多圈相对运动 |

MIT 控制律：`τ = Kp·(p_des − p) + Kd·(v_des − v) + t_ff`（无积分，静差靠 `t_ff` 补）。

### 9.2 初始化与配置（全部在 HW_Config.c）

```c
// 统一入口，main 只调这一句
Mo_Init();          // = Mo_Init_DJI() + Mo_Init_DM() + 通道使能
```

达妙电机用表驱动，加电机只是加一行：

```c
static const DM_Motor_Cfg_t dm_cfg[] = {
    { .can_ch = 1, .idx = 0,              // CAN1，槽位0 → CAN ID=1，反馈 ID=0x11
      .type = J4030, .enc = DM_ENC_2E,
      .p_max = 12.5f, .v_max = 20.0f, .t_max = 28.0f,   // 必须与调试助手一致
      .v_lim_dps = 240.0f, .mit_kp = 80.0f, .mit_kd = 3.0f,
      .grav_k = 0.0f, .grav_phase = 0.0f,
      .mode = DJI_CTRL_POSITION, .target = 90.0f },     // 上电转到 90°
};
```

### 9.3 API

```c
// 【配置函数】
void DM_Mo_Setting_Init(can_ch, idx, type, enc, p_max, v_max, t_max); // 协议+量程
void DM_Motion_Config(id, v_lim_dps, kp_mit, kd_mit, can_ch);         // 斜坡速度 + 内部 PD
void DM_Set_Gravity_FF(id, k, phase_deg, can_ch);                     // 重力前馈

// 运行时（随时可调，幂等）
void DM_PID_Speed_Control(id, rpm, can_ch);          // 定速
void DM_PID_Position_Control(id, angle_deg, can_ch); // 绝对角度（0~360°，最短路径）
void DM_PID_Increment_Control(id, delta_deg, can_ch);// 增量角度（可多圈）
void DM_Set_CtrlMode(id, mode, can_ch);
float DM_Pos_MultiTurn_Deg(id, can_ch);              // 读多圈累计角度
```

### 9.4 轨迹与保持的关系

```c
// 每 20ms 在 TIM17 里执行：
step = v_lim_dps * 0.02f;                 // 本周最多走多少度
ref += clamp(target - ref, ±step);        // 目标做成斜坡
发 MIT(ref, v_ref, kp_mit, kd_mit, t_ff); // 位置保持交给电机
```

- MCU 只负责「让目标点平滑移动」，不承担保持。
- 位置保持由电机内部 kHz 级 PD 完成，**停止抖动远小于外部 50Hz 环**（实测稳态 SD = 0.00°）。

### 9.5 实测（DM-J4030 / CAN1 / 1 Mbps）

| 项 | 结果 |
|----|------|
| 定角度到位误差 | **0.1°** |
| 稳态标准差 | **0.00°** |
| 超调 | < 1° |
| 最大转速 | 5.6 rad/s（斜坡设 240°/s） |

### 9.6 达妙注意事项

- `p_max / v_max / t_max` 必须与调试助手「控制幅值」一致，否则反馈反算和 MIT 打包整体成比例偏。
- MIT 的 `p_des` 只能表示 **±p_max**（12.5 rad ≈ ±716°），**单次增量超过约 716° 会被截断**；多圈要分多次增量。
- 电机靠「收到帧才回反馈」，未收到反馈时固件每 500ms 自动补发一次使能帧，避免上电丢帧后死锁。
- 同一台电机不要同时开 MCU 侧 PID 和 `kp_mit/kd_mit`，两套环会互相打架。
- 达妙出厂是经典 CAN 1 Mbps 标准帧，总线配置用 `bsp_can_switch(&hfdcanX, CAN_CLASS, CAN_BR_1M)`。
