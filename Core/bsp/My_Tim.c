//
// Created by Ken on 2026/11/22.
//

#include "My_Tim.h"
#include "BMI088driver.h"

/* IMU 姿态数据（全局，其它模块通过 extern 读取） */
IMU_Attitude_t imu = {0};

/* ==================== 定时器启动 ==================== */
void My_Tim_Init(void)
{
    /* TIM14：1ms（1kHz）周期定时器
     * PSC=239 / ARR=999 → 240MHz / 240 / 1000 = 1kHz，NVIC 已由 CubeMX 配好 */
    HAL_TIM_Base_Start_IT(&htim14);

    /* 四元数初始化为单位四元数（姿态角 = 0） */
    IMU_Quat_Init(&imu.quat);

    /* 以后加更多定时器在这里接着 start，例如：
     * HAL_TIM_Base_Start_IT(&htim2);
     */
}

/* ==================== 定时器周期回调 ====================
 * 所有定时器的溢出中断都会进这里，按 htim->Instance 分发。
 * 注意：这个函数全局只能有一份，以后所有定时器的周期任务都写在这里。 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t cnt_1ms = 0;   // 1ms 计数，做软件分频用

    if (htim->Instance == TIM14)
    {
        /* 软件分频：1ms → 100ms（10Hz） */
        if (++cnt_1ms >= 100)
        {
            cnt_1ms = 0;

            /* --- 100ms 周期任务 --- */
            float gyro[3], accel[3], temp;

            BMI088_read(gyro, accel, &temp);

            IMU_Quat_Update(&imu.quat, gyro[0], gyro[1], gyro[2],
                            accel[0], accel[1], accel[2]);
            IMU_Quat_ToEuler(&imu.quat, &imu.angle);
        }
    }
    /* 以后加更多定时器：
     * else if (htim->Instance == TIM2)
     * {
     *     // TIM2 的周期任务
     * }
     */
}
