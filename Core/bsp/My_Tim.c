//
// Created by Ken on 2026/11/22.
//

#include "My_Tim.h"
#include "BMI088driver.h"

/* IMU 姿态数据（全局，其它模块通过 extern 读取） */
IMU_Attitude_t imu = {0};

/* ==================== 陀螺仪零偏 + 温度补偿 ==================== */
static float gyro_bias[3] = {0.0f};    // 静止时测得的零偏 (rad/s)
static float calib_temp = 0.0f;        // 标定时的温度 (°C)

/* 温度系数：陀螺仪零偏随温度的变化率 (rad/s 每 °C)
 * BMI088 典型约 0.015 °/s/K，换算成 rad/s/°C 后约 2.6e-4。
 * 这个值最好自己标定（不同传感器有差异），先给典型值。 */
#define GYRO_TEMP_COEF  (0.015f * 0.017453293f)

/* 上电静止时调用一次：采集 N 个样本取平均，得到零偏和参考温度 */
void IMU_Gyro_Calibrate(int samples)
{
    float gyro[3], accel[3], temp;
    float sum[3] = {0.0f};
    float tsum = 0.0f;

    for (int i = 0; i < samples; i++)
    {
        BMI088_read(gyro, accel, &temp);
        sum[0] += gyro[0];
        sum[1] += gyro[1];
        sum[2] += gyro[2];
        tsum += temp;
        HAL_Delay(1);
    }

    gyro_bias[0] = sum[0] / samples;
    gyro_bias[1] = sum[1] / samples;
    gyro_bias[2] = sum[2] / samples;
    calib_temp   = tsum / samples;
}

/* 应用零偏 + 温度漂移补偿 */
static void Gyro_Compensate(float gyro[3], float temp)
{
    float dt = temp - calib_temp;   // 相对标定温度的变化量

    gyro[0] -= gyro_bias[0] + GYRO_TEMP_COEF * dt;
    gyro[1] -= gyro_bias[1] + GYRO_TEMP_COEF * dt;
    gyro[2] -= gyro_bias[2] + GYRO_TEMP_COEF * dt;
}

/* ==================== 定时器启动 ==================== */
void My_Tim_Init(void)
{
    /* 上电静止标定（约 0.5s，板子必须保持完全静止！） */
    IMU_Gyro_Calibrate(500);

    /* 四元数初始化为单位四元数（姿态角 = 0） */
    IMU_Quat_Init(&imu.quat);

    /* TIM14：1ms（1kHz）周期定时器
     * PSC=239 / ARR=999 → 240MHz / 240 / 1000 = 1kHz，NVIC 已由 CubeMX 配好 */
    HAL_TIM_Base_Start_IT(&htim14);

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

            /* 零偏 + 温度补偿 */
            Gyro_Compensate(gyro, temp);

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
