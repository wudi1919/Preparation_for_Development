#include "imu_quaternion.h"
#include <math.h>
#include "BMI088driver.h"

/* 采样频率（Hz）：必须与调用 IMU_Quat_Update 的实际周期一致。
 * 当前在 100ms（10Hz）分支里调用，所以填 10.0f */
#define IMU_SAMPLE_FREQ   10.0f

/* Mahony 互补滤波增益：Kp 比例、Ki 积分（2 * 增益） */
#define IMU_TWO_KP        (2.0f * 0.5f)
#define IMU_TWO_KI        (2.0f * 0.0f)

static float integralFBx = 0.0f;
static float integralFBy = 0.0f;
static float integralFBz = 0.0f;

/* ==================== 陀螺仪零偏 + 温度补偿 ==================== */

/* 全局 IMU 姿态数据（四元数 + 欧拉角） */
IMU_Attitude_t imu = {0};

static float gyro_bias[3] = {0.0f};    // 静止时测得的零偏 (rad/s)
static float calib_temp = 0.0f;        // 标定时的温度 (°C)

/* 温度系数：陀螺仪零偏随温度的变化率 (rad/s 每 °C)
 * BMI088 典型约 0.015 °/s/K，换算成 rad/s/°C 约 2.6e-4 */
#define GYRO_TEMP_COEF  (0.015f * 0.017453293f)

/* 上电静止时调用：采集 N 个样本取平均，得到零偏和参考温度 */
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
void IMU_Gyro_Compensate(float gyro[3], float temp)
{
    float dt = temp - calib_temp;

    gyro[0] -= gyro_bias[0] + GYRO_TEMP_COEF * dt;
    gyro[1] -= gyro_bias[1] + GYRO_TEMP_COEF * dt;
    gyro[2] -= gyro_bias[2] + GYRO_TEMP_COEF * dt;
}

/* IMU 初始化：零偏+温度标定 + 四元数初始化 */
void IMU_Init(void)
{
    IMU_Gyro_Calibrate(500);
    IMU_Quat_Init(&imu.quat);
}

void IMU_Quat_Init(Quaternion_t *q)
{
    q->q0 = 1.0f;
    q->q1 = 0.0f;
    q->q2 = 0.0f;
    q->q3 = 0.0f;

    integralFBx = 0.0f;
    integralFBy = 0.0f;
    integralFBz = 0.0f;
}

void IMU_Quat_Update(Quaternion_t *q, float gx, float gy, float gz,
                     float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* 加速度计测量有效时才做反馈修正，避免全零时归一化出 NaN */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        /* 归一化加速度测量 */
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* 当前四元数估计的重力方向 */
        halfvx = q->q1 * q->q3 - q->q0 * q->q2;
        halfvy = q->q0 * q->q1 + q->q2 * q->q3;
        halfvz = q->q0 * q->q0 - 0.5f + q->q3 * q->q3;

        /* 误差 = 测量重力 与 估计重力 的叉积 */
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        /* 积分反馈（Ki > 0 时启用） */
        if (IMU_TWO_KI > 0.0f)
        {
            integralFBx += IMU_TWO_KI * halfex * (1.0f / IMU_SAMPLE_FREQ);
            integralFBy += IMU_TWO_KI * halfey * (1.0f / IMU_SAMPLE_FREQ);
            integralFBz += IMU_TWO_KI * halfez * (1.0f / IMU_SAMPLE_FREQ);
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else
        {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        /* 比例反馈 */
        gx += IMU_TWO_KP * halfex;
        gy += IMU_TWO_KP * halfey;
        gz += IMU_TWO_KP * halfez;
    }

    /* 积分四元数变化率：0.5 * q 点乘 w * dt */
    gx *= (0.5f * (1.0f / IMU_SAMPLE_FREQ));
    gy *= (0.5f * (1.0f / IMU_SAMPLE_FREQ));
    gz *= (0.5f * (1.0f / IMU_SAMPLE_FREQ));
    qa = q->q0;
    qb = q->q1;
    qc = q->q2;
    q->q0 += (-qb * gx - qc * gy - q->q3 * gz);
    q->q1 += (qa * gx + qc * gz - q->q3 * gy);
    q->q2 += (qa * gy - qb * gz + q->q3 * gx);
    q->q3 += (qa * gz + qb * gy - qc * gx);

    /* 归一化四元数 */
    recipNorm = 1.0f / sqrtf(q->q0 * q->q0 + q->q1 * q->q1 +
                             q->q2 * q->q2 + q->q3 * q->q3);
    q->q0 *= recipNorm;
    q->q1 *= recipNorm;
    q->q2 *= recipNorm;
    q->q3 *= recipNorm;
}

void IMU_Quat_ToEuler(const Quaternion_t *q, EulerAngle_t *e)
{
    /* ZYX 顺序欧拉角（rad），与参考工程 imu_task.c 的 GetAngle 一致 */
    e->yaw   = atan2f(2.0f * (q->q0 * q->q3 + q->q1 * q->q2),
                     2.0f * (q->q0 * q->q0 + q->q1 * q->q1) - 1.0f);
    e->pitch = asinf(-2.0f * (q->q1 * q->q3 - q->q0 * q->q2));
    e->roll  = atan2f(2.0f * (q->q0 * q->q1 + q->q2 * q->q3),
                     2.0f * (q->q0 * q->q0 + q->q3 * q->q3) - 1.0f);
}
