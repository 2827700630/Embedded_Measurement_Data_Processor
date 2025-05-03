// 雪豹测量系统 - 卡尔曼滤波器
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include <stdint.h>

// --- 一维卡尔曼滤波器 ---
// 这是一个简化版本的卡尔曼滤波器，适用于一维数据
typedef struct
{
    float x;             // 状态估计值
    float P;             // 状态估计协方差
    float Q;             // 过程噪声协方差
    float R;             // 测量噪声协方差
    float K;             // 卡尔曼增益
    uint8_t initialized; // 是否已初始化
} KalmanFilter1D;

// 初始化一维卡尔曼滤波器
void KalmanFilter1D_Init(KalmanFilter1D *filter, float initial_x, float initial_P, float Q, float R);

// 更新滤波器并获取滤波后的值
float KalmanFilter1D_Update(KalmanFilter1D *filter, float measurement);

// --- 多维卡尔曼滤波器 (2D案例) ---
// 这是一个二维卡尔曼滤波器，适用于位置-速度模型
// 状态向量 x = [position, velocity]^T
typedef struct
{
    float x[2];          // 状态估计值 [位置, 速度]
    float P[2][2];       // 状态估计协方差
    float F[2][2];       // 状态转移矩阵
    float H[2];          // 测量矩阵 (这里简化为仅测量位置)
    float Q[2][2];       // 过程噪声协方差
    float R;             // 测量噪声协方差 (这里简化为标量)
    float K[2];          // 卡尔曼增益
    uint8_t initialized; // 是否已初始化
    float dt;            // 时间步长
} KalmanFilter2D;

// 初始化二维卡尔曼滤波器 (位置-速度模型)
void KalmanFilter2D_Init(KalmanFilter2D *filter, float initial_pos, float initial_vel,
                         float P_pos, float P_vel, float Q_pos, float Q_vel,
                         float R, float dt);

// 更新二维卡尔曼滤波器，返回估计的位置
float KalmanFilter2D_UpdatePosition(KalmanFilter2D *filter, float position_measurement);

// 获取当前估计的速度
float KalmanFilter2D_GetVelocity(KalmanFilter2D *filter);

// 更新卡尔曼滤波器的dt(时间步长)值，用于处理不等间隔的采样
void KalmanFilter2D_SetDt(KalmanFilter2D *filter, float new_dt);

#endif // __KALMAN_FILTER_H