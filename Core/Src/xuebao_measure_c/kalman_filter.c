// 雪豹测量系统 - 卡尔曼滤波器实现
#include "xuebao_measure_h/kalman_filter.h"
#include <stddef.h> // For NULL
#include <math.h>   // For mathematical operations

/**
 * @brief 初始化一维卡尔曼滤波器
 * @param filter 指向滤波器结构体的指针
 * @param initial_x 初始状态估计值
 * @param initial_P 初始状态估计协方差
 * @param Q 过程噪声协方差
 * @param R 测量噪声协方差
 */
void KalmanFilter1D_Init(KalmanFilter1D *filter, float initial_x, float initial_P, float Q, float R) {
    if (filter == NULL) {
        return;
    }
    
    filter->x = initial_x;
    filter->P = initial_P;
    filter->Q = (Q > 0) ? Q : 0.01f; // 确保过程噪声为正值
    filter->R = (R > 0) ? R : 0.1f;  // 确保测量噪声为正值
    filter->K = 0.0f;
    filter->initialized = 1;
}

/**
 * @brief 更新一维卡尔曼滤波器
 * @param filter 指向滤波器结构体的指针
 * @param measurement 测量值
 * @return 滤波后的估计值
 */
float KalmanFilter1D_Update(KalmanFilter1D *filter, float measurement) {
    if (filter == NULL || !filter->initialized) {
        return measurement; // 如果滤波器未初始化，直接返回测量值
    }
    
    // 预测步骤（假设状态转移模型为恒等映射，即x(k) = x(k-1)）
    // 预测状态：x_pred = F * x
    // 这里 F = 1，所以 x_pred = x
    
    // 预测协方差：P_pred = F * P * F^T + Q
    // 由于 F = 1，所以 P_pred = P + Q
    filter->P = filter->P + filter->Q;
    
    // 更新步骤
    // 计算卡尔曼增益：K = P_pred * H^T * (H * P_pred * H^T + R)^-1
    // 由于 H = 1 (我们直接观察状态), 所以 K = P_pred / (P_pred + R)
    filter->K = filter->P / (filter->P + filter->R);
    
    // 更新状态估计：x = x_pred + K * (z - H * x_pred)
    // 由于 H = 1，所以 x = x_pred + K * (z - x_pred) = (1 - K) * x_pred + K * z
    filter->x = filter->x + filter->K * (measurement - filter->x);
    
    // 更新状态估计协方差：P = (I - K * H) * P_pred
    // 由于 H = 1，所以 P = (1 - K) * P_pred
    filter->P = (1.0f - filter->K) * filter->P;
    
    return filter->x;
}

/**
 * @brief 初始化二维卡尔曼滤波器 (位置-速度模型)
 * @param filter 指向滤波器结构体的指针
 * @param initial_pos 初始位置估计
 * @param initial_vel 初始速度估计
 * @param P_pos 初始位置估计的协方差
 * @param P_vel 初始速度估计的协方差
 * @param Q_pos 位置过程噪声协方差
 * @param Q_vel 速度过程噪声协方差
 * @param R 测量噪声协方差
 * @param dt 时间步长
 */
void KalmanFilter2D_Init(KalmanFilter2D *filter, float initial_pos, float initial_vel, 
                         float P_pos, float P_vel, float Q_pos, float Q_vel, 
                         float R, float dt) {
    if (filter == NULL) {
        return;
    }
    
    // 初始化状态向量
    filter->x[0] = initial_pos;
    filter->x[1] = initial_vel;
    
    // 初始化协方差矩阵
    filter->P[0][0] = P_pos;
    filter->P[0][1] = 0.0f;
    filter->P[1][0] = 0.0f;
    filter->P[1][1] = P_vel;
    
    // 设置过程噪声协方差
    filter->Q[0][0] = Q_pos;
    filter->Q[0][1] = 0.0f;
    filter->Q[1][0] = 0.0f;
    filter->Q[1][1] = Q_vel;
    
    // 设置测量噪声协方差
    filter->R = (R > 0) ? R : 0.1f;
    
    // 设置时间步长
    filter->dt = (dt > 0) ? dt : 0.1f;
    
    // 设置状态转移矩阵 F = [1, dt; 0, 1]
    filter->F[0][0] = 1.0f;
    filter->F[0][1] = filter->dt;
    filter->F[1][0] = 0.0f;
    filter->F[1][1] = 1.0f;
    
    // 设置测量矩阵 H = [1, 0] (只测量位置)
    filter->H[0] = 1.0f;
    filter->H[1] = 0.0f;
    
    // 初始化卡尔曼增益
    filter->K[0] = 0.0f;
    filter->K[1] = 0.0f;
    
    filter->initialized = 1;
}

/**
 * @brief 更新二维卡尔曼滤波器
 * @param filter 指向滤波器结构体的指针
 * @param position_measurement 位置测量值
 * @return 滤波后的位置估计值
 */
float KalmanFilter2D_UpdatePosition(KalmanFilter2D *filter, float position_measurement) {
    if (filter == NULL || !filter->initialized) {
        return position_measurement;
    }
    
    // --- 预测步骤 ---
    // 预测状态：x_pred = F * x
    float x_pred[2];
    x_pred[0] = filter->F[0][0] * filter->x[0] + filter->F[0][1] * filter->x[1];
    x_pred[1] = filter->F[1][0] * filter->x[0] + filter->F[1][1] * filter->x[1];
    
    // 预测协方差：P_pred = F * P * F^T + Q
    float P_pred[2][2];
    // P_pred = F * P
    float FP[2][2];
    FP[0][0] = filter->F[0][0] * filter->P[0][0] + filter->F[0][1] * filter->P[1][0];
    FP[0][1] = filter->F[0][0] * filter->P[0][1] + filter->F[0][1] * filter->P[1][1];
    FP[1][0] = filter->F[1][0] * filter->P[0][0] + filter->F[1][1] * filter->P[1][0];
    FP[1][1] = filter->F[1][0] * filter->P[0][1] + filter->F[1][1] * filter->P[1][1];
    
    // P_pred = FP * F^T + Q
    P_pred[0][0] = FP[0][0] * filter->F[0][0] + FP[0][1] * filter->F[1][0] + filter->Q[0][0];
    P_pred[0][1] = FP[0][0] * filter->F[0][1] + FP[0][1] * filter->F[1][1] + filter->Q[0][1];
    P_pred[1][0] = FP[1][0] * filter->F[0][0] + FP[1][1] * filter->F[1][0] + filter->Q[1][0];
    P_pred[1][1] = FP[1][0] * filter->F[0][1] + FP[1][1] * filter->F[1][1] + filter->Q[1][1];
    
    // --- 更新步骤 ---
    // 计算测量预测：z_pred = H * x_pred
    float z_pred = filter->H[0] * x_pred[0] + filter->H[1] * x_pred[1];
    
    // 计算测量残差(innovation): y = z - z_pred
    float y = position_measurement - z_pred;
    
    // 计算残差协方差: S = H * P_pred * H^T + R
    float PHt[2];  // P_pred * H^T
    PHt[0] = P_pred[0][0] * filter->H[0] + P_pred[0][1] * filter->H[1];
    PHt[1] = P_pred[1][0] * filter->H[0] + P_pred[1][1] * filter->H[1];
    
    float S = filter->H[0] * PHt[0] + filter->H[1] * PHt[1] + filter->R;
    
    // 计算卡尔曼增益: K = P_pred * H^T * S^-1
    filter->K[0] = PHt[0] / S;
    filter->K[1] = PHt[1] / S;
    
    // 更新状态估计: x = x_pred + K * y
    filter->x[0] = x_pred[0] + filter->K[0] * y;
    filter->x[1] = x_pred[1] + filter->K[1] * y;
    
    // 更新状态协方差: P = (I - K * H) * P_pred
    filter->P[0][0] = (1.0f - filter->K[0] * filter->H[0]) * P_pred[0][0] - filter->K[0] * filter->H[1] * P_pred[1][0];
    filter->P[0][1] = (1.0f - filter->K[0] * filter->H[0]) * P_pred[0][1] - filter->K[0] * filter->H[1] * P_pred[1][1];
    filter->P[1][0] = -filter->K[1] * filter->H[0] * P_pred[0][0] + (1.0f - filter->K[1] * filter->H[1]) * P_pred[1][0];
    filter->P[1][1] = -filter->K[1] * filter->H[0] * P_pred[0][1] + (1.0f - filter->K[1] * filter->H[1]) * P_pred[1][1];
    
    // 返回更新后的位置估计
    return filter->x[0];
}

/**
 * @brief 获取二维卡尔曼滤波器当前估计的速度
 * @param filter 指向滤波器结构体的指针
 * @return 当前速度估计值
 */
float KalmanFilter2D_GetVelocity(KalmanFilter2D *filter) {
    if (filter == NULL || !filter->initialized) {
        return 0.0f;
    }
    return filter->x[1];
}

/**
 * @brief 更新卡尔曼滤波器的dt(时间步长)值
 * @param filter 指向滤波器结构体的指针
 * @param new_dt 新的时间步长
 */
void KalmanFilter2D_SetDt(KalmanFilter2D *filter, float new_dt) {
    if (filter == NULL || new_dt <= 0) {
        return;
    }
    
    filter->dt = new_dt;
    
    // 更新状态转移矩阵 F = [1, dt; 0, 1]
    filter->F[0][1] = filter->dt;
}