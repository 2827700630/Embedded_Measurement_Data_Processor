// 雪豹测量系统 - 数据插值
#ifndef __INTERPOLATION_H
#define __INTERPOLATION_H

#include <stdint.h>

// --- 线性插值 ---
// 在已知的两点之间进行线性插值
// y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
float Linear_Interpolate(float x, float x1, float y1, float x2, float y2);

// --- 分段线性插值 ---
// 使用多个数据点进行分段线性插值
float Piecewise_Linear_Interpolate(float x, const float *x_values, const float *y_values, uint16_t num_points);

// --- 拉格朗日插值 ---
// 使用拉格朗日多项式进行插值（适用于较少数据点）
float Lagrange_Interpolate(float x, const float *x_values, const float *y_values, uint16_t num_points);

// --- 三次样条插值 ---
// 这是一个简化版的三次样条插值实现，使用自然样条（边界二阶导数为0）
// 注意：这个函数需要一个临时缓冲区来存储计算的中间结果
typedef struct
{
    float *a;   // 系数 a (实际上是 y 值)
    float *b;   // 系数 b
    float *c;   // 系数 c
    float *d;   // 系数 d
    float *h;   // 步长 h_i = x_{i+1} - x_i
    uint16_t n; // 区间数量（n = 数据点数量 - 1）
} CubicSpline;

// 初始化三次样条插值
// 返回0表示成功，-1表示失败
int CubicSpline_Init(CubicSpline *spline, const float *x_values, const float *y_values,
                     uint16_t num_points, float *work_buffer);

// 使用三次样条插值计算值
// 如果 x 超出范围，则返回最近端点的值
float CubicSpline_Interpolate(const CubicSpline *spline, const float *x_values, float x);

#endif // __INTERPOLATION_H