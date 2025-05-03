// 雪豹测量系统 - 数据拟合
#ifndef __CURVE_FITTING_H
#define __CURVE_FITTING_H

#include <stdint.h>

// --- 线性拟合 (y = a + b*x) ---

// 线性拟合结果结构体
typedef struct
{
    float a;           // 截距
    float b;           // 斜率
    float r_squared;   // 决定系数 (R^2)，表示拟合优度
    float std_error_a; // 截距标准误差
    float std_error_b; // 斜率标准误差
} LinearFitResult;

// 线性最小二乘拟合
// 返回0表示成功，-1表示失败
int Linear_Fit(const float *x_values, const float *y_values, uint16_t num_points, LinearFitResult *result);

// 使用线性拟合结果计算预测值
float Linear_Predict(const LinearFitResult *fit, float x);

// --- 多项式拟合 (y = a_0 + a_1*x + a_2*x^2 + ... + a_n*x^n) ---

// 多项式拟合结果结构体
typedef struct
{
    float *coefficients; // 多项式系数，从低阶到高阶 [a_0, a_1, ..., a_n]
    uint16_t degree;     // 多项式阶数 (n)
    float r_squared;     // 决定系数 (R^2)
} PolynomialFitResult;

// 多项式最小二乘拟合
// work_buffer: 工作缓冲区，大小至少为 (degree+1)*(degree+2) + (degree+1)*(degree+1) + 2*(degree+1) 个 float
// 返回0表示成功，-1表示失败
int Polynomial_Fit(const float *x_values, const float *y_values, uint16_t num_points,
                   uint16_t degree, PolynomialFitResult *result, float *coefficients, float *work_buffer);

// 使用多项式拟合结果计算预测值
float Polynomial_Predict(const PolynomialFitResult *fit, float x);

// --- 指数拟合 (y = a * exp(b*x)) ---
// 通过取对数转换为线性问题 ln(y) = ln(a) + b*x

// 指数拟合结果结构体
typedef struct
{
    float a;         // 系数a
    float b;         // 指数系数b
    float r_squared; // 决定系数 (R^2)
} ExponentialFitResult;

// 指数最小二乘拟合
// 返回0表示成功，-1表示失败
int Exponential_Fit(const float *x_values, const float *y_values, uint16_t num_points, ExponentialFitResult *result);

// 使用指数拟合结果计算预测值
float Exponential_Predict(const ExponentialFitResult *fit, float x);

#endif // __CURVE_FITTING_H