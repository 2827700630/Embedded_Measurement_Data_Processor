// 雪豹测量系统 - 数据插值实现
#include "xuebao_measure_h/interpolation.h"
#include <stddef.h> // For NULL
#include <math.h>   // For mathematical operations

/**
 * @brief 线性插值
 * @param x 要插值的x坐标
 * @param x1 已知点1的x坐标
 * @param y1 已知点1的y坐标
 * @param x2 已知点2的x坐标
 * @param y2 已知点2的y坐标
 * @return 插值结果
 */
float Linear_Interpolate(float x, float x1, float y1, float x2, float y2) {
    // 检查输入是否有效
    if (x1 == x2) {
        return (y1 + y2) / 2.0f; // 避免除以零，返回平均值
    }
    
    // 线性插值公式: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

/**
 * @brief 分段线性插值
 * @param x 要插值的x坐标
 * @param x_values 已知点的x坐标数组 (必须按升序排列)
 * @param y_values 已知点的y坐标数组
 * @param num_points 已知点的数量
 * @return 插值结果
 */
float Piecewise_Linear_Interpolate(float x, const float *x_values, const float *y_values, uint16_t num_points) {
    if (x_values == NULL || y_values == NULL || num_points < 2) {
        return 0.0f; // 参数错误
    }
    
    // 处理超出范围的情况
    if (x <= x_values[0]) {
        return y_values[0]; // 小于最小x值，返回第一个y值
    }
    if (x >= x_values[num_points - 1]) {
        return y_values[num_points - 1]; // 大于最大x值，返回最后一个y值
    }
    
    // 找到x所在的区间 [x_values[i], x_values[i+1]]
    uint16_t i;
    for (i = 0; i < num_points - 1; i++) {
        if (x >= x_values[i] && x <= x_values[i + 1]) {
            break;
        }
    }
    
    // 在找到的区间内进行线性插值
    return Linear_Interpolate(x, x_values[i], y_values[i], x_values[i + 1], y_values[i + 1]);
}

/**
 * @brief 拉格朗日插值
 * @param x 要插值的x坐标
 * @param x_values 已知点的x坐标数组
 * @param y_values 已知点的y坐标数组
 * @param num_points 已知点的数量
 * @return 插值结果
 */
float Lagrange_Interpolate(float x, const float *x_values, const float *y_values, uint16_t num_points) {
    if (x_values == NULL || y_values == NULL || num_points < 2) {
        return 0.0f; // 参数错误
    }
    
    float result = 0.0f;
    
    // 拉格朗日插值公式: p(x) = sum(y_i * L_i(x))
    // 其中 L_i(x) = product((x - x_j) / (x_i - x_j)) for j != i
    for (uint16_t i = 0; i < num_points; i++) {
        float term = y_values[i];
        for (uint16_t j = 0; j < num_points; j++) {
            if (j != i) {
                if (x_values[i] == x_values[j]) {
                    // 避免除以零
                    continue;
                }
                term *= (x - x_values[j]) / (x_values[i] - x_values[j]);
            }
        }
        result += term;
    }
    
    return result;
}

/**
 * @brief 初始化三次样条插值
 * @param spline 指向样条结构体的指针
 * @param x_values 已知点的x坐标数组 (必须按升序排列)
 * @param y_values 已知点的y坐标数组
 * @param num_points 已知点的数量
 * @param work_buffer 工作缓冲区，大小必须至少为 (4*n + n-1) 个float
 * @return 0表示成功，-1表示失败
 */
int CubicSpline_Init(CubicSpline *spline, const float *x_values, const float *y_values, 
                   uint16_t num_points, float *work_buffer) {
    if (spline == NULL || x_values == NULL || y_values == NULL || num_points < 3 || work_buffer == NULL) {
        return -1; // 参数错误
    }
    
    uint16_t n = num_points - 1; // 区间数量
    
    // 分配工作缓冲区
    spline->a = work_buffer;            // n+1个元素
    spline->b = spline->a + (n+1);      // n个元素
    spline->c = spline->b + n;          // n+1个元素
    spline->d = spline->c + (n+1);      // n个元素
    spline->h = spline->d + n;          // n个元素
    spline->n = n;
    
    // 初始化系数a为y值
    for (uint16_t i = 0; i <= n; i++) {
        spline->a[i] = y_values[i];
    }
    
    // 计算步长h
    for (uint16_t i = 0; i < n; i++) {
        spline->h[i] = x_values[i + 1] - x_values[i];
        if (spline->h[i] <= 0) {
            // x值必须严格递增
            return -1;
        }
    }
    
    // 三对角矩阵求解用的辅助数组
    float *u = spline->b; // 复用b的内存
    
    // 自然边界条件: 边界处的二阶导数为0
    spline->c[0] = 0;
    spline->c[n] = 0;
    
    // 构造三对角矩阵方程
    for (uint16_t i = 1; i < n; i++) {
        spline->c[i] = 3.0f * ((spline->a[i+1] - spline->a[i]) / spline->h[i] - 
                              (spline->a[i] - spline->a[i-1]) / spline->h[i-1]);
    }
    
    // 使用追赶法求解三对角矩阵方程
    u[0] = 0;
    for (uint16_t i = 1; i < n; i++) {
        float p = 2.0f * (x_values[i+1] - x_values[i-1]) - spline->h[i-1] * u[i-1];
        if (p == 0) {
            return -1; // 矩阵奇异
        }
        u[i] = spline->h[i] / p;
        spline->c[i] = (spline->c[i] - spline->h[i-1] * spline->c[i-1] / p);
    }
    
    // 回代求解
    for (uint16_t i = n-1; i > 0; i--) {
        spline->c[i] = spline->c[i] - u[i] * spline->c[i+1];
    }
    
    // 计算其他系数b和d
    for (uint16_t i = 0; i < n; i++) {
        spline->d[i] = (spline->c[i+1] - spline->c[i]) / (3.0f * spline->h[i]);
        spline->b[i] = (spline->a[i+1] - spline->a[i]) / spline->h[i] - 
                      spline->h[i] * (2.0f * spline->c[i] + spline->c[i+1]) / 3.0f;
    }
    
    return 0; // 成功
}

/**
 * @brief 使用三次样条插值计算值
 * @param spline 指向样条结构体的指针
 * @param x_values 已知点的x坐标数组
 * @param x 要插值的x坐标
 * @return 插值结果
 */
float CubicSpline_Interpolate(const CubicSpline *spline, const float *x_values, float x) {
    if (spline == NULL || x_values == NULL || spline->n < 1) {
        return 0.0f; // 参数错误
    }
    
    // 处理超出范围的情况
    if (x <= x_values[0]) {
        return spline->a[0]; // 小于最小x值，返回第一个y值
    }
    if (x >= x_values[spline->n]) {
        return spline->a[spline->n]; // 大于最大x值，返回最后一个y值
    }
    
    // 二分查找找到x所在的区间
    uint16_t low = 0;
    uint16_t high = spline->n;
    uint16_t mid;
    
    while (low < high - 1) {
        mid = (low + high) / 2;
        if (x_values[mid] <= x) {
            low = mid;
        } else {
            high = mid;
        }
    }
    
    uint16_t i = low;
    float dx = x - x_values[i];
    
    // 使用三次样条公式计算插值
    // S_i(x) = a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3
    return spline->a[i] + 
           spline->b[i] * dx + 
           spline->c[i] * dx * dx + 
           spline->d[i] * dx * dx * dx;
}