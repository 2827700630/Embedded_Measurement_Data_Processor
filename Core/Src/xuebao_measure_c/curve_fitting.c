// 雪豹测量系统 - 数据拟合实现
#include "xuebao_measure_h/curve_fitting.h"
#include <stddef.h> // For NULL
#include <math.h>   // For mathematical operations
#include <string.h> // For memset, memcpy

/**
 * @brief 线性最小二乘拟合 (y = a + b*x)
 * @param x_values x坐标数组
 * @param y_values y坐标数组
 * @param num_points 数据点数量
 * @param result 拟合结果
 * @return 0表示成功，-1表示失败
 */
int Linear_Fit(const float *x_values, const float *y_values, uint16_t num_points, LinearFitResult *result) {
    if (x_values == NULL || y_values == NULL || num_points < 2 || result == NULL) {
        return -1; // 参数错误
    }
    
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xx = 0.0f;
    float sum_xy = 0.0f;
    float x, y;
    
    // 计算各种和
    for (uint16_t i = 0; i < num_points; i++) {
        x = x_values[i];
        y = y_values[i];
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }
    
    // 计算斜率和截距
    float n = (float)num_points;
    float denominator = n * sum_xx - sum_x * sum_x;
    
    if (fabsf(denominator) < 1e-10f) {
        // 分母接近于零，无法计算
        return -1;
    }
    
    result->b = (n * sum_xy - sum_x * sum_y) / denominator;
    result->a = (sum_y - result->b * sum_x) / n;
    
    // 计算R^2和标准误差
    float sum_squared_residuals = 0.0f;
    float sum_squared_total = 0.0f;
    float mean_y = sum_y / n;
    float residual, predicted;
    
    for (uint16_t i = 0; i < num_points; i++) {
        predicted = result->a + result->b * x_values[i];
        residual = y_values[i] - predicted;
        sum_squared_residuals += residual * residual;
        sum_squared_total += (y_values[i] - mean_y) * (y_values[i] - mean_y);
    }
    
    if (fabsf(sum_squared_total) < 1e-10f) {
        result->r_squared = 1.0f; // 所有y值相同
    } else {
        result->r_squared = 1.0f - (sum_squared_residuals / sum_squared_total);
    }
    
    // 标准误差
    if (num_points > 2) {
        float s_squared = sum_squared_residuals / (n - 2);
        result->std_error_b = sqrtf(s_squared * n / denominator);
        result->std_error_a = result->std_error_b * sqrtf(sum_xx / n);
    } else {
        result->std_error_a = 0.0f;
        result->std_error_b = 0.0f;
    }
    
    return 0; // 成功
}

/**
 * @brief 使用线性拟合结果计算预测值
 * @param fit 拟合结果
 * @param x x坐标
 * @return 预测的y值
 */
float Linear_Predict(const LinearFitResult *fit, float x) {
    if (fit == NULL) {
        return 0.0f;
    }
    return fit->a + fit->b * x;
}

/**
 * @brief 多项式最小二乘拟合 (y = a_0 + a_1*x + ... + a_n*x^n)
 * @param x_values x坐标数组
 * @param y_values y坐标数组
 * @param num_points 数据点数量
 * @param degree 多项式阶数
 * @param result 拟合结果
 * @param coefficients 多项式系数数组，大小为 degree+1
 * @param work_buffer 工作缓冲区
 * @return 0表示成功，-1表示失败
 */
int Polynomial_Fit(const float *x_values, const float *y_values, uint16_t num_points, 
                 uint16_t degree, PolynomialFitResult *result, float *coefficients, float *work_buffer) {
    if (x_values == NULL || y_values == NULL || num_points <= degree || 
        result == NULL || coefficients == NULL || work_buffer == NULL) {
        return -1; // 参数错误
    }
    
    // 初始化结果
    result->coefficients = coefficients;
    result->degree = degree;
    
    // 分配工作缓冲区
    uint16_t n = degree + 1; // 多项式系数个数
    float *matrix = work_buffer;                // (n+1) * n 矩阵 (增广矩阵)
    float *temp_matrix = matrix + n * (n + 1);  // n * n 矩阵 (用于高斯消元)
    float *x_powers = temp_matrix + n * n;      // 2*n 数组 (用于存储x的幂)
    
    // 初始化为0
    memset(matrix, 0, n * (n + 1) * sizeof(float));
    
    // 计算多项式拟合的正规方程组
    for (uint16_t i = 0; i < num_points; i++) {
        float x = x_values[i];
        float y = y_values[i];
        
        // 计算x的幂: x^0, x^1, ..., x^(2*n-2)
        x_powers[0] = 1.0f;
        for (uint16_t j = 1; j < 2*n; j++) {
            x_powers[j] = x_powers[j-1] * x;
        }
        
        // 填充系数矩阵
        for (uint16_t j = 0; j < n; j++) {
            for (uint16_t k = 0; k < n; k++) {
                matrix[j * (n + 1) + k] += x_powers[j + k];
            }
            // 填充常数项
            matrix[j * (n + 1) + n] += y * x_powers[j];
        }
    }
    
    // 高斯消元法解方程组
    for (uint16_t i = 0; i < n; i++) {
        // 复制当前行到临时矩阵
        memcpy(temp_matrix + i * n, matrix + i * (n + 1), n * sizeof(float));
        
        // 找主元 (部分主元消去法)
        uint16_t max_row = i;
        float max_val = fabsf(temp_matrix[i * n + i]);
        
        for (uint16_t j = i + 1; j < n; j++) {
            float val = fabsf(temp_matrix[j * n + i]);
            if (val > max_val) {
                max_val = val;
                max_row = j;
            }
        }
        
        // 如果主元太小，认为矩阵奇异
        if (max_val < 1e-10f) {
            return -1;
        }
        
        // 交换行
        if (max_row != i) {
            for (uint16_t j = i; j < n; j++) {
                float temp = temp_matrix[i * n + j];
                temp_matrix[i * n + j] = temp_matrix[max_row * n + j];
                temp_matrix[max_row * n + j] = temp;
            }
            float temp = matrix[i * (n + 1) + n];
            matrix[i * (n + 1) + n] = matrix[max_row * (n + 1) + n];
            matrix[max_row * (n + 1) + n] = temp;
        }
        
        // 对当前行进行归一化
        float pivot = temp_matrix[i * n + i];
        for (uint16_t j = i; j < n; j++) {
            temp_matrix[i * n + j] /= pivot;
        }
        matrix[i * (n + 1) + n] /= pivot;
        
        // 消元
        for (uint16_t j = 0; j < n; j++) {
            if (j == i) continue;
            
            float factor = temp_matrix[j * n + i];
            for (uint16_t k = i; k < n; k++) {
                temp_matrix[j * n + k] -= factor * temp_matrix[i * n + k];
            }
            matrix[j * (n + 1) + n] -= factor * matrix[i * (n + 1) + n];
        }
    }
    
    // 提取结果
    for (uint16_t i = 0; i < n; i++) {
        coefficients[i] = matrix[i * (n + 1) + n];
    }
    
    // 计算R^2
    float sum_squared_residuals = 0.0f;
    float sum_squared_total = 0.0f;
    float mean_y = 0.0f;
    
    for (uint16_t i = 0; i < num_points; i++) {
        mean_y += y_values[i];
    }
    mean_y /= (float)num_points;
    
    for (uint16_t i = 0; i < num_points; i++) {
        float predicted = Polynomial_Predict(result, x_values[i]);
        float residual = y_values[i] - predicted;
        sum_squared_residuals += residual * residual;
        sum_squared_total += (y_values[i] - mean_y) * (y_values[i] - mean_y);
    }
    
    if (fabsf(sum_squared_total) < 1e-10f) {
        result->r_squared = 1.0f; // 所有y值相同
    } else {
        result->r_squared = 1.0f - (sum_squared_residuals / sum_squared_total);
    }
    
    return 0; // 成功
}

/**
 * @brief 使用多项式拟合结果计算预测值
 * @param fit 拟合结果
 * @param x x坐标
 * @return 预测的y值
 */
float Polynomial_Predict(const PolynomialFitResult *fit, float x) {
    if (fit == NULL || fit->coefficients == NULL) {
        return 0.0f;
    }
    
    float result = 0.0f;
    float x_power = 1.0f;
    
    for (uint16_t i = 0; i <= fit->degree; i++) {
        result += fit->coefficients[i] * x_power;
        x_power *= x;
    }
    
    return result;
}

/**
 * @brief 指数最小二乘拟合 (y = a * exp(b*x))
 * @param x_values x坐标数组
 * @param y_values y坐标数组 (必须全部为正值)
 * @param num_points 数据点数量
 * @param result 拟合结果
 * @return 0表示成功，-1表示失败
 */
int Exponential_Fit(const float *x_values, const float *y_values, uint16_t num_points, ExponentialFitResult *result) {
    if (x_values == NULL || y_values == NULL || num_points < 2 || result == NULL) {
        return -1; // 参数错误
    }
    
    // 检查y值是否全部为正
    for (uint16_t i = 0; i < num_points; i++) {
        if (y_values[i] <= 0.0f) {
            return -1; // 对数未定义
        }
    }
    
    float sum_x = 0.0f;
    float sum_lny = 0.0f;
    float sum_xx = 0.0f;
    float sum_xlny = 0.0f;
    float x, lny;
    
    // 对数转换后的线性拟合: ln(y) = ln(a) + b*x
    for (uint16_t i = 0; i < num_points; i++) {
        x = x_values[i];
        lny = logf(y_values[i]);
        
        sum_x += x;
        sum_lny += lny;
        sum_xx += x * x;
        sum_xlny += x * lny;
    }
    
    // 计算斜率和截距
    float n = (float)num_points;
    float denominator = n * sum_xx - sum_x * sum_x;
    
    if (fabsf(denominator) < 1e-10f) {
        // 分母接近于零，无法计算
        return -1;
    }
    
    float lna, b;
    b = (n * sum_xlny - sum_x * sum_lny) / denominator;
    lna = (sum_lny - b * sum_x) / n;
    
    // 转换回原始参数
    result->a = expf(lna);
    result->b = b;
    
    // 计算R^2
    float sum_squared_residuals = 0.0f;
    float sum_squared_total = 0.0f;
    float mean_lny = sum_lny / n;
    float residual, predicted_lny;
    
    for (uint16_t i = 0; i < num_points; i++) {
        predicted_lny = lna + b * x_values[i];
        residual = logf(y_values[i]) - predicted_lny;
        sum_squared_residuals += residual * residual;
        sum_squared_total += (logf(y_values[i]) - mean_lny) * (logf(y_values[i]) - mean_lny);
    }
    
    if (fabsf(sum_squared_total) < 1e-10f) {
        result->r_squared = 1.0f; // 所有ln(y)值相同
    } else {
        result->r_squared = 1.0f - (sum_squared_residuals / sum_squared_total);
    }
    
    return 0; // 成功
}

/**
 * @brief 使用指数拟合结果计算预测值
 * @param fit 拟合结果
 * @param x x坐标
 * @return 预测的y值
 */
float Exponential_Predict(const ExponentialFitResult *fit, float x) {
    if (fit == NULL) {
        return 0.0f;
    }
    return fit->a * expf(fit->b * x);
}