// 雪豹测量系统 - 不确定度计算实现
#include "xuebao_measure_h/uncertainty.h"
#include <stddef.h> // For NULL
#include <math.h>   // For mathematical operations
#include <stdlib.h> // For rand(), qsort()
#include <string.h> // For memcpy

/**
 * @brief 计算类型A不确定度 (基于统计分析)
 * @param data 测量数据数组
 * @param size 数据点数量
 * @param coverage_factor 包含因子k (通常k=2表示约95%置信水平)
 * @param result 不确定度计算结果
 */
void Calculate_TypeA_Uncertainty(const float *data, uint16_t size, float coverage_factor, TypeAUncertainty *result) {
    if (data == NULL || size < 2 || result == NULL) {
        return;
    }
    
    // 计算平均值
    float sum = 0.0f;
    for (uint16_t i = 0; i < size; i++) {
        sum += data[i];
    }
    float mean = sum / (float)size;
    
    // 计算标准差 (样本标准差，分母为n-1)
    float sum_squared_diff = 0.0f;
    for (uint16_t i = 0; i < size; i++) {
        float diff = data[i] - mean;
        sum_squared_diff += diff * diff;
    }
    float variance = sum_squared_diff / (float)(size - 1);
    float std_deviation = sqrtf(variance);
    
    // 计算标准不确定度 u_A = s / sqrt(n)
    float std_uncertainty = std_deviation / sqrtf((float)size);
    
    // 计算扩展不确定度 U = k * u_A
    float expanded_uncertainty = coverage_factor * std_uncertainty;
    
    // 估计置信水平 (简化近似，假设正态分布)
    float confidence_level;
    if (coverage_factor <= 1.0f) {
        confidence_level = 0.68f; // 约68%
    } else if (coverage_factor <= 2.0f) {
        confidence_level = 0.95f; // 约95%
    } else if (coverage_factor <= 3.0f) {
        confidence_level = 0.997f; // 约99.7%
    } else {
        confidence_level = 0.9999f; // 约99.99%
    }
    
    // 填充结果
    result->mean = mean;
    result->std_deviation = std_deviation;
    result->std_uncertainty = std_uncertainty;
    result->expanded_uncertainty = expanded_uncertainty;
    result->sample_size = size;
    result->confidence_level = confidence_level;
}

/**
 * @brief 计算类型B不确定度 (基于非统计信息)
 * @param input 不确定度输入参数
 * @return 标准不确定度 u_B
 */
float Calculate_TypeB_Uncertainty(const TypeBInput *input) {
    if (input == NULL || input->half_width <= 0.0f) {
        return 0.0f;
    }
    
    float divisor;
    
    // 根据分布类型计算分母
    switch (input->distribution) {
        case RECTANGULAR:
            // 矩形分布 (均匀分布)
            divisor = sqrtf(3.0f);
            break;
        case TRIANGULAR:
            // 三角分布
            divisor = sqrtf(6.0f);
            break;
        case NORMAL:
            // 正态分布
            if (input->coverage_factor <= 0.0f) {
                divisor = 2.0f; // 默认使用k=2
            } else {
                divisor = input->coverage_factor;
            }
            break;
        case U_SHAPED:
            // U型分布
            divisor = sqrtf(2.0f);
            break;
        default:
            divisor = 1.0f;
            break;
    }
    
    // 计算标准不确定度 u_B = a / divisor
    return input->half_width / divisor;
}

// t分布临界值的近似公式 (对于自由度v和置信水平1-alpha)
static float t_critical_value(float dof, float confidence_level) {
    if (dof < 1.0f) {
        dof = 1.0f;
    }
    
    // 常见置信水平的近似计算
    // 基于t分布的特性和简化
    if (confidence_level >= 0.999f) {
        // 99.9% 置信水平
        return 3.0f + 6.0f / (dof + 2.0f);
    } else if (confidence_level >= 0.99f) {
        // 99% 置信水平
        return 2.6f + 3.0f / (dof + 2.0f);
    } else if (confidence_level >= 0.95f) {
        // 95% 置信水平
        return 2.0f + 1.0f / (dof + 2.0f);
    } else if (confidence_level >= 0.90f) {
        // 90% 置信水平
        return 1.7f + 0.6f / (dof + 2.0f);
    } else if (confidence_level >= 0.80f) {
        // 80% 置信水平
        return 1.3f + 0.3f / (dof + 2.0f);
    } else {
        // 默认68%的置信水平
        return 1.0f;
    }
}

/**
 * @brief 计算合成不确定度
 * @param u_values 各分量标准不确定度数组
 * @param dof_values 各分量自由度数组 (对于B类型，自由度可视为无穷大，使用一个较大的值如1e6)
 * @param sensitivity 灵敏系数数组 (c_i = ∂f/∂x_i)
 * @param size 不确定度分量的数量
 * @param confidence_level 期望的置信水平 (0.0-1.0)
 * @param result 合成不确定度结果
 */
void Calculate_Combined_Uncertainty(const float *u_values, const float *dof_values, 
                                  const float *sensitivity, uint16_t size, 
                                  float confidence_level, CombinedUncertainty *result) {
    if (u_values == NULL || dof_values == NULL || sensitivity == NULL || result == NULL || size == 0) {
        return;
    }
    
    float sum_squared_uncertainty = 0.0f;
    float sum_numerator = 0.0f;
    float sum_denominator = 0.0f;
    
    // 计算合成标准不确定度 u_c = sqrt(sum(c_i^2 * u_i^2))
    for (uint16_t i = 0; i < size; i++) {
        float term = sensitivity[i] * u_values[i];
        float squared_term = term * term;
        sum_squared_uncertainty += squared_term;
        
        // 为Welch-Satterthwaite公式累积分子和分母
        if (dof_values[i] > 0.0f) {
            sum_numerator += squared_term;
            sum_denominator += squared_term * squared_term / dof_values[i];
        }
    }
    
    float combined_std_uncertainty = sqrtf(sum_squared_uncertainty);
    
    // 使用Welch-Satterthwaite公式计算有效自由度
    float effective_dof;
    if (sum_denominator > 0.0f) {
        effective_dof = sum_numerator * sum_numerator / sum_denominator;
        if (effective_dof < 1.0f) {
            effective_dof = 1.0f;
        }
    } else {
        effective_dof = 1e6f; // 假设为无穷大
    }
    
    // 根据有效自由度和置信水平确定包含因子k
    float coverage_factor = t_critical_value(effective_dof, confidence_level);
    
    // 计算扩展不确定度 U = k * u_c
    float expanded_uncertainty = coverage_factor * combined_std_uncertainty;
    
    // 填充结果
    result->combined_std_uncertainty = combined_std_uncertainty;
    result->expanded_uncertainty = expanded_uncertainty;
    result->effective_dof = effective_dof;
    result->coverage_factor = coverage_factor;
    result->confidence_level = confidence_level;
}

// 用于qsort的比较函数
static int compare_float(const void *a, const void *b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

/**
 * @brief 使用蒙特卡洛方法计算不确定度
 * @note 此实现很简化，仅适用于小规模计算
 * @param inputs 输入参数的描述数组
 * @param size 输入参数数量
 * @param model 模型函数指针
 * @param num_simulations 模拟次数
 * @param rand_buffer 随机数缓冲区 (大小必须至少为 num_simulations 个float)
 * @param confidence_level 置信水平 (0.0-1.0)
 * @param result 结果输出
 * @return 0表示成功，-1表示失败
 */
int Calculate_MonteCarlo_Uncertainty(const MonteCarloInput *inputs, uint16_t size,
                                   ModelFunction model, uint16_t num_simulations,
                                   float *rand_buffer, float confidence_level,
                                   MonteCarloResult *result) {
    if (inputs == NULL || model == NULL || num_simulations < 100 || 
        rand_buffer == NULL || result == NULL || size == 0) {
        return -1;
    }
    
    float *simulation_inputs = rand_buffer;  // 复用缓冲区前一部分
    float *simulation_results = rand_buffer; // 最终结果将覆盖输入
    
    float sum = 0.0f;
    
    // 进行Monte Carlo模拟
    for (uint16_t i = 0; i < num_simulations; i++) {
        // 为每个输入生成随机值
        for (uint16_t j = 0; j < size; j++) {
            float random_value;
            
            // 根据指定分布生成随机数
            // 注：这是一个简化的实现，使用了rand()，实际应使用更好的随机数生成器
            switch (inputs[j].distribution) {
                case RECTANGULAR:
                    // 均匀分布在 [mean-std_dev, mean+std_dev] 范围内
                    random_value = inputs[j].mean - inputs[j].std_deviation + 
                                  2.0f * inputs[j].std_deviation * ((float)rand() / RAND_MAX);
                    break;
                
                case TRIANGULAR:
                    // 简化的三角分布模拟
                    {
                        float u1 = (float)rand() / RAND_MAX;
                        float u2 = (float)rand() / RAND_MAX;
                        random_value = inputs[j].mean + inputs[j].std_deviation * sqrtf(6.0f) * 
                                      (u1 + u2 - 1.0f) / 2.0f;
                    }
                    break;
                
                case NORMAL:
                    // 简化的正态分布模拟 (Box-Muller变换)
                    {
                        float u1 = (float)rand() / RAND_MAX;
                        float u2 = (float)rand() / RAND_MAX;
                        if (u1 < 1e-10f) u1 = 1e-10f; // 避免对0取对数
                        float z = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159f * u2);
                        random_value = inputs[j].mean + inputs[j].std_deviation * z;
                    }
                    break;
                
                case U_SHAPED:
                    // 简化的U型分布模拟
                    {
                        float u = (float)rand() / RAND_MAX;
                        float angle = 2.0f * 3.14159f * u;
                        random_value = inputs[j].mean + inputs[j].std_deviation * sqrtf(2.0f) * cosf(angle);
                    }
                    break;
                
                default:
                    random_value = inputs[j].mean;
                    break;
            }
            
            simulation_inputs[j] = random_value;
        }
        
        // 运行模型
        float result_value = model(simulation_inputs, size);
        simulation_results[i] = result_value;
        sum += result_value;
    }
    
    // 计算平均值
    float mean = sum / (float)num_simulations;
    
    // 计算标准差 (标准不确定度)
    float sum_squared_diff = 0.0f;
    for (uint16_t i = 0; i < num_simulations; i++) {
        float diff = simulation_results[i] - mean;
        sum_squared_diff += diff * diff;
    }
    float variance = sum_squared_diff / (float)(num_simulations - 1);
    float std_deviation = sqrtf(variance);
    
    // 对结果进行排序以便计算百分位数
    qsort(simulation_results, num_simulations, sizeof(float), compare_float);
    
    // 确定置信区间
    float alpha = 1.0f - confidence_level;
    uint16_t lower_index = (uint16_t)(0.5f * alpha * num_simulations);
    uint16_t upper_index = (uint16_t)((1.0f - 0.5f * alpha) * num_simulations);
    
    if (lower_index >= num_simulations) lower_index = num_simulations - 1;
    if (upper_index >= num_simulations) upper_index = num_simulations - 1;
    
    float lower_bound = simulation_results[lower_index];
    float upper_bound = simulation_results[upper_index];
    
    // 计算扩展不确定度 (用区间半宽度表示)
    float expanded_uncertainty = 0.5f * (upper_bound - lower_bound);
    
    // 填充结果
    result->mean = mean;
    result->std_deviation = std_deviation;
    result->lower_bound = lower_bound;
    result->upper_bound = upper_bound;
    result->expanded_uncertainty = expanded_uncertainty;
    
    return 0;
}