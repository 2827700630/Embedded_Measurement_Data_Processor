// 雪豹测量系统 - 不确定度计算
#ifndef __UNCERTAINTY_H
#define __UNCERTAINTY_H

#include <stdint.h>

// --- 类型A不确定度 (基于统计分析的评定) ---

// 类型A不确定度结果结构体
typedef struct {
    float mean;              // 平均值
    float std_deviation;     // 标准差
    float std_uncertainty;   // 标准不确定度 u_A = s / sqrt(n)
    float expanded_uncertainty; // 扩展不确定度 U = k * u_A
    uint16_t sample_size;    // 样本大小
    float confidence_level;  // 置信水平
} TypeAUncertainty;

// 计算类型A不确定度
// data: 测量数据数组
// size: 数据点数量
// coverage_factor: 包含因子k (通常k=2表示约95%置信水平)
void Calculate_TypeA_Uncertainty(const float *data, uint16_t size, float coverage_factor, TypeAUncertainty *result);

// --- 类型B不确定度 (基于非统计信息的评定) ---

// 概率分布类型
typedef enum {
    RECTANGULAR,    // 矩形分布 (均匀分布) - 系数 = 1.0 / sqrt(3)
    TRIANGULAR,     // 三角分布 - 系数 = 1.0 / sqrt(6)
    NORMAL,         // 正态分布 - 系数 = 1.0 / coverage_factor (对于给定的置信水平)
    U_SHAPED        // U型分布 - 系数 = 1.0 / sqrt(2)
} DistributionType;

// 类型B不确定度输入结构体
typedef struct {
    float half_width;       // 半宽度 (a)
    DistributionType distribution; // 分布类型
    float coverage_factor;  // 仅用于NORMAL分布
} TypeBInput;

// 计算类型B不确定度 (对单个来源)
// 返回标准不确定度 u_B
float Calculate_TypeB_Uncertainty(const TypeBInput *input);

// --- 合成不确定度 (类型A+B) ---

// 合成不确定度结果结构体
typedef struct {
    float combined_std_uncertainty;  // 合成标准不确定度 u_c
    float expanded_uncertainty;      // 扩展不确定度 U = k * u_c
    float effective_dof;             // 有效自由度 (Welch-Satterthwaite公式)
    float coverage_factor;           // 包含因子 k
    float confidence_level;          // 置信水平
} CombinedUncertainty;

// 计算合成不确定度 (简化版，假设不相关)
// u_values: 各分量标准不确定度数组
// dof_values: 各分量自由度数组 (对于B类型，自由度可视为无穷大，使用一个较大的值如1e6)
// sensitivity: 灵敏系数数组 (c_i = ∂f/∂x_i)
// size: 不确定度分量的数量
// confidence_level: 期望的置信水平 (0.0-1.0)
void Calculate_Combined_Uncertainty(const float *u_values, const float *dof_values, 
                                   const float *sensitivity, uint16_t size, 
                                   float confidence_level, CombinedUncertainty *result);

// --- 蒙特卡洛不确定度计算 (简化版) ---

// 蒙特卡洛模拟输入描述结构体
typedef struct {
    float mean;              // 均值或中心值
    float std_deviation;     // 标准差或半宽度
    DistributionType distribution; // 分布类型
} MonteCarloInput;

// 模型函数指针类型定义
// inputs: 输入参数数组
// size: 输入参数数量
// 返回模型计算结果
typedef float (*ModelFunction)(const float *inputs, uint16_t size);

// 蒙特卡洛不确定度结果结构体
typedef struct {
    float mean;              // 模拟结果的平均值
    float std_deviation;     // 模拟结果的标准差 (标准不确定度)
    float lower_bound;       // 下界 (2.5百分位数，对于95%置信区间)
    float upper_bound;       // 上界 (97.5百分位数，对于95%置信区间)
    float expanded_uncertainty; // 扩展不确定度
} MonteCarloResult;

// 使用蒙特卡洛方法计算不确定度
// inputs: 输入参数的描述数组
// size: 输入参数数量
// model: 模型函数指针
// num_simulations: 模拟次数
// rand_buffer: 随机数缓冲区
// confidence_level: 置信水平 (0.0-1.0)
// result: 结果输出
// 返回0表示成功，-1表示失败
int Calculate_MonteCarlo_Uncertainty(const MonteCarloInput *inputs, uint16_t size,
                                    ModelFunction model, uint16_t num_simulations,
                                    float *rand_buffer, float confidence_level,
                                    MonteCarloResult *result);

#endif // __UNCERTAINTY_H