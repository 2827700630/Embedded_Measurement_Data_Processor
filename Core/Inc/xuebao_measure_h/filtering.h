// 雪豹测量系统 - 简单滤波器实现
#ifndef __FILTERING_H
#define __FILTERING_H

#include <stdint.h>

// --- 移动平均滤波器 ---
typedef struct
{
    float *buffer;   // 存储历史数据的缓冲区
    uint16_t size;   // 缓冲区大小 (滤波器窗口大小)
    uint16_t index;  // 当前数据插入位置
    float sum;       // 当前缓冲区内数据的总和
    uint8_t is_full; // 缓冲区是否已满
} MovingAverageFilter;

// 初始化移动平均滤波器
void MovingAverageFilter_Init(MovingAverageFilter *filter, float *buffer, uint16_t size);

// 更新滤波器并获取滤波后的值
float MovingAverageFilter_Update(MovingAverageFilter *filter, float new_value);


// --- 中值滤波器 ---
typedef struct
{
    float *buffer;      // 存储历史数据的缓冲区
    float *sorted_buffer; // 用于排序的临时缓冲区
    uint16_t size;      // 缓冲区大小 (滤波器窗口大小)
    uint16_t index;     // 当前数据插入位置
    uint8_t is_full;    // 缓冲区是否已满
} MedianFilter;

// 初始化中值滤波器
void MedianFilter_Init(MedianFilter *filter, float *buffer, float *sorted_buffer, uint16_t size);

// 更新中值滤波器并获取滤波后的值
float MedianFilter_Update(MedianFilter *filter, float new_value);


// --- 指数加权移动平均 (EWMA) 滤波器 ---
typedef struct
{
    float alpha;        // 平滑因子 (0 < alpha <= 1)
    float last_ewma;    // 上一次的 EWMA 值
    uint8_t initialized;// 是否已初始化 (至少有一个值)
} EWMAFilter;

// 初始化 EWMA 滤波器
void EWMAFilter_Init(EWMAFilter *filter, float alpha);

// 更新 EWMA 滤波器并获取滤波后的值
float EWMAFilter_Update(EWMAFilter *filter, float new_value);


#endif // __FILTERING_H