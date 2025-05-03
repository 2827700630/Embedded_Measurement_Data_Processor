// 雪豹测量系统 - 简单滤波器实现
#include "xuebao_measure_h/filtering.h"
#include <stddef.h> // For NULL
#include <stdlib.h> // For qsort
#include <string.h> // For memcpy

/**
 * @brief 初始化移动平均滤波器
 * @param filter 指向滤波器结构体的指针
 * @param buffer 指向用于存储历史数据的缓冲区的指针
 * @param size 滤波器窗口大小 (缓冲区大小)
 */
void MovingAverageFilter_Init(MovingAverageFilter *filter, float *buffer, uint16_t size)
{
    if (filter == NULL || buffer == NULL || size == 0)
    {
        // Handle error, maybe assert or return an error code
        return;
    }
    filter->buffer = buffer;
    filter->size = size;
    filter->index = 0;
    filter->sum = 0.0f;
    filter->is_full = 0;

    // 清空缓冲区
    for (uint16_t i = 0; i < size; ++i)
    {
        filter->buffer[i] = 0.0f;
    }
}

/**
 * @brief 更新移动平均滤波器
 * @param filter 指向滤波器结构体的指针
 * @param new_value 新的测量值
 * @return 滤波后的值
 */
float MovingAverageFilter_Update(MovingAverageFilter *filter, float new_value)
{
    if (filter == NULL || filter->buffer == NULL || filter->size == 0)
    {
        // Handle error
        return new_value; // Or return an error indicator like NAN
    }

    // 从总和中减去最旧的值 (如果缓冲区已满)
    if (filter->is_full)
    {
        filter->sum -= filter->buffer[filter->index];
    }

    // 将新值添加到缓冲区和总和中
    filter->buffer[filter->index] = new_value;
    filter->sum += new_value;

    // 更新索引
    filter->index++;
    if (filter->index >= filter->size)
    {
        filter->index = 0;
        filter->is_full = 1; // 缓冲区现在满了
    }

    // 计算平均值
    if (filter->is_full)
    {
        return filter->sum / filter->size;
    }
    else
    {
        // 如果缓冲区未满，则基于当前元素数量计算平均值
        return filter->sum / filter->index;
    }
}

// --- 中值滤波器实现 ---

// qsort 的比较函数 (升序)
static int compare_float(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

/**
 * @brief 初始化中值滤波器
 * @param filter 指向滤波器结构体的指针
 * @param buffer 指向用于存储历史数据的缓冲区的指针
 * @param sorted_buffer 指向用于排序的临时缓冲区的指针 (大小必须与 buffer 相同)
 * @param size 滤波器窗口大小 (缓冲区大小)
 */
void MedianFilter_Init(MedianFilter *filter, float *buffer, float *sorted_buffer, uint16_t size)
{
    if (filter == NULL || buffer == NULL || sorted_buffer == NULL || size == 0)
    {
        return;
    }
    filter->buffer = buffer;
    filter->sorted_buffer = sorted_buffer;
    filter->size = size;
    filter->index = 0;
    filter->is_full = 0;

    // 清空缓冲区
    for (uint16_t i = 0; i < size; ++i)
    {
        filter->buffer[i] = 0.0f;
    }
}

/**
 * @brief 更新中值滤波器
 * @param filter 指向滤波器结构体的指针
 * @param new_value 新的测量值
 * @return 滤波后的值 (中值)
 */
float MedianFilter_Update(MedianFilter *filter, float new_value)
{
    if (filter == NULL || filter->buffer == NULL || filter->sorted_buffer == NULL || filter->size == 0)
    {
        return new_value; // Error
    }

    // 将新值添加到缓冲区
    filter->buffer[filter->index] = new_value;

    // 更新索引
    filter->index++;
    if (filter->index >= filter->size)
    {
        filter->index = 0;
        filter->is_full = 1; // 缓冲区现在满了
    }

    uint16_t current_count = filter->is_full ? filter->size : filter->index;

    // 如果只有一个元素，直接返回
    if (current_count == 0)
        return new_value; // Should not happen if called after Init
    if (current_count == 1)
        return filter->buffer[0];

    // 复制数据到排序缓冲区
    memcpy(filter->sorted_buffer, filter->buffer, current_count * sizeof(float));

    // 对排序缓冲区进行排序
    qsort(filter->sorted_buffer, current_count, sizeof(float), compare_float);

    // 找到中值
    if (current_count % 2 == 1)
    {
        // 奇数个元素，中值是中间的那个
        return filter->sorted_buffer[current_count / 2];
    }
    else
    {
        // 偶数个元素，中值是中间两个元素的平均值
        float mid1 = filter->sorted_buffer[current_count / 2 - 1];
        float mid2 = filter->sorted_buffer[current_count / 2];
        return (mid1 + mid2) / 2.0f;
    }
}

// --- 指数加权移动平均 (EWMA) 滤波器实现 ---

/**
 * @brief 初始化 EWMA 滤波器
 * @param filter 指向滤波器结构体的指针
 * @param alpha 平滑因子 (0 < alpha <= 1). 值越小越平滑，对噪声抑制越好，但响应越慢.
 */
void EWMAFilter_Init(EWMAFilter *filter, float alpha)
{
    if (filter == NULL)
    {
        return;
    }
    // Clamp alpha to (0, 1]
    if (alpha <= 0.0f)
    {
        filter->alpha = 0.01f; // Set a minimum default alpha if invalid
    }
    else if (alpha > 1.0f)
    {
        filter->alpha = 1.0f;
    }
    else
    {
        filter->alpha = alpha;
    }
    filter->last_ewma = 0.0f;
    filter->initialized = 0;
}

/**
 * @brief 更新 EWMA 滤波器
 * @param filter 指向滤波器结构体的指针
 * @param new_value 新的测量值
 * @return 滤波后的值
 */
float EWMAFilter_Update(EWMAFilter *filter, float new_value)
{
    if (filter == NULL)
    {
        return new_value; // Error
    }

    if (!filter->initialized)
    {
        // 第一次更新，直接使用新值作为初始 EWMA 值
        filter->last_ewma = new_value;
        filter->initialized = 1;
    }
    else
    {
        // EWMA 公式: Y_t = alpha * X_t + (1 - alpha) * Y_{t-1}
        filter->last_ewma = filter->alpha * new_value + (1.0f - filter->alpha) * filter->last_ewma;
    }
    return filter->last_ewma;
}