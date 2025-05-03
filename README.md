# 嵌入式测量数据处理器 (Embedded Measurement Data Processor)

## 项目概述

本项目为STM32F103平台开发了一套全面的测量数据处理算法库，适用于实验室、工业控制和科学研究等高精度测量场景。该库通过优化的数学算法，解决了测量过程中的噪声干扰、数据插补、曲线拟合和不确定度评估等核心问题，显著提高了测量系统的准确性、可靠性和鲁棒性。

本项目特别考虑了嵌入式系统的资源限制，所有算法均经过优化以适应STM32MCU的计算能力和内存限制，同时保持高精度的计算结果。

## 功能模块与特点

### 1. 数据滤波模块 (filtering.h/c)

- **移动平均滤波器 (Moving Average Filter)**
  - 原理：计算窗口内N个数据点的算术平均值
  - 特点：易于实现，适合处理随机噪声，计算量小
  - 应用：温度、湿度等缓变信号的平滑处理

- **中值滤波器 (Median Filter)**
  - 原理：找出窗口内数据的中值作为输出
  - 特点：对脉冲噪声有极强的抑制能力，能保留信号边缘特性
  - 应用：去除测量数据中的尖峰和异常值

- **指数加权移动平均滤波器 (EWMA Filter)**
  - 原理：根据平滑因子α对新旧数据进行加权平均
  - 特点：响应速度快，可通过单一参数调节平滑强度
  - 应用：需要快速响应变化同时抑制噪声的场合

### 2. 卡尔曼滤波器模块 (kalman_filter.h/c)

- **一维卡尔曼滤波器**
  - 原理：结合测量值与预测值，根据各自不确定度动态调整权重
  - 特点：能够处理高斯噪声，适应系统动态变化
  - 应用：单变量状态估计，如温度、位置等

- **二维卡尔曼滤波器（位置-速度模型）**
  - 原理：同时估计位置和速度两个状态变量
  - 特点：能够处理系统动力学模型，预测未来状态
  - 应用：运动物体的轨迹跟踪，速度估计

### 3. 数据插值模块 (interpolation.h/c)

- **线性插值**
  - 原理：假设数据点之间为线性关系
  - 特点：计算简单，低计算资源占用
  - 应用：简单的数据重构与缺失数据补全

- **拉格朗日插值**
  - 原理：构造经过所有数据点的多项式
  - 特点：高精度，适用于少量数据点
  - 应用：复杂非线性曲线的插值

- **三次样条插值**
  - 原理：构造分段三次多项式，保证一阶、二阶导数连续
  - 特点：插值曲线光滑，无剧烈振荡
  - 应用：需要高质量平滑曲线的场合，如曲线绘制

### 4. 数据拟合模块 (curve_fitting.h/c)

- **线性拟合**
  - 原理：最小二乘法求解直线参数
  - 特点：提供斜率、截距及其标准误差、R²等评价指标
  - 应用：线性关系测量、传感器校准

- **多项式拟合**
  - 原理：扩展最小二乘法求解高阶多项式系数
  - 特点：可拟合复杂曲线关系
  - 应用：非线性系统建模，校准曲线生成

- **指数拟合**
  - 原理：通过对数转换为线性问题求解
  - 特点：适用于指数增长/衰减现象
  - 应用：热学、化学反应速率等指数现象

### 5. 不确定度计算模块 (uncertainty.h/c)

- **A类不确定度评定**
  - 原理：基于统计分析的评定方法
  - 特点：利用多次测量数据的标准偏差评估不确定度
  - 应用：重复性测量场景的不确定度评估

- **B类不确定度评定**
  - 原理：基于先验信息（如仪表精度、温漂等）的评定
  - 特点：支持不同概率分布模型（矩形、三角、正态、U型）
  - 应用：系统误差源的不确定度评估

- **合成不确定度计算**
  - 原理：基于GUM方法，通过误差传递定律合成多源不确定度
  - 特点：考虑自由度、灵敏度系数，计算扩展不确定度
  - 应用：复合测量系统的总体不确定度评估

- **蒙特卡洛不确定度分析**
  - 原理：通过随机抽样模拟评估复杂模型的不确定度
  - 特点：适用于非线性系统，提供完整概率分布
  - 应用：复杂测量模型的不确定度评估

## 数学原理详解

### 1. 滤波器

#### 移动平均滤波器
移动平均滤波器是最简单的滤波器之一，它将最近的N个数据点取平均值作为当前输出：

$$y[n] = \\frac{1}{N} \\sum_{i=0}^{N-1} x[n-i]$$

其频域特性是一个低通滤波器，截止频率约为 $f_s/(2N)$，其中 $f_s$ 是采样频率。移动平均滤波器简单有效，但会引入时间延迟（约为窗口大小的一半），且对阶跃响应较慢。

**代码实现:**
- **初始化:** `MovingAverageFilter_Init(MovingAverageFilter *filter, float *buffer, uint16_t size)`
  - 输入: `filter` (滤波器结构体指针), `buffer` (数据缓冲区指针), `size` (窗口大小)
  - 输出: 无 (通过指针修改 `filter` 结构体)
- **更新:** `MovingAverageFilter_Update(MovingAverageFilter *filter, float new_value)`
  - 输入: `filter` (滤波器结构体指针), `new_value` (新的测量值)
  - 输出: `float` (滤波后的值)

#### 中值滤波器
中值滤波器是一种非线性滤波器，不对数据进行加权平均，而是选取排序后的中间值：

$$y[n] = \\text{median}\\{x[n], x[n-1], ..., x[n-(N-1)]\\}$$

它能有效去除脉冲噪声（如"椒盐噪声"），同时保持信号边沿特性，但对高斯噪声的抑制效果不如移动平均。

**代码实现:**
- **初始化:** `MedianFilter_Init(MedianFilter *filter, float *buffer, float *sorted_buffer, uint16_t size)`
  - 输入: `filter` (滤波器结构体指针), `buffer` (数据缓冲区指针), `sorted_buffer` (排序用缓冲区指针), `size` (窗口大小)
  - 输出: 无 (通过指针修改 `filter` 结构体)
- **更新:** `MedianFilter_Update(MedianFilter *filter, float new_value)`
  - 输入: `filter` (滤波器结构体指针), `new_value` (新的测量值)
  - 输出: `float` (滤波后的中值)

#### 指数加权移动平均滤波器
EWMA滤波器是一种IIR（无限冲激响应）滤波器，仅需存储前一个输出值：

$$y[n] = \\alpha \\cdot x[n] + (1 - \\alpha) \\cdot y[n-1]$$

其中 $\\alpha$ 是平滑因子，范围为(0,1]。该滤波器的等效窗口大小约为 $2/\\alpha - 1$，响应速度比移动平均快，适合需要及时响应的场合。

**代码实现:**
- **初始化:** `EWMAFilter_Init(EWMAFilter *filter, float alpha)`
  - 输入: `filter` (滤波器结构体指针), `alpha` (平滑因子)
  - 输出: 无 (通过指针修改 `filter` 结构体)
- **更新:** `EWMAFilter_Update(EWMAFilter *filter, float new_value)`
  - 输入: `filter` (滤波器结构体指针), `new_value` (新的测量值)
  - 输出: `float` (滤波后的值)

### 2. 卡尔曼滤波

卡尔曼滤波是一种递归最优估计器，其核心思想是基于系统动力学模型和测量模型，不断预测和更新状态估计：

#### 预测步骤
1. 状态预测：$\\hat{x}_{n|n-1} = F \\cdot \\hat{x}_{n-1|n-1}$
2. 协方差预测：$P_{n|n-1} = F \\cdot P_{n-1|n-1} \\cdot F^T + Q$

#### 更新步骤
1. 卡尔曼增益：$K_n = P_{n|n-1} \\cdot H^T \\cdot (H \\cdot P_{n|n-1} \\cdot H^T + R)^{-1}$
2. 状态更新：$\\hat{x}_{n|n} = \\hat{x}_{n|n-1} + K_n \\cdot (z_n - H \\cdot \\hat{x}_{n|n-1})$
3. 协方差更新：$P_{n|n} = (I - K_n \\cdot H) \\cdot P_{n|n-1}$

其中，$F$是状态转移矩阵，$H$是测量矩阵，$Q$是过程噪声协方差，$R$是测量噪声协方差。

**代码实现 (一维):**
- **初始化:** `KalmanFilter1D_Init(KalmanFilter1D *filter, float initial_state, float initial_covariance, float process_noise, float measurement_noise)`
  - 输入: `filter` (滤波器结构体指针), `initial_state` (初始状态估计), `initial_covariance` (初始协方差), `process_noise` (过程噪声方差 Q), `measurement_noise` (测量噪声方差 R)
  - 输出: 无 (通过指针修改 `filter` 结构体)
- **更新:** `KalmanFilter1D_Update(KalmanFilter1D *filter, float measurement)`
  - 输入: `filter` (滤波器结构体指针), `measurement` (新的测量值)
  - 输出: `float` (更新后的状态估计值)

**代码实现 (二维 - 位置速度模型):**
- **初始化:** `KalmanFilter2D_Init(KalmanFilter2D *filter, float dt, float process_noise_pos, float process_noise_vel, float measurement_noise_pos)`
  - 输入: `filter` (滤波器结构体指针), `dt` (时间间隔), `process_noise_pos/vel` (过程噪声), `measurement_noise_pos` (测量噪声)
  - 输出: 无 (通过指针修改 `filter` 结构体)
- **更新:** `KalmanFilter2D_Update(KalmanFilter2D *filter, float measurement_pos)`
  - 输入: `filter` (滤波器结构体指针), `measurement_pos` (新的位置测量值)
  - 输出: 无 (通过指针修改 `filter` 结构体中的状态 `state[0]`-位置, `state[1]`-速度)

### 3. 数据插值

#### 线性插值
在两点 $(x_1, y_1)$ 和 $(x_2, y_2)$ 之间的任意点 $x$ 的值为：

$$y = y_1 + \\frac{(x - x_1)(y_2 - y_1)}{x_2 - x_1}$$

**代码实现:**
- `Linear_Interpolate(const float *x_known, const float *y_known, uint16_t size, float x_interp)`
  - 输入: `x_known`, `y_known` (已知数据点数组), `size` (数据点数量), `x_interp` (需要插值的x坐标)
  - 输出: `float` (插值得到的y值)

#### 拉格朗日插值
构造经过所有给定点的多项式：

$$P(x) = \\sum_{i=0}^{n} y_i \\cdot L_i(x)$$

其中拉格朗日基本多项式为：

$$L_i(x) = \\prod_{j=0, j \\neq i}^{n} \\frac{x - x_j}{x_i - x_j}$$

**代码实现:**
- `Lagrange_Interpolate(const float *x_known, const float *y_known, uint16_t size, float x_interp)`
  - 输入: `x_known`, `y_known` (已知数据点数组), `size` (数据点数量), `x_interp` (需要插值的x坐标)
  - 输出: `float` (插值得到的y值)

#### 三次样条插值
三次样条在每个区间 $[x_i, x_{i+1}]$ 构造一个三次多项式：

$$S_i(x) = a_i + b_i(x - x_i) + c_i(x - x_i)^2 + d_i(x - x_i)^3$$

并保证在节点处一阶导数和二阶导数连续。

**代码实现:**
- **初始化:** `CubicSpline_Init(CubicSpline *spline, const float *x_known, const float *y_known, uint16_t size, float *work_buffer)`
  - 输入: `spline` (样条结构体指针), `x_known`, `y_known` (已知数据点数组), `size` (数据点数量), `work_buffer` (计算用的工作缓冲区)
  - 输出: `int` (0表示成功, -1表示失败)
- **插值:** `CubicSpline_Interpolate(const CubicSpline *spline, const float *x_known, float x_interp)`
  - 输入: `spline` (已初始化的样条结构体指针), `x_known` (已知x坐标数组), `x_interp` (需要插值的x坐标)
  - 输出: `float` (插值得到的y值)

### 4. 数据拟合

#### 线性拟合
使用最小二乘法求解 $y = a + bx$ 中的参数：

$$b = \\frac{n\\sum{xy} - \\sum{x}\\sum{y}}{n\\sum{x^2} - (\\sum{x})^2}$$
$$a = \\frac{\\sum{y} - b\\sum{x}}{n}$$

拟合优度用决定系数 $R^2$ 评估：

$$R^2 = 1 - \\frac{\\sum{(y_i - \\hat{y}_i)^2}}{\\sum{(y_i - \\bar{y})^2}}$$

**代码实现:**
- `Linear_Fit(const float *x_data, const float *y_data, uint16_t size, LinearFitResult *result)`
  - 输入: `x_data`, `y_data` (待拟合数据数组), `size` (数据点数量), `result` (存储拟合结果的结构体指针)
  - 输出: `int` (0表示成功, -1表示失败), 拟合结果通过 `result` 指针返回 (包括斜率 `b`, 截距 `a`, R², 标准误差等)

#### 多项式拟合
扩展为求解 $y = a_0 + a_1x + a_2x^2 + ... + a_nx^n$ 的系数，通过建立法方程组并用高斯消元法求解。

**代码实现:**
- `Polynomial_Fit(const float *x_data, const float *y_data, uint16_t size, uint8_t degree, PolynomialFitResult *result, float *coefficients_buffer, float *work_buffer)`
  - 输入: `x_data`, `y_data` (待拟合数据数组), `size` (数据点数量), `degree` (多项式阶数), `result` (存储拟合结果的结构体指针), `coefficients_buffer` (存储系数的缓冲区), `work_buffer` (计算用的工作缓冲区)
  - 输出: `int` (0表示成功, -1表示失败), 拟合结果通过 `result` 指针返回 (包括系数数组指针, R²)

#### 指数拟合
通过对数转换，将 $y = a \\cdot e^{bx}$ 转为 $\\ln(y) = \\ln(a) + bx$，然后用线性拟合求解。

**代码实现:**
- `Exponential_Fit(const float *x_data, const float *y_data, uint16_t size, ExponentialFitResult *result, float *work_buffer)`
  - 输入: `x_data`, `y_data` (待拟合数据数组), `size` (数据点数量), `result` (存储拟合结果的结构体指针), `work_buffer` (计算用的工作缓冲区)
  - 输出: `int` (0表示成功, -1表示失败), 拟合结果通过 `result` 指针返回 (包括系数 `a`, `b`, R²)

### 5. 不确定度计算

#### A类评定
对多次测量数据进行统计分析：

$$u_A = \\frac{s}{\\sqrt{n}}$$

其中 $s$ 是样本标准差，$n$ 是测量次数。

**代码实现:**
- `Calculate_TypeA_Uncertainty(const float *data, uint16_t size, float coverage_factor, TypeAUncertainty *result)`
  - 输入: `data` (测量数据数组), `size` (数据点数量), `coverage_factor` (包含因子k), `result` (存储结果的结构体指针)
  - 输出: 无, 结果通过 `result` 指针返回 (包括平均值, 标准差, 标准不确定度 $u_A$, 扩展不确定度 $U_A$ 等)

#### B类评定
根据先验信息确定，常见分布的标准不确定度有：
- 矩形分布： $u_B = a / \\sqrt{3}$
- 三角分布： $u_B = a / \\sqrt{6}$
- 正态分布： $u_B = a / k$ ，其中 $k$ 为包含因子

**代码实现:**
- `Calculate_TypeB_Uncertainty(const TypeBInput *input)`
  - 输入: `input` (包含分布类型、半宽度a、包含因子k等信息的结构体指针)
  - 输出: `float` (计算得到的B类标准不确定度 $u_B$)

#### 合成不确定度
通过误差传递定律合成（以独立输入量为例）：

$$u_c^2(y) = \\sum_{i=1}^{N} \\left(\\frac{\\partial f}{\\partial x_i}\\right)^2 u^2(x_i)$$

扩展不确定度 $U = k \\cdot u_c$，$k$ 通常取2（95%置信水平），其值可通过Welch-Satterthwaite公式计算有效自由度后查表或近似计算得到。

**代码实现:**
- `Calculate_Combined_Uncertainty(const float *u_values, const float *dof_values, const float *sensitivity, uint16_t size, float confidence_level, CombinedUncertainty *result)`
  - 输入: `u_values` (各分量标准不确定度数组), `dof_values` (各分量自由度数组), `sensitivity` (灵敏系数数组 $\\partial f / \\partial x_i$), `size` (分量数量), `confidence_level` (置信水平), `result` (存储结果的结构体指针)
  - 输出: 无, 结果通过 `result` 指针返回 (包括合成标准不确定度 $u_c$, 有效自由度, 包含因子k, 扩展不确定度 $U$)

## 使用案例

### 1. 基本滤波处理

下面是一个完整的例子，演示如何使用各种滤波器处理传感器数据：

```c
#include "xuebao_measure_h/filtering.h"
#include "xuebao_measure_h/kalman_filter.h"
#include <stdio.h>

#define MA_WINDOW_SIZE 10
#define MED_WINDOW_SIZE 5
#define SAMPLE_COUNT 100

// 假设的传感器读取函数
float GetSensorReading() {
    // 在实际应用中，这里是从硬件读取数据的代码
    // 为示例返回一个带噪声的值
    static float true_value = 25.0f;
    true_value += ((float)rand() / RAND_MAX - 0.5f) * 0.2f; // 模拟缓慢变化
    float noise = ((float)rand() / RAND_MAX - 0.5f) * 1.0f; // 模拟噪声
    if (rand() % 20 == 0) { // 模拟脉冲噪声
        noise *= 5.0f;
    }
    return true_value + noise;
}


int main(void) {
    // 1. 初始化滤波器
    float ma_buffer[MA_WINDOW_SIZE];
    MovingAverageFilter ma_filter;
    MovingAverageFilter_Init(&ma_filter, ma_buffer, MA_WINDOW_SIZE);

    float med_buffer[MED_WINDOW_SIZE];
    float med_sorted_buffer[MED_WINDOW_SIZE];
    MedianFilter med_filter;
    MedianFilter_Init(&med_filter, med_buffer, med_sorted_buffer, MED_WINDOW_SIZE);

    EWMAFilter ewma_filter;
    EWMAFilter_Init(&ewma_filter, 0.2f);

    KalmanFilter1D kalman_filter;
    // 初始状态估计为25，初始协方差较大(不确定)，过程噪声小，测量噪声较大
    KalmanFilter1D_Init(&kalman_filter, 25.0f, 1.0f, 0.01f, 0.5f);

    // 2. 模拟传感器数据处理
    float sensor_data;
    float ma_result, med_result, ewma_result, kalman_result;

    printf("Sample | Raw   | MA    | Median| EWMA  | Kalman|\n");
    printf("-------|-------|-------|-------|-------|-------|\n");

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // 假设这是从传感器读取的数据
        sensor_data = GetSensorReading();

        // 应用各种滤波器
        ma_result = MovingAverageFilter_Update(&ma_filter, sensor_data);
        med_result = MedianFilter_Update(&med_filter, sensor_data);
        ewma_result = EWMAFilter_Update(&ewma_filter, sensor_data);
        kalman_result = KalmanFilter1D_Update(&kalman_filter, sensor_data);

        // 使用处理后的数据
        printf("%6d | %6.2f | %6.2f | %6.2f | %6.2f | %6.2f |\n",
               i+1, sensor_data, ma_result, med_result, ewma_result, kalman_result);
    }

    return 0;
}
```

### 2. 数据插值与曲线拟合

以下例子展示了如何使用插值和拟合功能进行数据分析：

```c
#include "xuebao_measure_h/interpolation.h"
#include "xuebao_measure_h/curve_fitting.h"
#include <stdio.h>
#include <math.h> // For expf

#define DATA_POINTS 5
#define INTERP_POINTS 10 // 插值点数量减少以简化输出
#define POLY_DEGREE 2

int main(void) {
    // 1. 示例数据点 (假设是某个传感器的校准数据)
    float x_values[DATA_POINTS] = {0.0f, 2.5f, 5.0f, 7.5f, 10.0f}; // 输入物理量
    float y_values[DATA_POINTS] = {1.0f, 2.2f, 3.5f, 4.1f, 4.8f}; // 传感器输出值

    printf("--- 数据拟合 ---\n");
    // 2. 线性拟合
    LinearFitResult linear_fit;
    if (Linear_Fit(x_values, y_values, DATA_POINTS, &linear_fit) == 0) {
        printf("线性拟合: y = %.4f + %.4fx\n", linear_fit.a, linear_fit.b);
        printf("  R² = %.4f, StdErr(a) = %.4f, StdErr(b) = %.4f\n",
               linear_fit.r_squared, linear_fit.std_error_a, linear_fit.std_error_b);
    } else {
        printf("线性拟合失败\n");
    }

    // 3. 多项式拟合
    float coef[POLY_DEGREE + 1];
    float poly_work_buffer[Polynomial_Fit_Required_Buffer_Size(DATA_POINTS, POLY_DEGREE)]; // 计算所需缓冲区大小
    PolynomialFitResult poly_fit;
    poly_fit.coefficients = coef; // 指向存储系数的缓冲区

    if (Polynomial_Fit(x_values, y_values, DATA_POINTS, POLY_DEGREE, &poly_fit, coef, poly_work_buffer) == 0) {
        printf("二次多项式拟合: y = %.4f + %.4fx + %.4fx²\n",
               poly_fit.coefficients[0], poly_fit.coefficients[1], poly_fit.coefficients[2]);
        printf("  R² = %.4f\n", poly_fit.r_squared);
    } else {
        printf("多项式拟合失败\n");
    }

    // 示例指数数据点
    float exp_x[] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    float exp_y[] = {2.7f, 7.2f, 19.8f, 54.0f, 148.0f}; // 接近 y = e^x
    ExponentialFitResult exp_fit;
    float exp_work_buffer[Exponential_Fit_Required_Buffer_Size(5)];

    if (Exponential_Fit(exp_x, exp_y, 5, &exp_fit, exp_work_buffer) == 0) {
        printf("指数拟合: y = %.4f * exp(%.4fx)\n", exp_fit.a, exp_fit.b);
        printf("  R² = %.4f\n", exp_fit.r_squared);
    } else {
        printf("指数拟合失败\n");
    }


    printf("\n--- 数据插值 ---\n");
    // 4. 使用三次样条插值
    CubicSpline spline;
    // 计算三次样条所需缓冲区大小
    float spline_buffer[CubicSpline_Required_Buffer_Size(DATA_POINTS)];

    if (CubicSpline_Init(&spline, x_values, y_values, DATA_POINTS, spline_buffer) == 0) {
        printf("三次样条插值结果:\n");
        printf("  x    | Linear | Lagrange | Spline |\n");
        printf("-------|--------|----------|--------|\n");
        for (int i = 0; i <= INTERP_POINTS; i++) {
            float x = i * 10.0f / INTERP_POINTS;
            float y_linear = Linear_Interpolate(x_values, y_values, DATA_POINTS, x);
            float y_lagrange = Lagrange_Interpolate(x_values, y_values, DATA_POINTS, x);
            float y_spline = CubicSpline_Interpolate(&spline, x_values, x);
            printf("%6.2f | %6.4f | %8.4f | %6.4f |\n", x, y_linear, y_lagrange, y_spline);
        }
    } else {
        printf("三次样条初始化失败\n");
    }

    return 0;
}
```

### 3. 不确定度计算

下面展示了如何计算测量不确定度的例子：

```c
#include "xuebao_measure_h/uncertainty.h"
#include <stdio.h>
#include <stdlib.h> // For rand()
#include <math.h>   // For sqrtf, logf, cosf

#define MEASURE_TIMES 10
#define MC_SIMULATIONS 1000 // 蒙特卡洛模拟次数

// 简单的质量计算模型函数 m = ρ * V
float mass_model(const float* inputs, uint16_t size) {
    // inputs[0]: 密度 ρ, inputs[1]: 体积 V
    if (size >= 2) {
        return inputs[0] * inputs[1];
    }
    return 0.0f;
}

int main(void) {
    printf("--- 不确定度计算 ---\n");
    // 1. A类不确定度 - 基于多次测量
    float temperature_data[MEASURE_TIMES] = {
        25.1f, 25.3f, 25.0f, 25.2f, 25.1f,
        24.9f, 25.2f, 25.3f, 25.0f, 25.1f
    };

    TypeAUncertainty temp_uncertainty;
    // 使用包含因子 k=2 (约95%置信水平)
    Calculate_TypeA_Uncertainty(temperature_data, MEASURE_TIMES, 2.0f, &temp_uncertainty);

    printf("A类评定 (温度):\n");
    printf("  均值 = %.2f °C\n", temp_uncertainty.mean);
    printf("  标准偏差 s = %.4f °C\n", temp_uncertainty.std_deviation);
    printf("  标准不确定度 u_A = %.4f °C\n", temp_uncertainty.std_uncertainty);
    printf("  扩展不确定度 U_A = %.4f °C (k=%.1f, 置信水平 ~%.1f%%)\n",
           temp_uncertainty.expanded_uncertainty, 2.0f,
           temp_uncertainty.confidence_level * 100.0f);

    // 2. B类不确定度 - 基于仪器规格
    TypeBInput pressure_uncertainty_input;
    pressure_uncertainty_input.half_width = 0.05f; // 仪表精度 ±0.05 kPa
    pressure_uncertainty_input.distribution = RECTANGULAR; // 假设均匀分布
    pressure_uncertainty_input.coverage_factor = 0; // 对于矩形分布，此参数未使用

    float u_B_pressure = Calculate_TypeB_Uncertainty(&pressure_uncertainty_input);
    printf("\nB类评定 (压力):\n");
    printf("  半宽度 a = %.2f kPa\n", pressure_uncertainty_input.half_width);
    printf("  分布 = 矩形\n");
    printf("  标准不确定度 u_B = %.4f kPa\n", u_B_pressure);

    // 3. 合成不确定度
    // 假设测量一个电阻 R = V/I, V=5.0V, I=0.1A, R=50Ω
    // 电压测量: A类评定, u(V)=0.02V, 自由度 ν=9
    // 电流测量: B类评定, u(I)=0.001A (来自规格), 自由度 ν=∞ (用大数表示)
    float u_values[2] = {0.02f, 0.001f};    // 电压和电流的标准不确定度
    float dof_values[2] = {9.0f, 1e6f};     // 自由度 (B类设为无穷大)
    // 灵敏系数: ∂R/∂V = 1/I = 1/0.1 = 10 Ω/V
    //            ∂R/∂I = -V/I² = -5.0 / (0.1*0.1) = -500 Ω/A
    float sensitivity[2] = {10.0f, -500.0f};

    CombinedUncertainty combined_unc;
    float target_confidence = 0.95f; // 目标置信水平 95%
    Calculate_Combined_Uncertainty(u_values, dof_values, sensitivity, 2, target_confidence, &combined_unc);

    printf("\n合成不确定度 (电阻 R = V/I):\n");
    printf("  合成标准不确定度 u_c(R) = %.4f Ω\n", combined_unc.combined_std_uncertainty);
    printf("  有效自由度 ν_eff = %.2f\n", combined_unc.effective_dof);
    printf("  包含因子 k = %.2f (置信水平 %.1f%%)\n", combined_unc.coverage_factor, combined_unc.confidence_level * 100.0f);
    printf("  扩展不确定度 U(R) = %.4f Ω\n", combined_unc.expanded_uncertainty);
    printf("  最终结果: R = 50.00 ± %.2f Ω (k=%.2f)\n", combined_unc.expanded_uncertainty, combined_unc.coverage_factor);


    // 4. 蒙特卡洛不确定度分析 (如果资源允许)
    // 对质量计算 m = ρ * V 进行不确定度分析
    MonteCarloInput mc_inputs[2];
    // 密度 ρ = 7.85 g/cm³, 标准不确定度 0.02 g/cm³, 正态分布
    mc_inputs[0].mean = 7.85f;
    mc_inputs[0].std_deviation = 0.02f;
    mc_inputs[0].distribution = NORMAL;

    // 体积 V = 10.0 cm³, 标准不确定度 0.05 cm³, 矩形分布
    // 注意：对于矩形分布，std_deviation = half_width / sqrt(3)
    mc_inputs[1].mean = 10.0f;
    mc_inputs[1].std_deviation = 0.05f / sqrtf(3.0f); // B类矩形分布的标准不确定度
    mc_inputs[1].distribution = RECTANGULAR;

    MonteCarloResult mc_result;
    // 需要一个足够大的缓冲区来存储模拟结果
    float mc_buffer[MC_SIMULATIONS];

    printf("\n蒙特卡洛不确定度分析 (质量 m = ρ * V):\n");
    if (Calculate_MonteCarlo_Uncertainty(mc_inputs, 2, mass_model, MC_SIMULATIONS,
                                        mc_buffer, 0.95f, &mc_result) == 0) {
        printf("  模拟次数 = %d\n", MC_SIMULATIONS);
        printf("  输出均值 = %.4f g\n", mc_result.mean);
        printf("  输出标准差 (标准不确定度) = %.4f g\n", mc_result.std_deviation);
        printf("  95%% 置信区间: [%.4f, %.4f] g\n",
               mc_result.lower_bound, mc_result.upper_bound);
        printf("  扩展不确定度 (区间半宽度) = %.4f g\n", mc_result.expanded_uncertainty);
        printf("  最终结果: m = %.2f ± %.2f g (95%% 置信区间)\n",
               mc_result.mean, mc_result.expanded_uncertainty);
    } else {
        printf("  蒙特卡洛模拟失败 (可能是缓冲区不足或参数错误)\n");
    }

    return 0;
}
```

## 预期结果与性能

1. **滤波器性能**
   - 移动平均滤波器：能有效抑制随机噪声，信噪比提升约 $\\sqrt{N}$ 倍
   - 中值滤波器：可完全消除幅度小于窗口大小一半的脉冲噪声
   - EWMA滤波器：与一阶低通滤波器等效，截止频率约为 $f_s \\cdot \\alpha / (2\\pi)$

2. **卡尔曼滤波器性能**
   - 在噪声为高斯分布时达到最优滤波效果
   - 二维卡尔曼滤波器可预测位置，平均预测误差约为测量噪声的30-50%

3. **插值精度**
   - 线性插值：在数据点间距小时误差可控
   - 三次样条插值：提供连续光滑的曲线，二阶导数连续，避免了拉格朗日插值的龙格现象

4. **拟合效果**
   - 线性拟合：提供参数估计及其不确定度，适合线性相关数据
   - 多项式拟合：适用于有明显非线性趋势的数据，但阶数不宜过高（一般≤3）
   - 指数拟合：对指数增长/衰减数据有良好拟合效果

5. **不确定度计算**
   - A类评定：随测量次数增加，不确定度按 $1/\\sqrt{n}$ 缩小
   - B类评定：根据设备规格获得，不随测量次数变化
   - 合成不确定度：涵盖各误差源，扩展不确定度(k=2)对应约95%置信区间

## 资源占用

- **ROM使用**：约10-15KB（根据使用的功能模块和优化等级）
- **RAM使用**：
  - 静态部分：约1KB
  - 动态部分：视具体应用而定，如滤波器缓冲区、拟合/插值/蒙特卡洛的工作缓冲区等。缓冲区大小直接影响RAM占用。
- **计算资源**：卡尔曼滤波、三次样条初始化、多项式拟合和蒙特卡洛模拟计算量相对较大，其他算法在STM32F103上能实时处理（毫秒级或微秒级）。

## 开发与贡献

1. 克隆仓库并将 `Core/Inc/xuebao_measure_h` 和 `Core/Src/xuebao_measure_c` 目录下的文件添加到您的项目中。
2. 在需要使用的地方包含相应的头文件 (e.g., `#include "xuebao_measure_h/filtering.h"`)。
3. 根据需要配置和初始化所需的功能模块（滤波器、拟合器等）。
4. 将函数调用集成到您的数据采集和处理流程中。

贡献代码前请确保：
- 遵循现有代码风格（例如，使用 Doxygen 风格注释）。
- 添加详细注释，解释算法逻辑和参数含义。
- 考虑嵌入式平台的资源限制（内存、计算速度）。
- 如果添加新功能，请提供单元测试或示例代码。

## 开发日志 (Development Log)

### 2025-05-03
- 完成卡尔曼滤波器（一维和二维）、数据插值（线性、拉格朗日、三次样条）、数据拟合（线性、多项式、指数）和不确定度计算（A类、B类、合成、蒙特卡洛）模块的C语言实现。
- 编写了各模块的头文件和源文件。
- 更新 README.md 文件，添加了详细的功能介绍、数学原理、代码实现说明、使用案例、性能预期和资源占用分析。
- 修复了 README.md 中部分 LaTeX 公式在 GitHub 上的显示问题。
- 添加了开发日志部分。

## 许可证

本项目采用MIT许可证。详情请参阅LICENSE文件。