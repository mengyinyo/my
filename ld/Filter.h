#ifndef __FILTER_H__
#define __FILTER_H__

#include "ld.h"

// 滑动窗口大小定义
#define SLIDING_WINDOW_SIZE 5

// 卡尔曼滤波器结构体定义
typedef struct {
    float x;      // 状态估计值
    float A;      // 状态转移矩阵
    float H;      // 观测矩阵
    float q;      // 过程噪声协方差
    float r;      // 测量噪声协方差
    float p;      // 估计误差协方差
    float k;      // 卡尔曼增益
} kalman_filter_t;

// 滑动平均滤波器结构体
typedef struct {
    float buffer[SLIDING_WINDOW_SIZE];  // 数据缓冲区
    int index;                          // 当前索引位置
    int count;                          // 当前有效数据计数
} sliding_filter_t;

// 函数声明
void Kalman_Filter_Init(void);
void Sliding_Filter_Init(void);
float Sliding_Average_Filter(float measurement, unsigned char channel);
float Kalman_Filter(float measurement, unsigned char channel);

// ...可能已有的其他代码...

#endif  // __FILTER_H__