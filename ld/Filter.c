#include "Filter.h"

// 卡尔曼滤波器数组
kalman_filter_t kalman_filters[6];
// 滑动平均滤波器数组
sliding_filter_t sliding_filters[6];


// 滑动平均滤波器初始化
void Sliding_Filter_Init(void) {
    int i, j;
    for (i = 0; i < 6; i++) {
        sliding_filters[i].index = 0;
        sliding_filters[i].count = 0;
        for (j = 0; j < SLIDING_WINDOW_SIZE; j++) {
            sliding_filters[i].buffer[j] = 0;
        }
    }
}

// 卡尔曼滤波器和滑动平均滤波器初始化
void Kalman_Filter_Init(void) {
    int i;
    for (i = 0; i < 6; i++) {
        kalman_filters[i].x = 0;
        kalman_filters[i].A = 1;
        kalman_filters[i].H = 1;
        kalman_filters[i].q = 0.01f;
        kalman_filters[i].r = 0.1f;
        kalman_filters[i].p = 1;
    }
    // 初始化滑动平均滤波器
    Sliding_Filter_Init();
}

// 去极值滑动平均滤波算法
float Sliding_Average_Filter(float measurement, unsigned char channel) {
    float temp[SLIDING_WINDOW_SIZE]; /* 所有变量声明移到函数开头 */
    float sum;
    float t;
    int i, j;
    
    if (channel >= 6) return measurement;
    
    // 更新缓冲区
    sliding_filters[channel].buffer[sliding_filters[channel].index] = measurement;
    sliding_filters[channel].index = (sliding_filters[channel].index + 1) % SLIDING_WINDOW_SIZE;
    if (sliding_filters[channel].count < SLIDING_WINDOW_SIZE) {
        sliding_filters[channel].count++;
    }
    
    // 如果数据不足，直接返回测量值
    if (sliding_filters[channel].count < 3) {
        return measurement;
    }
    
    // 复制数据到临时数组进行排序
    for (i = 0; i < sliding_filters[channel].count; i++) {
        temp[i] = sliding_filters[channel].buffer[i];
    }
    
    // 冒泡排序
    for (i = 0; i < sliding_filters[channel].count - 1; i++) {
        for (j = 0; j < sliding_filters[channel].count - 1 - i; j++) {
            if (temp[j] > temp[j + 1]) {
                t = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = t;
            }
        }
    }
    
    // 去除最大值和最小值，计算平均值
    sum = 0;
    for (i = 1; i < sliding_filters[channel].count - 1; i++) {
        sum += temp[i];
    }
    
    return sum / (sliding_filters[channel].count - 2);
}

// 卡尔曼滤波算法
float Kalman_Filter(float measurement, unsigned char channel) {
    if (channel >= 6) return measurement;
    
    // 使用直接数组访问代替指针访问
    // 预测
    kalman_filters[channel].x = kalman_filters[channel].A * kalman_filters[channel].x;
    kalman_filters[channel].p = kalman_filters[channel].p + kalman_filters[channel].q;
    
    // 更新
    kalman_filters[channel].k = kalman_filters[channel].p * kalman_filters[channel].H / 
                              (kalman_filters[channel].H * kalman_filters[channel].p * 
                               kalman_filters[channel].H + kalman_filters[channel].r);
                               
    kalman_filters[channel].x = kalman_filters[channel].x + 
                              kalman_filters[channel].k * 
                              (measurement - kalman_filters[channel].H * kalman_filters[channel].x);
                              
    kalman_filters[channel].p = (1 - kalman_filters[channel].k * kalman_filters[channel].H) * 
                              kalman_filters[channel].p;
    
    return kalman_filters[channel].x;
}