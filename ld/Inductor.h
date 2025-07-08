#ifndef __INDUCTOR_H
#define __INDUCTOR_H

#include "ld.h"

// 电感数据结构体定义
typedef struct {
    int original[6];      // 原始电感值 [左横L1, 左竖L2, 中横M1, 中竖M2, 右横R1, 右竖R2]
    int normalized[6];    // 归一化后的电感值 (1-100)
    int max_value[6];     // 电感最大值
    int min_value[6];     // 电感最小值
    float error;          // 偏差值
    float weight_h;       // 水平电感权重
    float weight_v;       // 垂直电感权重
    float amp_factor;     // 偏差放大因子
} Inductance_t;


// 函数声明
void Beep_Ctrl(uint8_t state);
void Inductor_Init(void);
void Inductance_Original(int *adc_values);
void ADC_Unify(int *adc_values, int *normalized, int *max_values, int *min_values);
float Cha_bi_he_Cha(float h, float v, int *normalized);
void Inductor_Read(void);
void Calculate_Track_Error(void);
extern int left_1, left_2, middle_1, middle_2, right_1, right_2;

#endif /* __INDUCTOR_H */