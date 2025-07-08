/* pid.h */
#ifndef __PID_H
#define __PID_H

#include "ld.h"

// PID结构体定义
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    float Error;        // 当前误差
    float LastError;    // 上一次误差
    float PrevError;    // 上上次误差
    
    float Integral;     // 积分项
    float Differential; // 微分项
    float SumError;     // 积分累加（兼容旧代码）
    
    float Output;       // 输出值
    float OutputLimit;  // 输出限幅
    
    float IntegralLimit; // 积分限幅
    float MaxOutput;    // 输出最大值（兼容旧代码）
    float MaxIOut;      // 积分最大值（兼容旧代码）
    float MaxIntegral;  // 积分最大值（兼容旧代码）
    
    uint8_t IntegralEnable; // 积分使能标志
} PID_TypeDef;


 


// 函数声明
void PID_Init(void);
float PID_Calculate(PID_TypeDef *pid, float target, float current);
float PID_Calculate_Speed(PID_TypeDef *pid, float target, float current);
void PID_Reset(PID_TypeDef *pid);
void PID_Set_Parameters(PID_TypeDef *pid, float kp, float ki, float kd);

// PID辅助函数声明
double PID_Increment(PID_TypeDef *pid, short actual_Speed, short target_Speed);
void control_Motors(void);
float K_Speed(float turn_out);
float LocP_DCalc(PID_TypeDef *pid, float target, float current, float maxOut);
float LocP_DCalc_Speed(PID_TypeDef *pid, float target, float current, float maxOut);

double PositionPDControl(double NowPoint, double SetPoint, double *PD_params);

#endif /* __PID_H__ */