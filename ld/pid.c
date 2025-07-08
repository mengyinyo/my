/* pid.c */
#include "pid.h"
#include <math.h>

// PID控制器定义 - 这些全局变量只在这里定义
PID_TypeDef GyroPID;      // 角速度环PID
PID_TypeDef SpeedPID_Left;  // 左轮速度PID
PID_TypeDef SpeedPID_Right; // 右轮速度PID
PID_TypeDef Turn_PID;       // 转向环PID
PID_TypeDef GyroSpeedPID;   // 角速度环PID

// 定义全局变量
short left_TargetSpeed = 0, right_TargetSpeed = 0;     // 左右电机目标速度
short left_EncoderValue = 0, right_EncoderValue = 0;   // 左右编码器实际速度值
int16_t MOTOR_MAX_DUTY = 3000;                           // 电机最大占空比
int16_t MOTOR_MIN_DUTY = 0;                              // 电机最小占空比
double MAX_DIFF_SPEED = 100;// 差速最大值


/**
* @brief 增量式 PID 控制器实现
* @param pid 指向 PID 控制器的指针
* @param actual_Speed 实际速度值
* @param target_Speed 目标速度值
* @return 本次 PID 计算得到的增量值
*/
double PID_Increment(PID_TypeDef *pid, short actual_Speed, short target_Speed)
{
 double Increment = 0;
 // 设置 PID 参数
 //pid->Kp = 18;
 //pid->Ki = 18;
 //pid->Kd = 0;
 pid->Error = target_Speed - actual_Speed; // 计算误差
 // 计算增量
 Increment = pid->Kp * (pid->Error - pid->LastError)
 + pid->Ki * pid->Error
 + pid->Kd * (pid->Error - 2 * pid->LastError + pid->PrevError);
 // 更新误差记录
 pid->PrevError = pid->LastError;
 pid->LastError = pid->Error;

 return Increment;
}

// PID参数初始化
void PID_Init(void)
{
    // 转向环PID参数 - 用于将偏差转换为目标角速度
    Turn_PID.Kp = 2.5f;
    Turn_PID.Ki = 0.0f;
    Turn_PID.Kd = 0.3f;
    Turn_PID.OutputLimit = 100.0f;
    Turn_PID.IntegralLimit = 10.0f;
    Turn_PID.IntegralEnable = 0; // 通常转向不需要积分
    Turn_PID.MaxOutput = 100.0f;  // 兼容旧代码
    Turn_PID.MaxIntegral = 10.0f; // 兼容旧代码
    
    // 角速度环PID参数 - 用于控制舵机实现目标角速度
    GyroSpeedPID.Kp = 0.8f;
    GyroSpeedPID.Ki = 0.1f;
    GyroSpeedPID.Kd = 0.05f;
    GyroSpeedPID.OutputLimit = 1000.0f; // 具体参数需要根据硬件调整
    GyroSpeedPID.IntegralLimit = 500.0f;
    GyroSpeedPID.IntegralEnable = 1;
    GyroSpeedPID.MaxOutput = 1000.0f;  // 兼容旧代码
    GyroSpeedPID.MaxIntegral = 500.0f; // 兼容旧代码
    
    // 左轮速度环PID参数
    SpeedPID_Left.Kp = 100.0f;
    SpeedPID_Left.Ki = 15.0f;
    SpeedPID_Left.Kd = 0.11f;
    SpeedPID_Left.OutputLimit = 8000.0f; // PWM最大值
    SpeedPID_Left.IntegralLimit = 3000.0f;
    SpeedPID_Left.IntegralEnable = 1;
    SpeedPID_Left.MaxOutput = 8000.0f;   // 兼容旧代码
    SpeedPID_Left.MaxIntegral = 3000.0f; // 兼容旧代码
    
    // 右轮速度环PID参数
    SpeedPID_Right.Kp = SpeedPID_Left.Kp;
    SpeedPID_Right.Ki = SpeedPID_Left.Ki;
    SpeedPID_Right.Kd = SpeedPID_Left.Kd;
    SpeedPID_Right.OutputLimit = SpeedPID_Left.OutputLimit;
    SpeedPID_Right.IntegralLimit = SpeedPID_Left.IntegralLimit;
    SpeedPID_Right.IntegralEnable = SpeedPID_Left.IntegralEnable;
    SpeedPID_Right.MaxOutput = SpeedPID_Left.OutputLimit;
    SpeedPID_Right.MaxIntegral = SpeedPID_Left.IntegralLimit;
}

// 位置式PD控制器 - 适用于角速度环

float LocP_DCalc(PID_TypeDef *pid, float target, float current, float maxOut)
{
    static float output_turn = 0;
    // 计算误差
    pid->Error = target - current;
    
    // 计算输出
    output_turn = pid->Kp * pid->Error + pid->Kd * (pid->Error - pid->LastError);
    
    // 更新历史误差
    pid->LastError = pid->Error;
    
    // 输出限幅
    if(output_turn > maxOut) output_turn = maxOut;
    if(output_turn < -maxOut) output_turn = -maxOut;
    
    return output_turn;
}

// 位置式PID控制器 - 适用于速度环

float LocP_DCalc_Speed(PID_TypeDef *pid, float target, float current, float maxOut)
{
    float output_speed = 0;
    // 计算误差
    pid->Error = target - current;
    
    // 积分项计算（带积分限幅）
    pid->SumError += pid->Error;
    if(pid->SumError > pid->MaxIOut) pid->SumError = pid->MaxIOut;
    if(pid->SumError < -pid->MaxIOut) pid->SumError = -pid->MaxIOut;
    
    // 计算输出
    output_speed = pid->Kp * pid->Error + 
                   pid->Ki * pid->SumError + 
                   pid->Kd * (pid->Error - pid->LastError);
    
    // 更新历史误差
    pid->LastError = pid->Error;
    
    // 输出限幅
    if(output_speed > maxOut) output_speed = maxOut;
    if(output_speed < -maxOut) output_speed = -maxOut;
    
    return output_speed;
}

// 差速速度系数计算

float K_Speed(float turn_out)
{
    float diff_speed = 0;
    // 转向越大，差速越大
    diff_speed = turn_out / 2500.0f * MAX_DIFF_SPEED;
    
    // 限制最大差速
    if(diff_speed > MAX_DIFF_SPEED) diff_speed = MAX_DIFF_SPEED;
    if(diff_speed < -MAX_DIFF_SPEED) diff_speed = -MAX_DIFF_SPEED;
    
    return diff_speed;
}


// PID算法实现 - 位置式PID
float intMax = 0;
float PID_Calculate(PID_TypeDef *pid, float target, float current)
{
    // 更新误差历史
    pid->PrevError = pid->LastError;
    pid->LastError = pid->Error;
    pid->Error = target - current;
    
    // 计算微分项 - 采用微分先行结构，减小突变影响
    pid->Differential = pid->Error - 2 * pid->LastError + pid->PrevError;
    
    // 积分项 - 带积分分离和抗饱和
    if (pid->IntegralEnable) {
        // 积分分离：误差太大时不进行积分
        if (fabs(pid->Error) < pid->IntegralLimit) {
            pid->Integral += pid->Error;
            pid->SumError = pid->Integral; // 更新兼容字段
        }
        
        // 积分抗饱和
        intMax = pid->IntegralLimit / pid->Ki;
        pid->Integral = pid->Integral > intMax ? intMax : pid->Integral;
        pid->Integral = pid->Integral < -intMax ? -intMax : pid->Integral;
        pid->SumError = pid->Integral; // 更新兼容字段
    } else {
        pid->Integral = 0;
        pid->SumError = 0;
    }
    
    // 计算PID输出
    pid->Output = pid->Kp * pid->Error + 
                  pid->Ki * pid->Integral + 
                  pid->Kd * pid->Differential;
    
    // 输出限幅
    if (pid->Output > pid->OutputLimit) {
        pid->Output = pid->OutputLimit;
    } else if (pid->Output < -pid->OutputLimit) {
        pid->Output = -pid->OutputLimit;
    }
    
    return pid->Output;
}

// 速度PID特殊处理 - 增量式PID适合电机控制

float PID_Calculate_Speed(PID_TypeDef *pid, float target, float current)
{
    float last_output = 0;
    float delta_output = 0;
    last_output = pid->Output;
    
    // 更新误差历史
    pid->PrevError = pid->LastError;
    pid->LastError = pid->Error;
    pid->Error = target - current;
    
    // 计算增量
    delta_output = pid->Kp * (pid->Error - pid->LastError) +
                        pid->Ki * pid->Error +
                        pid->Kd * (pid->Error - 2 * pid->LastError + pid->PrevError);
    
    // 更新输出
    pid->Output = last_output + delta_output;
    
    // 输出限幅
    if (pid->Output > pid->OutputLimit) {
        pid->Output = pid->OutputLimit;
    } else if (pid->Output < -pid->OutputLimit) {
        pid->Output = -pid->OutputLimit;
    }
    
    return pid->Output;
}

// 重置PID状态
void PID_Reset(PID_TypeDef *pid)
{
    pid->Error = 0;
    pid->LastError = 0;
    pid->PrevError = 0;
    pid->Integral = 0;
    pid->SumError = 0;
    pid->Output = 0;
}

// 动态设置PID参数
void PID_Set_Parameters(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}























double Bias_PID[2] = {0.206, 0.06};  // 外环PD参数[Kp, Kd]
double Gyro_PID[2]= {0.207, 1.4};   // 内环PD参数[Kp, Kd]


/**
 * @brief 位置式PD控制器
 * @param NowPoint 当前测量值
 * @param SetPoint 设定目标值
 * @param PD_params PD参数数组[比例项Kp, 微分项Kd]
 * @param LastError 指向上次误差的指针(需持久化存储)
 * @return 控制器输出值
 */
double PositionPDControl(double NowPoint, double SetPoint, double *PD_params)
{
    double Kp = PD_params[0];
    double Kd = PD_params[1];
    double LastError;
    double currentError = SetPoint - NowPoint;
    double deltaError = currentError - LastError;
    
    // 计算PD输出
    double output = Kp * currentError + Kd * deltaError;
    
    // 更新历史误差
    LastError = currentError;
    
    return output;
}









