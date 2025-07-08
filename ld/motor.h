#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "ld.h"


typedef struct PID
{
	double iError;  // 本次偏差
    double LastError;  // 上次偏差
    double PrevError;  // 上上次偏差
    double KP;
    double KI;
    double KD;
}PID;


// 电机控制函数声明
void Motors_Setup(void);
void Motor_Control(void);//电机控制主函数
void Cascaded_Control(float current_error, float current_gyro);
void Adjust_Speed_For_Curve(float error_abs);
void Motor_Stop(void);
void Encoder_Read(void);


extern double PlacePID_Control(PID*sptr, double NowPiont, double SetPoint, double *Turn_PID);  // 位置式 PID
extern double PID_Realize(PID*sptr, double ActualSpeed, double SetSpeed, double *MOTOR_PID);   // 增量式 PID

#endif /* __MOTOR_H__ */