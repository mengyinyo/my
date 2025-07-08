#ifndef __LD_H
#define __LD_H

#include "include.h"
#include "Filter.h"
#include "PID.h"
#include "motor.h"
#include "Inductor.h"
#include "LSM6DSR.h"



/* 数据类型定义 */





// 电机输出结构
typedef struct {
    float Turn_Out;          // 转向输出
    float Steer_Speed_Out;   // 转向速度输出
    float All_PWM_Out_Left;  // 左侧电机PWM输出
    float All_PWM_Out_Right; // 右侧电机PWM输出
} PWM_Out_TypeDef;

// 编码器参数结构
typedef struct {
    float Speed_Left;      // 左轮速度
    float Speed_Right;     // 右轮速度
    float Target_Speed;    // 目标速度
} Encoder_TypeDef;


/* 函数声明 */

// 初始化相关
void Init(void);                // 跑车初始化函数
void pit_callback(void);// 中断处理

//测试函数
void Test_Inductor_System(void); // 电感循迹系统测试程序
void Test_Inductor_Sensor(void); // 电感传感器测试程序
void Test_Motor_Basic(void);     // 电机基础测试程序
void Test_Error_Calculation(void);// 偏差计算与差速控制测试
void Test_PID_Tuning(void);      // PID参数调试测试
void Test_Speed_Parameters(void);// 基础速度与转弯系数调整测试
void Test_Speed_PID(void);      // 速度环PID参数调整测试


// 特殊路段处理
/* 特殊路段类型枚举 */
typedef enum {
    ROAD_NORMAL = 0,    // 普通直道/弯道
    ROAD_CROSS,         // 十字路口
    ROAD_CIRCLE,        // 环岛
    ROAD_RAMP,          // 坡道
    ROAD_OBSTACLE       // 障碍物
} Special_Road_Type;





#endif /* __LD_H */