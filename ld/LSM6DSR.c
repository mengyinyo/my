#include "LSM6DSR.h"

/* 常量定义 */
#define GYRO_SENSITIVITY 0.061f        // 2000dps/32767 转换系数
#define LOW_PASS_ALPHA   0.9f          // 低通滤波系数
#define LOW_PASS_BETA    0.1f          // 1-ALPHA
#define CALIBRATION_SAMPLES 100        // 校准样本数
#define CALIBRATION_DELAY   5          // 校准间隔(ms)

/* 全局变量 */
float yaw_angle = 0.0f;                // 偏航角，单位：度
unsigned long prev_time = 0;           // 上次采样时间
float gyro_z_offset = 0.0f;            // Z轴角速度零偏
float last_gyro_z = 0.0f;              // 低通滤波用的上次角速度值

/* 陀螺仪卡尔曼滤波参数结构体 */
typedef struct {
    float Q_angle;                     // 角度过程噪声协方差
    float Q_bias;                      // 偏置过程噪声协方差
    float R_measure;                   // 测量噪声协方差
    float angle;                       // 角度
    float bias;                        // 陀螺仪零偏估计
    float P[2][2];                     // 误差协方差矩阵
} Gyro_Kalman_t;

Gyro_Kalman_t gyro_kalman;

/* 内部函数原型声明 */
static float normalize_angle(float angle);
static signed short read_gyro_z(void);
static float get_delta_time(void);

/**
 * 角度归一化到0-360度
 */
static float normalize_angle(float angle) {
    while(angle >= 360.0f) angle -= 360.0f;
    while(angle < 0.0f) angle += 360.0f;
    return angle;
}

/**
 * 读取Z轴角速度
 */
static signed short read_gyro_z(void) {
    unsigned char buf[2];
    LQ_SPI_Read(LSM6DSR_OUTZ_L_GYRO, 2, buf);
    return ((uint16)buf[1] << 8) | buf[0];
}

/**
 * 计算时间差并更新上次时间
 */
static float get_delta_time(void) {
    unsigned long current_time = get_system_time_ms();
    float dt = (current_time - prev_time) / 1000.0f;
    prev_time = current_time;
    return dt;
}

/**
 * 更新偏航角函数(无零偏校准)
 */
void update_yaw_angle(void) {
    signed short gyro_z = read_gyro_z();
    float gyro_z_dps = gyro_z * GYRO_SENSITIVITY;
    float dt = get_delta_time();
    
    yaw_angle += gyro_z_dps * dt;
    yaw_angle = normalize_angle(yaw_angle);
}

/**
 * 增强版5秒零偏校准函数
 * 通过多次采样和实时显示进度条，提高校准精度
 */
void extended_gyro_calibration(void)
{
    /* 变量声明（C89要求所有变量在函数开始处声明） */
    float sum;
    float segment_sum;
    float new_offset;
    int i, j, k;
    int progress;
    int total_samples;
    signed short gyro_z;
    char txt[20];
    
    /* 初始化变量 */
    sum = 0.0f;
    total_samples = 0;
    
    /* 清屏并显示校准提示 */
    OLED_CLS();
    OLED_P6x8Str(0, 0, "Hold still!");
    OLED_P6x8Str(0, 1, "5s Calibration");

    /* 共5段校准，每段1秒 */
    for(i = 5; i > 0; i--)
    {
        /* 显示倒计时 */
        sprintf(txt, "Time: %ds", i);
        OLED_P6x8Str(0, 2, txt);
        
        /* 每段采集多个数据点 */
        segment_sum = 0.0f;
        for(j = 0; j < 20; j++)  /* 每段采集20个样本 */
        {
            gyro_z = read_gyro_z();
            segment_sum += gyro_z;
            total_samples++;
            
            /* 显示进度条 - 修复坐标溢出问题 */
OLED_P6x8Str(0, 3, "Progress: [");
progress = 20 - j;
            
/* 修改为8格进度条，确保不会超出128像素宽度 */
for(k = 0; k < 8; k++)  /* 减少为8个字符的进度条 */
{
    unsigned char x_pos;  /* 使用无符号字符确保不会溢出 */
    x_pos = 60 + k*6;
    
    if(k < progress/2.5f)  /* 调整比例以匹配8格进度条 */
    {
        OLED_P6x8Str(x_pos, 3, "=");
    }
    else
    {
        OLED_P6x8Str(x_pos, 3, " ");
    }
}
OLED_P6x8Str(108, 3, "]");  /* 将右括号放在安全位置 */
            
            delay_ms(50);  /* 50ms × 20 = 1秒 */
        }
        
        /* 累加当前段的平均值到总和 */
        sum += segment_sum / 20.0f;
    }
    
    /* 计算所有段的平均零偏值 */
    new_offset = sum / 5.0f;  /* 5段数据 */
    
    /* 应用零偏值 (使用低通滤波平滑过渡) */
    if(gyro_z_offset != 0.0f)
    {
        /* 热启动场景 - 平滑过渡 */
        gyro_z_offset = gyro_z_offset * 0.2f + new_offset * 0.8f;
    }
    else
    {
        /* 首次校准直接赋值 */
        gyro_z_offset = new_offset;
    }
    
    /* 显示校准完成和零偏值 */
    OLED_CLS();
    OLED_P6x8Str(0, 0, "Calibration done!");
    sprintf(txt, "Offset: %.2f", gyro_z_offset);
    OLED_P6x8Str(0, 1, txt);
    OLED_P6x8Str(0, 3, "Starting...");
    delay_ms(1000);
}

/**
 * 更新偏航角（包含零偏校准）
 */
void update_yaw_angle_calibrated(void) {
    signed short gyro_z = read_gyro_z();
    float gyro_z_dps = (gyro_z - gyro_z_offset) * GYRO_SENSITIVITY;
    float dt = get_delta_time();
    
    yaw_angle += gyro_z_dps * dt;
    yaw_angle = normalize_angle(yaw_angle);
}

/**
 * 带低通滤波的偏航角更新
 */
void simplified_yaw_update(void) {
    signed short gyro_z = read_gyro_z();
    float gyro_z_dps = (gyro_z - gyro_z_offset) * GYRO_SENSITIVITY;
    float dt = get_delta_time();
    
    // 应用低通滤波减少噪声影响
    gyro_z_dps = gyro_z_dps * LOW_PASS_ALPHA + last_gyro_z * LOW_PASS_BETA;
    last_gyro_z = gyro_z_dps;
    
    yaw_angle += gyro_z_dps * dt;
    yaw_angle = normalize_angle(yaw_angle);
}

/**
 * 初始化陀螺仪卡尔曼滤波器
 */
void Gyro_Kalman_Init(void) {
    // 建议调整卡尔曼参数为：
    gyro_kalman.Q_angle = 0.0008f;   // 微调小一点更稳定
    gyro_kalman.Q_bias = 0.004f;     // 略微提高以适应零偏变化
    gyro_kalman.R_measure = 0.02f;   // 降低以更信任测量值，提高响应速度
    gyro_kalman.angle = 0.0f;
    gyro_kalman.bias = 0.0f;
    gyro_kalman.P[0][0] = 0.0f;
    gyro_kalman.P[0][1] = 0.0f;
    gyro_kalman.P[1][0] = 0.0f;
    gyro_kalman.P[1][1] = 0.0f;
}

/**
 * 陀螺仪卡尔曼滤波算法
 */
float Gyro_Kalman_Filter(float new_angle, float new_rate, float dt) {
    float rate;
    float S;
    float K[2];
    float y;
    float P00_temp;
    float P01_temp;
    
    /* 预测步骤 */
    rate = new_rate - gyro_kalman.bias;
    gyro_kalman.angle += dt * rate;
    
    /* 更新误差协方差矩阵 */
    gyro_kalman.P[0][0] += dt * (dt * gyro_kalman.P[1][1] - gyro_kalman.P[0][1] - gyro_kalman.P[1][0] + gyro_kalman.Q_angle);
    gyro_kalman.P[0][1] -= dt * gyro_kalman.P[1][1];
    gyro_kalman.P[1][0] -= dt * gyro_kalman.P[1][1];
    gyro_kalman.P[1][1] += gyro_kalman.Q_bias * dt;
    
    /* 计算卡尔曼增益 */
    S = gyro_kalman.P[0][0] + gyro_kalman.R_measure;
    K[0] = gyro_kalman.P[0][0] / S;
    K[1] = gyro_kalman.P[1][0] / S;
    
    /* 计算角度误差 */
    y = new_angle - gyro_kalman.angle;
    
    /* 计算最终结果 */
    gyro_kalman.angle += K[0] * y;
    gyro_kalman.bias += K[1] * y;
    
    /* 更新协方差矩阵 */
    P00_temp = gyro_kalman.P[0][0];
    P01_temp = gyro_kalman.P[0][1];
    gyro_kalman.P[0][0] -= K[0] * P00_temp;
    gyro_kalman.P[0][1] -= K[0] * P01_temp;
    gyro_kalman.P[1][0] -= K[1] * P00_temp;
    gyro_kalman.P[1][1] -= K[1] * P01_temp;
    
    return gyro_kalman.angle;
}

/**
 * 使用卡尔曼滤波更新偏航角
 */
void kalman_yaw_update(void) {
    // 原有代码
    static int stable_count = 0;
    static float gyro_sum = 0.0f;
    
    signed short gyro_z = read_gyro_z();
    float raw_gyro_z_dps = gyro_z * GYRO_SENSITIVITY;  // 未减零偏的原始值
    float gyro_z_dps = (gyro_z - gyro_z_offset) * GYRO_SENSITIVITY;
    float dt;
    
    // 静止检测与零偏动态调整
    if(fabs(gyro_z_dps) < 0.3f) {
        stable_count++;
        gyro_sum += raw_gyro_z_dps;
        
        // 如果连续100次检测都很稳定，更新零偏
        if(stable_count >= 100) {
            // 动态更新零偏，使用低通滤波平滑过渡
            gyro_z_offset = gyro_z_offset * 0.95f + (gyro_z_offset + gyro_sum/stable_count) * 0.05f;
            stable_count = 0;
            gyro_sum = 0.0f;
        }
        
        // 静止状态下角速度置零，避免积分漂移
        gyro_z_dps = 0.0f;
    } else {
        // 不稳定状态重置计数器
        stable_count = 0;
        gyro_sum = 0.0f;
    }
    
    // 其余代码不变
    dt = get_delta_time();
    yaw_angle = Gyro_Kalman_Filter(yaw_angle, gyro_z_dps, dt);
    yaw_angle = normalize_angle(yaw_angle);
}

/**
 * 重置偏航角到指定值
 */
void reset_yaw_angle(float new_angle) {
    yaw_angle = normalize_angle(new_angle);
}

/**
 * 获取当前偏航角
 */
float get_yaw_angle(void) {
    return yaw_angle;
}

/**
 * 主循环函数
 */
void main_loop(void) 
{
        char txt[20];      // 显示文本缓冲区
        //float gz = Get_Gyro_Value();  // 从陀螺仪获取Z轴角速度值
        // 选择一种姿态更新方法
        //update_yaw_angle_calibrated();  // 基本校准版本
        //simplified_yaw_update();           // 低通滤波版本
        kalman_yaw_update();            // 卡尔曼滤波版本、
        

        OLED_CLS();        // 清屏
        
        // 显示当前姿态角
        sprintf(txt, "Yaw: %.2f", yaw_angle);
        //sprintf(txt, "gz: %.2f", gz);
        OLED_P6x8Str(0, 0, txt);
        
        delay_ms(10);  // 调整循环频率
}


/**
 * 修改LSM6DSR初始化函数，集成扩展校准
 */
void LSM6DSR_Init(void)
{
    /* 软件SPI初始化 */
    Soft_SPI_Init();
    
    /* 1. 初始化SPI通信和配置陀螺仪寄存器 */
    LQ_SPI_LSM60DSR_Init();
    
    /* 2. 初始化卡尔曼滤波器参数 */
    Gyro_Kalman_Init();
    
    /* 3. 初始化时间戳 */
    prev_time = get_system_time_ms();
    
    /* 4. 执行5秒增强版零偏校准代替原校准函数 */
    extended_gyro_calibration();
    
    /* 5. 重置偏航角 */
    reset_yaw_angle(0.0f);
}


// 获取当前Z轴角速度值（度/秒）
float Get_Gyro_Value(void) 
{
    signed short gyro_z = read_gyro_z();
    // 应用零偏校准并转换单位
    float gyro_rate = (gyro_z - gyro_z_offset) * GYRO_SENSITIVITY;
    
    // 可选：静止状态下置零
    if(fabs(gyro_rate) < 0.3f) {
        gyro_rate = 0.0f;
    }
    
    return gyro_rate;
}