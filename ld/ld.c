#include "ld.h"

/* 定义常量 */
#define MIN_CURVE_ERROR 15      // 最小弯道判断偏差
#define Base_Speed 80          // 基础速度
#define K_Speed_Adjust 0.8     // 弯道减速系数
#define K_DIFFERENTIAL 0.5     // 差速系数
#define MIN_SPEED 30           // 最小速度
#define MAX_SPEED 120          // 最大速度
#define PROTECT_KEY 1          // 保护启用标志

extern double encoder_L, encoder_R;	   // 左右编码器实际速度值
extern float target_angle_velocity;// 目标角速度，测试结束后记得加上static
extern double eleOut_1,eleOut_0;               // 内环控制输出

/* 辅助宏函数 */
//#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
//#define ABS(x) ((x) > 0 ? (x) : -(x))



// 循迹控制主函数
void Trailing_control(void)
{
    //根据偏差error 调整电机控制，动态调整速度 - 弯道减速
    Motor_Control();//电机控制主函数
}






// 定时中断函数 - 控制系统核心
void pit_callback(void)
{
    // 获取所有传感器数据
    Inductor_Read();//电感传感器读取并处理
    Encoder_Read();//编码器读取
    
    // 执行循迹控制算法
    Trailing_control();
    
    
    
    // 调试数据输出
    #ifdef DEBUG_MODE
    Debug_Output();
    #endif
}


void Init(void)
{
    // 1. 基础系统初始化
    UART_Init(UART1,115200ul);      //初始化串口1 波特率115200 8位数据,切换管脚需进入该函数内部，默认第一管脚
    GPIO_LED_Init();
    GPIO_KEY_Init();
    Soft_SPI_Init();     //软件SPI初始化，用于陀螺仪
    
    Global_IRQ_Enable();            //使能全局中断    Global_IRQ_Disable();

    OLED_Init();                    //OLED初始化
    delay_ms(100);
    OLED_CLS();   //清屏
    OLED_Show_LQLogo();             //显示LOGO
    UART1_PutStr("USER Init OK \n");      // UART1 打印 printf("USER Init OK \n");
    


   
    // 2. 传感器初始化
    ADC_Init();                 //ADC初始化
    Inductor_Init();            //电感传感器初始化

    //LSM6DSR_Init(); //陀螺仪初始化

    Timer34EncInit();                          //两个方向编码器接口初始化， 定时器3/4做脉冲计数输入，管脚固定
    
    // 3. 执行单元初始化
    Motors_Setup();             //电机初始化
    
    // 4. 算法参数初始化
    PID_Init();               // PID参数初始化
    Kalman_Filter_Init(); // 卡尔曼滤波和滑动平均滤波初始化
    
    
    // 6. 启动定时中断
    TIM_Inits(Timer0,2); //定时器0初始化，2ms中断
    TIM_Inits(Timer1,2);//定时器1初始化
    
    
    // 等待按键启动
    //while(KEY_Read(KEY0));
    delay_ms(100);
    
    OLED_CLS();
    
    // 启动蜂鸣器短响
    //Beep_Ctrl(SET);
    delay_ms(1000);
    Beep_Ctrl(RESET);
}



































// 电感结构体变量声明
extern Inductance_t g_Inductance;  // 电感数据结构体，测试结束后得改定义
// 偏差和控制变量声明
extern float error;                 // 电感循迹偏差值，测试结束后得改定义
extern float current_base_speed;    // 当前基础速度，测试结束后得改定义
// 电机控制变量声明
extern double left_MotorDuty;       // 左电机占空比
extern double right_MotorDuty;      // 右电机占空比
// PID控制相关变量声明
extern PID_TypeDef Turn_PID;        // 转向环PID
extern PID_TypeDef SpeedPID_Left;   // 左轮速度PID
extern PID_TypeDef SpeedPID_Right;  // 右轮速度PID

extern double Left_High_Speed, Right_High_Speed;  // 左右电机目标速度
/**
 * @brief 完整电感循迹系统测试程序
 * @note  使用OLED显示电感数据和控制输出
 */
void Test_Inductor_System(void)
{
    char txt[20];      // 显示文本缓冲区
    u8 display_page = 0;  // 显示页面切换
    u8 motor_enable = 0;  // 电机使能标志
    
    
    OLED_CLS();        // 清屏
    OLED_P8x16Str(0, 0, "Inductor Test");  // 显示标题
    
    while(1)
    {
        // 按键检测
        if(KEY_Read(KEY0) == 0)  // 按下KEY0切换显示页面
        {
            delay_ms(10);  // 去抖
            if(KEY_Read(KEY0) == 0)
            {
                display_page = (display_page + 1) % 3;
                OLED_CLS();
                OLED_P8x16Str(0, 0, "Inductor Test");
                delay_ms(100);
            }
        }
        
        if(KEY_Read(KEY1) == 0)  // 按下KEY1切换电机状态
        {
            delay_ms(10);  // 去抖
            if(KEY_Read(KEY1) == 0)
            {
                motor_enable = !motor_enable;
                if(!motor_enable)
                {
                    // 停止电机
                    left_MotorDuty = 0;
                    right_MotorDuty = 0;
                    Motor_Ctrl(0, 0);
                }
                delay_ms(100);
            }
        }

        
        
       
        
        // 根据当前页面显示不同的信息
        switch(display_page)
        {
            case 0:  // 显示基本信息: 偏差和电机输出
                sprintf(txt, "Error: %5.1f", error);
                OLED_P6x8Str(0, 2, txt);
                
                sprintf(txt, "L:%4d R:%4d", (int)left_MotorDuty, (int)right_MotorDuty);
                OLED_P6x8Str(0, 3, txt);
                
                sprintf(txt, "M:%3d V:%1d", (int)eleOut_0, (int)eleOut_1);
                OLED_P6x8Str(0, 4, txt);
                
                sprintf(txt, "EL:%4d ER:%4d", (int)encoder_L, (int)encoder_R);
                OLED_P6x8Str(0, 5, txt);

                sprintf(txt, "TL:%4d TR:%4d", (int)Left_High_Speed, (int)Right_High_Speed);
                OLED_P6x8Str(0, 6, txt);
                break;
                
            case 1:  // 显示电感原始值
                sprintf(txt, "Raw ADC Values:");
                OLED_P6x8Str(0, 2, txt);
                
                sprintf(txt, "L1:%4d L2:%4d", g_Inductance.original[0], g_Inductance.original[1]);
                OLED_P6x8Str(0, 3, txt);
                
                sprintf(txt, "M1:%4d M2:%4d", g_Inductance.original[2], g_Inductance.original[3]);
                OLED_P6x8Str(0, 4, txt);
                
                sprintf(txt, "R1:%4d R2:%4d", g_Inductance.original[4], g_Inductance.original[5]);
                OLED_P6x8Str(0, 5, txt);
                break;
                
            case 2:  // 显示电感归一化值
                sprintf(txt, "Normalized:");
                OLED_P6x8Str(0, 2, txt);
                
                sprintf(txt, "L1:%3d L2:%3d", g_Inductance.normalized[0], g_Inductance.normalized[1]);
                OLED_P6x8Str(0, 3, txt);
                
                sprintf(txt, "M1:%3d M2:%3d", g_Inductance.normalized[2], g_Inductance.normalized[3]);
                OLED_P6x8Str(0, 4, txt);
                
                sprintf(txt, "R1:%3d R2:%3d", g_Inductance.normalized[4], g_Inductance.normalized[5]);
                OLED_P6x8Str(0, 5, txt);
                
                sprintf(txt, "WH:%3.2f WV:%3.2f", g_Inductance.weight_h, g_Inductance.weight_v);
                OLED_P6x8Str(0, 6, txt);
                break;
        }
        
        // 显示页面编号和电机状态
        sprintf(txt, "Page:%d Motor:%s", display_page, motor_enable ? "ON " : "OFF");
        OLED_P6x8Str(0, 7, txt);
        
        // LED闪烁，表示程序正在运行
        LED_Ctrl(LED0, RVS);
        //delay_ms(100);
    }
}




/**
 * @brief 电感传感器数据采集测试
 * @note  用于验证电感数据的采集和归一化处理是否正常
 */
void Test_Inductor_Sensor(void)
{
    char txt[20];
    OLED_CLS();
    OLED_P8x16Str(0, 0, "Sensor Test");
    
    while(1)
    {
        // 读取电感数据
        Inductor_Read();
        
        // 显示原始值
        sprintf(txt, "L1:%4d L2:%4d", g_Inductance.original[0], g_Inductance.original[1]);
        OLED_P6x8Str(0, 2, txt);
        
        sprintf(txt, "M1:%4d M2:%4d", g_Inductance.original[2], g_Inductance.original[3]);
        OLED_P6x8Str(0, 3, txt);
        
        sprintf(txt, "R1:%4d R2:%4d", g_Inductance.original[4], g_Inductance.original[5]);
        OLED_P6x8Str(0, 4, txt);
        
        // 显示归一化值
        sprintf(txt, "L1:%3d L2:%3d", g_Inductance.normalized[0], g_Inductance.normalized[1]);
        OLED_P6x8Str(0, 5, txt);
        sprintf(txt, "R1:%3d R2:%3d", g_Inductance.normalized[4], g_Inductance.normalized[5]);
        OLED_P6x8Str(0, 6, txt);
        
        sprintf(txt, "Err:%5.1f", error);
        OLED_P6x8Str(0, 7, txt);
        
        LED_Ctrl(LED0, RVS);
        delay_ms(100);
    }
}



/**
 * @brief 电机控制测试程序
 * @note  测试基本电机控制功能，确保左右电机都能正常工作并响应控制信号
 */
void Test_Motor_Basic(void)
{
    char txt[20];
    int16_t duty = 0;
    uint8_t mode = 0; // 0:停止 1:前进 2:后退 3:左转 4:右转
    
    OLED_CLS();
    OLED_P8x16Str(0, 0, "Motor Test");
    
    while(1)
    {
        if(KEY_Read(KEY0) == 0)  // 切换模式
        {
            delay_ms(10);
            if(KEY_Read(KEY0) == 0)
            {
                mode = (mode + 1) % 5;
                delay_ms(100);
            }
        }
        
        if(KEY_Read(KEY1) == 0)  // 调整速度
        {
            delay_ms(10);
            if(KEY_Read(KEY1) == 0)
            {
                duty = (duty < 3000) ? (duty + 200) : duty;
                delay_ms(100);
            }
        }
        
        if(KEY_Read(KEY2) == 0)  // 减小速度
        {
            delay_ms(10);
            if(KEY_Read(KEY2) == 0)
            {
                duty = (duty > 0) ? (duty - 200) : 0;
                delay_ms(100);
            }
        }
        
        // 根据模式控制电机
        switch(mode)
        {
            case 0: // 停止
                Motor_Ctrl(0, 0);
                sprintf(txt, "Mode: Stop   ");
                break;
            case 1: // 前进
                Motor_Ctrl(duty, duty);
                sprintf(txt, "Mode: Forward");
                break;
            case 2: // 后退
                Motor_Ctrl(-duty, -duty);
                sprintf(txt, "Mode: Backward");
                break;
            case 3: // 左转
                Motor_Ctrl(duty/2, duty);
                sprintf(txt, "Mode: Left   ");
                break;
            case 4: // 右转
                Motor_Ctrl(duty, duty/2);
                sprintf(txt, "Mode: Right  ");
                break;
        }
        
        OLED_P6x8Str(0, 2, txt);
        sprintf(txt, "Duty: %4d", duty);
        OLED_P6x8Str(0, 3, txt);
        
        // 显示编码器反馈
        Encoder_Read();
        sprintf(txt, "L:%4d R:%4d", (int)encoder_L, (int)encoder_R);
        OLED_P6x8Str(0, 4, txt);
        
        LED_Ctrl(LED0, RVS);
        delay_ms(100);
    }
}

