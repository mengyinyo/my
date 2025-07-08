#include "motor.h"

extern float error; // 偏差值
extern short left_EncoderValue, right_EncoderValue;   // 左右编码器实际速度值

// 电机控制常量定义
#define MIN_CURVE_ERROR 15.0f    // 最小弯道判断偏差
#define K_SPEED_ADJUST  0.8f     // 弯道减速系数
#define K_DIFFERENTIAL  1.0f     // 差速系数
#define BASE_SPEED      30.0f    // 基础速度
#define MIN_SPEED       -50.0f    // 最小速度
#define MAX_SPEED       120.0f   // 最大速度

// 级联控制变量
float target_angle_velocity = 0.0f;  // 目标角速度，测试结束后记得加上static
float current_base_speed = BASE_SPEED; // 当前基础速度，测试结束后记得加上static

// 外部变量引用
extern short left_TargetSpeed, right_TargetSpeed;
double left_MotorDuty, right_MotorDuty;
extern int16_t MOTOR_MAX_DUTY, MOTOR_MIN_DUTY;
extern double MAX_DIFF_SPEED;

// 外部PID控制器引用
extern PID_TypeDef Turn_PID;       // 转向环PID
extern PID_TypeDef GyroPID;        // 角速度环PID
extern PID_TypeDef SpeedPID_Left;  // 左轮速度PID
extern PID_TypeDef SpeedPID_Right; // 右轮速度PID

// 外部函数引用
extern float LocP_DCalc(PID_TypeDef *pid, float target, float current, float maxOut);
extern void control_Motors(void);
//extern float Get_Gyro_Value(void); // 获取陀螺仪Z轴角速度值(添加)



PID Left_MOTOR_PID, Right_MOTOR_PID, Turn_PID_ele;  // 三个结构体变量
double elemid = 0;					// 转向环目标位置偏差
double Turn_ele[2]={0.802, 4.04};	// 转向环 PD 参数
double Left_High_Speed, Right_High_Speed, High_Speed;	// 左右轮目标速度、基础目标速度
double Angle;  // 差速系数
double Left_MOTOR[3] = {20, 13, 0};   // 左轮速度环 PID 参数
double Right_MOTOR[3] = {20, 13, 0};  // 右轮速度环 PID 参数
double encoder_L=0, encoder_R=0;							// 左右轮编码器数据
#define MOTOR_MAX 3000  // 注意满占空比为3360（42000000/12500）







//编码器读取函数
void Encoder_Read(void)
{
    left_EncoderValue = -Read_Encoder(1)/200;//读取左编码器
    right_EncoderValue = Read_Encoder(2)/200;//读取右编码器
}


/**
 * @brief 限制数值在指定范围内
 */
static float limit_value(short value, short min, short max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
 * @brief 电机初始化
 */
void Motors_Setup(void)
{
    // 初始化速度变量
    left_TargetSpeed = 0;
    right_TargetSpeed = 0;
    left_MotorDuty = 0;
    right_MotorDuty = 0;
    
    // 初始化控制变量
    target_angle_velocity = 0.0f;
    current_base_speed = BASE_SPEED;
    
    // 初始化电机驱动硬件
    Motor_Init(Motor_FREQ);   //初始化电机PWM，参数PWM频率
    
    // 停止电机，确保初始状态安全
    //Motor_Stop();
}

/**
 * @brief 计算差速转向的左右轮目标速度
 * @param control_output 转向控制输出量
 */
static void calculate_differential_speed(float control_output)
{
    float k = 0.0f;  // 差速比例系数
    
    // 归一化控制输出
    k = fabs(control_output) / 100.0f;
    k = limit_value(k, 0.0f, 1.0f);
    
    // 非对称差速策略
    if (control_output >= 0) {  // 左转
        left_TargetSpeed = current_base_speed * (1.0f - k);
        right_TargetSpeed = current_base_speed * (1.0f + k * K_DIFFERENTIAL);
    } else {  // 右转
        left_TargetSpeed = current_base_speed * (1.0f + k * K_DIFFERENTIAL);
        right_TargetSpeed = current_base_speed * (1.0f - k);
    }
    
    // 速度限幅
    left_TargetSpeed = limit_value(left_TargetSpeed, MIN_SPEED, MAX_SPEED);
    right_TargetSpeed = limit_value(right_TargetSpeed, MIN_SPEED, MAX_SPEED);
}

/**
 * @brief 级联PID控制实现
 * @param current_error 当前偏差
 * @param current_gyro 当前角速度
 */
void Cascaded_Control(float current_error, float current_gyro)
{
    float angle_velocity_output = 0.0f;
    
    // 外环(偏差环) - 计算目标角速度
    target_angle_velocity = LocP_DCalc(&Turn_PID, 0, current_error, Turn_PID.OutputLimit);
    
    // 内环(角速度环) - 控制实际角速度
    angle_velocity_output = LocP_DCalc(&GyroPID, target_angle_velocity, current_gyro, GyroPID.OutputLimit);
    
    // 根据控制输出计算差速，计算差速转向的左右轮目标速度
    calculate_differential_speed(angle_velocity_output);
}

/**
 * @brief 根据赛道弯曲度动态调整速度
 * @param error_abs 偏差绝对值
 */
void Adjust_Speed_For_Curve(float error_abs)
{
    float curve_factor = 0.0f;
    
    // 计算弯道系数 - 弯道越大，速度越低
    if (error_abs > MIN_CURVE_ERROR) {
        curve_factor = K_SPEED_ADJUST * (error_abs - MIN_CURVE_ERROR) / 100.0f;
        curve_factor = limit_value(curve_factor, 0.0f, 0.5f); // 限制最大减速50%
        
        // 弯道减速
        current_base_speed = MAX_SPEED * (1.0f - curve_factor);
    } else {
        // 直道缓慢加速
        current_base_speed += 0.5f;
    }
    
    // 确保基础速度在限制范围内
    current_base_speed = limit_value(current_base_speed, MIN_SPEED, MAX_SPEED);
}


/**
 * @brief 紧急停车函数
 */
//void Motor_Stop(void)
//{
//    left_TargetSpeed = 0;
//    right_TargetSpeed = 0;
//    left_MotorDuty = 0;
//    right_MotorDuty = 0;
//    
//    // 立即更新物理输出
//    Motor_Ctrl(0, 0);  // 直接调用底层函数确保立即停止
//}

/**
* @brief 电机控制函数
* 计算并更新左右电机的占空比
*/
void control_Motors()
{
    
 // 更新左电机占空比
 left_MotorDuty += PID_Realize(&Left_MOTOR_PID, encoder_L, Left_High_Speed, Left_MOTOR);
 // 更新右电机占空比
 right_MotorDuty += PID_Realize(&Right_MOTOR_PID, encoder_R, Right_High_Speed, Right_MOTOR);

 if(left_MotorDuty>0)
	{
        left_MotorDuty = limit_value(left_MotorDuty, 0, MOTOR_MAX);
	}
	else
	{
        left_MotorDuty = limit_value(left_MotorDuty, -MOTOR_MAX, 0);
	}

	if(right_MotorDuty>0)
	{
        right_MotorDuty = limit_value(right_MotorDuty, 0, MOTOR_MAX);
	}
	else
	{
        right_MotorDuty = limit_value(right_MotorDuty, -MOTOR_MAX, 0);
	}
}












extern double Bias_PID[2],Gyro_PID[2];  // 外环PD参数[Kp, Kd]   // 内环PD参数[Kp, Kd]
double eleOut_0 = 0.0;               // 外环控制输出
double eleOut_1 = 0.0;               // 内环控制输出


/**
 * @brief 双环控制系统
 * @note 外环(偏差环)周期为内环的3倍
 */
void Dir_Control(float current_error, float current_gyro)
{
    static int updateCounter = 0;          // 外环更新计数器
    
    // 外环每3次更新执行一次
    if(++updateCounter >= 3) {
        updateCounter = 0;
        eleOut_0 = PlacePID_Control(&Turn_PID_ele, elemid, current_error, Turn_ele);  // 调用位置式 PID
    }
    
    // 内环实时更新
    eleOut_1 = PositionPDControl(current_gyro, eleOut_0, Gyro_PID);//暂时不使用
    eleOut_1 = PlacePID_Control(&Turn_PID_ele, elemid, current_error, Turn_ele);  // 调用位置式 PID
    eleOut_1 = limit_value(eleOut_1, -80, 80); // 限幅
}




/**
 * @brief 差速转向计算函数
 * @param steeringCtrl 转向控制量，范围[-100,100]
 * @note 采用非对称差速策略：
 * - 转向侧轮速衰减幅度更大（1 - |k|）
 * - 非转向侧轮速小幅增加（1 + |k|*STEERING_GAIN）
 * 确保转向灵敏度的同时维持整体速度
 */

void CalculateDifferentialDrive()
{
    High_Speed = 10.0;        // 基础行进速度
    if(eleOut_1 >= 0.0) // 需要左转
    {
        Angle = (eleOut_1) * 0.01;
        Left_High_Speed = High_Speed * (1 - Angle);      // 内轮(可能反转)更多
        Right_High_Speed = High_Speed * (1 + Angle*0.3); // 外轮微增
    }
    else // 需要右转
    {
        Angle = (-eleOut_1) * 0.01;
        Left_High_Speed = High_Speed * (1 + Angle*0.3); // 外轮微增
        Right_High_Speed = High_Speed * (1 - Angle);     // 内轮(可能反转)更多
    }
    right_TargetSpeed=limit_value(right_TargetSpeed, -200, 200);
    left_TargetSpeed=limit_value(left_TargetSpeed, -200, 200);
}







/**
 * @brief 电机控制主函数 - 由主循环或定时器中断调用
 */
void Motor_Control(void)
{
    float error_abs = fabs(error);//取绝对值
    //后续补全
    float gyro_value = Get_Gyro_Value(); // 从陀螺仪获取角速度值
    //float gyro_value = 0;//暂定为0，后续陀螺仪补全
    encoder_L = -Read_Encoder(1)/2;//读取左编码器
    encoder_R = Read_Encoder(2)/2;//读取右编码器
    
    // 根据赛道弯曲度调整基础速度
    //Adjust_Speed_For_Curve(error_abs);
    
    // 执行级联控制
    //Cascaded_Control(error, gyro_value);
    
    //双环控制系统，计算差速转向的左右轮目标速度
    Dir_Control(error, gyro_value);

    //非对称差速转向计算函数
    CalculateDifferentialDrive();
    
    // 更新电机占空比
    control_Motors();
				if(left_1<10&&right_1<10)//停车保护
		{
    right_MotorDuty = 0;
    left_MotorDuty = 0;
    }
    
    // 设置电机物理输出
    Motor_Ctrl((int16)right_MotorDuty, (int16)left_MotorDuty);//前右后左
}







/**
 * @brief 位置式 PID（用于转向控制）
 */
double PlacePID_Control(PID*sptr, double NowPiont, double SetPoint, double *Turn_PID)
{
	double Output;  // 本次输出
	
	sptr->KP = *Turn_PID;  // 参数赋值
	sptr->KD = *(Turn_PID+1);
	
	sptr->iError = SetPoint - NowPiont;  // 当前误差 = 目标值 - 实际值
	
	Output = sptr->KP * sptr->iError  // 比例项
		   + sptr->KD * (sptr->iError - sptr->LastError);  // 微分项
	
	sptr->LastError = sptr->iError;  // 更新误差
	
	return Output;
}


/**
 * @brief 增量式 PID（用于左右轮电机闭环控制）
 */
double PID_Realize(PID*sptr, double ActualSpeed, double SetSpeed, double *MOTOR_PID)
{
	double Increase;  // 单次PID输出
	
	sptr->KP = *MOTOR_PID;  // 参数赋值
	sptr->KI = *(MOTOR_PID+1);
	sptr->KD = *(MOTOR_PID+2);

	sptr->iError = SetSpeed - ActualSpeed;  // 当前误差 = 目标值 - 实际值

	Increase = sptr->KP * (sptr->iError - sptr->LastError)
			 + sptr->KI * sptr->iError
			 + sptr->KD * (sptr->iError - 2*sptr->LastError + sptr->PrevError);

	sptr->PrevError = sptr->LastError;  // 让上上次等于上次
	sptr->LastError = sptr->iError;  // 让上次等于这次

	return Increase;
}
