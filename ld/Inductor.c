#include "Inductor.h"


// 定义电感数据结构体
Inductance_t g_Inductance;// 电感数据结构体，测试结束后记得加上static
float error; // 偏差值
//定义六个变量，来保存六个电感的值
int left_1 = 0;//左横电感
int left_2 = 0;//左竖电感
int middle_1 = 0;//中横电感
int middle_2 = 0;//右竖电感
int right_1 = 0;//右横电感
int right_2 = 0;//右竖电感

// 蜂鸣器控制函数
void Beep_Ctrl(uint8_t state)
{
    GPIO_Write_Pin(P35, state);  
}

// 电感初始化函数
void Inductor_Init(void)
{
    // 设置电感最大最小值参考范围
    g_Inductance.max_value[0] = 2000;  // L1电感最大值
    g_Inductance.min_value[0] = 0;   // L1电感最小值
    
    g_Inductance.max_value[1] = 2000;  // L2电感最大值
    g_Inductance.min_value[1] = 0;   // L2电感最小值
    
    g_Inductance.max_value[2] = 2000;  // M1电感最大值
    g_Inductance.min_value[2] = 0;   // M1电感最小值
    
    g_Inductance.max_value[3] = 2000;  // M2电感最大值
    g_Inductance.min_value[3] = 0;   // M2电感最小值
    
    g_Inductance.max_value[4] = 2000;  // R1电感最大值
    g_Inductance.min_value[4] = 0;   // R1电感最小值
    
    g_Inductance.max_value[5] = 2000;  // R2电感最大值
    g_Inductance.min_value[5] = 0;   // R2电感最小值
    
    g_Inductance.error = 0;            // 初始化偏差值
    
    // ADC模块在主函数中已初始化，无需再次初始化
}


// 电感数据采集主函数
void Inductance_Original(int *adc_values)
{
    adc_values[0] = Get_ADCResult(5);  // 左横电感
    adc_values[1] = Get_ADCResult(4);  // 左竖电感
    adc_values[2] = Get_ADCResult(3);  // 中横电感
    adc_values[3] = Get_ADCResult(2);  // 中竖电感
    adc_values[4] = Get_ADCResult(1);  // 右横电感
    adc_values[5] = Get_ADCResult(0);  // 右竖电感
}

// 电感数据归一化 (1-100)
void ADC_Unify(int *adc_values, int *normalized, int *max_values, int *min_values)
{
    int i;
    long temp;  // 使用long类型避免中间计算溢出
    
    for (i = 0; i < 6; i++)
    {
        // 先进行限幅保护，确保数值在有效范围内
        if (adc_values[i] > max_values[i])
            adc_values[i] = max_values[i];
        else if (adc_values[i] < min_values[i])  // 使用else if优化逻辑
            adc_values[i] = min_values[i];
        
        // 计算归一化值 (1-100范围)
        // 使用long类型避免中间计算溢出
        temp = (long)(adc_values[i] - min_values[i]) * 99;
        normalized[i] = 1 + (int)(temp / (max_values[i] - min_values[i]));
    }
}

// 计算中间电感动态权重 (水平电感的权重比例 0.0-1.0)
static float Calculate_Weight_Mid(int M_value)
{
    float k = 0.0f;
    
    if(M_value >= 80) // 高于80，水平权重为100%
    {
        k = 1.0f;
    }
    else if(M_value >= 40) // 介于40-80之间，线性变化
    {
        k = 2.5*M_value - 100.0; // y=kx+b：根据 (40,0) 和 (80,100) 解二元一次方程组
    }
    else // 低于40，水平权重为0%
    {
        k = 0.0f;
    }
    
    // 将范围控制在：0.0 ~ 1.0
    return k >= 1.0 ? 1.0 : (k <= 0.0 ? 0.0 : k); // 将 k 限幅保护在 0.0 ~ 1.0 之间
}

// 计算竖直电感差值的放大系数
static float Calculate_Vertical_Amplifier(int L2_value, int R2_value)
{
    float k = 0.0f;
    float sum = 200.0f; // 两个竖直电感最大值的和
    
    // 使用竖直电感差值的绝对值计算权重
    k = (float)abs(L2_value - R2_value) / sum;
    
    // 限幅保护，分母因子不能为0，保留至少0.1
    return k < 0.1f ? 0.1f : (k > 1.0f ? 1.0f : k);
}

// 优化后的差比和差算法计算偏差函数
float Cha_bi_he_Cha(float h, float v, int *normalized)
{
    // 所有变量声明放在函数开头 - 解决C251不支持C99变量就近声明
    float weight_h, weight_v;
    float h_diff;
    float v_diff;
    float h_sum;
    float v_sum;
    float numerator;
    float denominator;
    float result;
    
    // normalized数组对应的电感说明
    // normalized[0]: 左横电感(L1)  normalized[1]: 左竖电感(L2)
    // normalized[2]: 中横电感(M1)  normalized[3]: 中竖电感(M2)
    // normalized[4]: 右横电感(R1)  normalized[5]: 右竖电感(R2)
    if(normalized[2]<25)normalized[2]=25;
    
    
    // 1. 计算动态权重系数
    //weight_h = Calculate_Weight_Mid(normalized[2]);  // 水平电感权重，M1 ↑，水平循迹比重 ↑
    //weight_v = 1.0f - weight_h;                      // 垂直电感权重
    weight_h = 1.0f;  // 水平电感权重，M1 ↑，水平循迹比重 ↑
    weight_v = 1.0f;                      // 垂直电感权重
    
    // 3. 差值项计算
    h_diff = h * (normalized[0] - normalized[4]);     // 水平方向差值(L1-R1)
    v_diff = v * (normalized[1] - normalized[5]);           // 垂直方向差值(L2-R2)
    
    // 4. 和值项计算
    h_sum = h * (normalized[0] + normalized[4]);      // 水平方向和值(L1+R1)
    v_sum = v * (normalized[1] + normalized[5]);            // 垂直方向和值(L2+R2)
    
    // 5. 计算分子（加权差值）
    numerator = weight_h * h_diff + weight_v * v_diff;
    
    // 6. 计算分母（加权和值+中间电感，并应用放大因子）
    denominator = weight_h * h_sum + weight_v * v_sum + normalized[2];
    
    // 7. 防止分母为0导致计算错误 
    if (denominator < 1.0f) {
        denominator = 1.0f;  // 设置一个安全的最小值
    }
    
    // 8. 计算偏差值
    result = (numerator / denominator) * 300.0f;
    
    // 9. 限幅保护
    
    
    return result;
}

// 简化版差比和差算法计算偏差函数，去除中间的电感
float Simple_Track_Error(float a, float b, int *normalized)
{
    // 所有变量声明放在函数开头
    float h_diff, v_diff;         // 水平和垂直差值
    float h_sum, v_sum;           // 水平和垂直和值
    float numerator;              // 分子
    float denominator;            // 分母
    float result;                 // 结果
    float h_weight = 0.6f;        // 水平电感权重(可调整)
    float v_weight = 0.4f;        // 垂直电感权重(可调整)
    
    // 电感索引说明:
    // normalized[0]: 左横电感(L1)  normalized[1]: 左竖电感(L2)
    // normalized[4]: 右横电感(R1)  normalized[5]: 右竖电感(R2)
    
    // 1. 计算水平和垂直电感的差值
    h_diff = b * (normalized[0] - normalized[4]);     // 水平方向差值(L1-R1)
    v_diff = a * (normalized[1] - normalized[5]);     // 垂直方向差值(L2-R2)
    
    // 2. 计算水平和垂直电感的和值
    h_sum = b * (normalized[0] + normalized[4]);      // 水平方向和值(L1+R1)
    v_sum = a * (normalized[1] + normalized[5]);      // 垂直方向和值(L2+R2)
    
    // 3. 计算分子（加权差值）
    numerator = h_weight * h_diff + v_weight * v_diff;
    
    // 4. 计算分母（加权和值）
    denominator = h_weight * h_sum + v_weight * v_sum;
    
    // 5. 防止分母为0导致计算错误
    if (abs(denominator) < 1) {
        denominator = (denominator < 0) ? -0.001f : 0.001f;
    }
    
    // 6. 计算偏差值
    result = (numerator / denominator) * 120.0f;
    
    // 7. 限幅保护
    if (result > 120.0f) {
        return 120.0f;
    } else if (result < -120.0f) {
        return -120.0f;
    }
    
    return result;
}


// 计算赛道偏差（使用增强版差比和差算法）
void Calculate_Track_Error(void)
{
    // 使用优化的差比和差算法
    // h=1.0: 水平电感权重
    // v=1.0: 竖直电感权重
    // 这些系数可以根据实际测试效果进行微调
    g_Inductance.error = Cha_bi_he_Cha(1.0, 1.0, g_Inductance.normalized);
    error = g_Inductance.error;
    
    // 调试用：记录中间状态值
    g_Inductance.weight_h = Calculate_Weight_Mid(g_Inductance.normalized[2]); // 水平权重
    g_Inductance.weight_v = 1.0f - g_Inductance.weight_h; // 垂直权重
    g_Inductance.amp_factor = Calculate_Vertical_Amplifier(
    g_Inductance.normalized[1], g_Inductance.normalized[5]); // 放大因子
}





// 读取电感值并处理
void Inductor_Read(void)
{
    unsigned char i;
    // 1. 采集原始电感数据
    Inductance_Original(g_Inductance.original);

    // 1.5. 先使用滑动去极值滤波处理
    for (i = 0; i < 6; i++)
    {
        g_Inductance.original[i] = Sliding_Average_Filter(g_Inductance.original[i], i);
    }
    
    // 1.6. 再使用卡尔曼滤波进一步平滑处理
    //for (i = 0; i < 6; i++)
    //{
    //    g_Inductance.original[i] = Kalman_Filter(g_Inductance.original[i], i);
    //}
    
    // 2. 归一化处理，范围1-100
    ADC_Unify(g_Inductance.original, g_Inductance.normalized, 
              g_Inductance.max_value, g_Inductance.min_value);
    
    // 3. 计算赛道偏差error
    Calculate_Track_Error();

    // 4. 将归一化后的电感值保存到全局变量中
    left_1 = g_Inductance.normalized[0];    // 左横电感
    left_2 = g_Inductance.normalized[1];    // 左竖电感
    middle_1 = g_Inductance.normalized[2];  // 中横电感
    middle_2 = g_Inductance.normalized[3];  // 右竖电感
    right_1 = g_Inductance.normalized[4];   // 右横电感
    right_2 = g_Inductance.normalized[5];   // 右竖电感

}