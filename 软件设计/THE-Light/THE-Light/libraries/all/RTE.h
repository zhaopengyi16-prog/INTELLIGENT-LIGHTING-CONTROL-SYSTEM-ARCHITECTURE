/*******************************************************************************
  * Name    :RTE.h
  * Author  : XiaoYI
  * Function:****
  * Version :V1.0.0
  * Data    :2025.4.9
*******************************************************************************/
//This layer is in the middle and is mainly used to process data and decouple software and hardware
#ifndef _RTE_H__
#define _RTE_H__

/*-------------include file---------------------------*/
#include  <Arduino.h>

#include <HAL.h>
#include <math.h>
/**************************************/

#define LIght_Normal_LUX 250
#define LIght_OFF_LUX 0



/*
This is the central control command selection
 */
typedef enum
{
    SYS_NORMAL,        //Normal mode
    SYS_AUTO_PROFILE,  //profile mode
    SYS_ENERGY_SAVING  //Energy saving mode

} SystemMode;
/*
Is anyone judging
 */
typedef enum
{
  None_State,
  Some_State
}AnyoneJudg;
//Light ID
typedef enum
{
  Light_One,
  Light_Two
}LightID;

/**************************************/


/*-------------variable statement---------------------*/
//LUX 
#define LUX_SAMPLE_INTERVAL 5    //  50ms
#define LUX_SAMPLE_TIMES    20     // Sampling times 20
//PID
#define PID_OUTPUT_MIN 0
#define PID_OUTPUT_MAX 255
#define PID_INTEGRAL_LIMIT 100.0

/*******LUX*******/


volatile uint16_t raw_adc_buffer[LUX_SAMPLE_TIMES] = {0};  // LUX原始值缓冲区
volatile uint8_t adc_index = 0;                             // 缓冲区索引
volatile uint32_t last_lux_sample_time = 0;                 // 上次采样时间
volatile uint32_t Lux_filter_val = 0;

/*******Uart*******/




/*******Get Ad val*******/


volatile uint16_t Rxaw_adc_buffer[LUX_SAMPLE_TIMES] = {0};  // ADC原始值缓冲区
volatile uint8_t adc_index_y = 0;                             // 缓冲区索引
volatile uint32_t last_AD_sample_time = 0;                 // 上次采样时间
volatile uint32_t AD_filter_val = 0;

/*********   struct    *********/
/*
This is PID
Kp：
Ki：
Kd：
*/
typedef struct {
    float Kp;
    float Ki; 
    float Kd;
    float integral;      
    float prev_error;    
    uint32_t last_time;  
} PID_Controller;
/*
This is the system architecture
 */
typedef struct
 {
    SystemMode Sys_mode;
   // AnyoneJudg Anyone_Judg;
    LightID Light_ID;
    PID_Controller PID_Controller_Structure;
    /*  */
 } Light_System;
Light_System LightSystem_Structure ;

/****************extern APP USE*******************/
extern void RTE_Light_basic_control(uint8_t warm,uint8_t cold,LightID light);
extern uint32_t RTE_Get_AD_Val(void);
extern void RTE_Protocol_LUX_Val(void);
extern float PID_Compute(PID_Controller* pid, float target, float actual);


/**************** SYS *******************/   



/*------------------function--------------------------*/

/****************************Light basic control *****************************/
void RTE_Light_basic_control(uint8_t warm,uint8_t cold,LightID light)
{
  //constrain LUX 0~240 duty cycle
  switch(light)
  {
    case Light_One:Light_ExecuteAction(HAL_Light_Control_Structure.Hal_LightOne, 
    constrain(warm,0,240),constrain(cold,0,240));break; 
   
    case Light_Two:Light_ExecuteAction(HAL_Light_Control_Structure.Hal_LIghtTwo,
   constrain(warm,0,240) ,constrain(cold,0,240));break; 
     
  }
}

/****************************Get AD val process *****************************/
uint32_t RTE_Get_AD_Val()
{
  //set Sampling time
  uint32_t now = millis();
  if (now - last_AD_sample_time < LUX_SAMPLE_INTERVAL) return;
  last_AD_sample_time = now;
 #if 1
  Rxaw_adc_buffer[adc_index_y] = HAL_GetLux();
  if(adc_index_y==LUX_SAMPLE_TIMES-1)
  {
    uint32_t adc_sum = 0;
    for (uint8_t i=0; i<LUX_SAMPLE_TIMES; i++)
    {
      adc_sum += Rxaw_adc_buffer[i];
    }
    uint16_t adc_avg = adc_sum / LUX_SAMPLE_TIMES;

    if(abs(adc_avg-AD_filter_val)>10)
    {
        AD_filter_val=adc_avg; 
    }   
   AD_filter_val = AD_filter_val/4-15;
    Serial.print("LUx_Val:");
    Serial.println(AD_filter_val);
  }
 #endif
  adc_index_y = (adc_index_y + 1) % LUX_SAMPLE_TIMES;
}

/**************************** communication protocol*****************************/



/****************************LUX val process + PID  *****************************/
/*If we replace the hardware platform that can use timer interrupt+DMA, 
then the data processing here can be rewritten to the underlying layer*/

double Lux_PWM_exp_model(double x) {
    // 归一化参数
    const double mean = 58.0;
    const double std = 54.47;
    
    // 模型系数（95%置信区间中值）
    const double a = 22.47;
    const double b = -0.7976;
    const double c = 23.79;
    const double d = -3.367;

    // 1. 归一化处理
    double x_norm = (x - mean) / std;

    // 2. 计算指数项
    double term1 = a * exp(b * x_norm);
    double term2 = c * exp(d * x_norm);

    // 3. 返回最终结果
    return term1 + term2;
}
//
double AD_exp_model(double x) {
    // 归一化参数
    const double mean = 111.9;
    const double std = 76.58;
    
    // 模型系数（直接使用提供的参数值）
    const double a = 4.926e-7;   // 注意：置信区间包含负值，需结合实际物理意义验证
    const double b = -13.57;
    const double c = 40.95;
    const double d = -0.4667;

    // 1. 输入归一化处理
    double x_norm = (x - mean) / std;

    // 2. 计算两个指数项
    double term1 = a * exp(b * x_norm);  // a极小，此项可能接近零
    double term2 = c * exp(d * x_norm);  // 主导项

    // 3. 返回结果
    return term1 + term2;
}
// 动态模型预测：x_{k+1} = x_k + alpha*(target_AD - x_k)
double dynamic_model(double x_current, double pwm, double alpha) {
    double target_AD = AD_exp_model(pwm);
    return x_current + alpha * (target_AD - x_current);
}
// 基于模型预测的PWM控制
double mpc_control(double current_AD, double target_AD, double alpha, int Np, double lambda) {
    const double u_min = 0.0, u_max = 240.0; // PWM范围约束
    const int num_candidates = 20; // 候选控制量数量（影响精度和计算量）
    double best_pwm = 0.0;
    double min_cost = 1e20; // 初始化为极大值

    // 生成候选PWM控制量（均匀采样）
    for (int i = 0; i <= num_candidates; i++) {
        double u_candidate = u_min + (u_max - u_min) * i / num_candidates;
        double total_cost = 0.0;
        double x_pred = current_AD;
        double u_prev = u_candidate;

        // 预测未来Np步的状态
        for (int t = 0; t < Np; t++) {
            x_pred = dynamic_model(x_pred, u_prev, alpha);
            // 计算跟踪误差项
            double error = x_pred - target_AD;
            total_cost += error * error;
            
            // 计算控制变化惩罚项（仅t>0时）
            if (t > 0) {
                double du = u_prev - u_candidate;
                total_cost += lambda * du * du;
            }
            u_prev = u_candidate; // 假设控制量不变（简化处理）
        }

        // 更新最优解
        if (total_cost < min_cost) {
            min_cost = total_cost;
            best_pwm = u_candidate;
        }
    }

    return best_pwm;
}

//lux val filter 
void RTE_Protocol_LUX_Val()
{
   //set Sampling time
  uint32_t now = millis();
  if (now - last_lux_sample_time < LUX_SAMPLE_INTERVAL) return;
  last_lux_sample_time = now;
 #if 1
  raw_adc_buffer[adc_index] = HAL_GetAdVal();
  if(adc_index==LUX_SAMPLE_TIMES-1)
  {
    uint32_t adc_sum = 0;
    for (uint8_t i=0; i<LUX_SAMPLE_TIMES; i++)
    {
      adc_sum += raw_adc_buffer[i];
    }
    uint16_t adc_avg = adc_sum / LUX_SAMPLE_TIMES;

    if(abs(adc_avg-Lux_filter_val)>10)
    {
        Lux_filter_val=adc_avg;
    }   
   // double y = Lux_PWM_exp_model((double)Lux_filter_val);
    //Serial.print("LUx_Val:");
    //Serial.println(y);
  }
 #endif
  adc_index = (adc_index + 1) % LUX_SAMPLE_TIMES;
  
}
//PID Init
void PID_Init(PID_Controller* pid)
{
    pid->Kp = 0.4;
    pid->Ki = 0.06;
    pid->Kd = 0.01;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->last_time = millis();

}
//PID computr
float PID_Compute(PID_Controller* pid, float target, float actual) 
{
  uint32_t now = millis();
    
    // 时间处理（含溢出保护）
    float dt = (now >= pid->last_time) ? 
              (now - pid->last_time)/1000.0f : 
              (0xFFFFFFFF - pid->last_time + now)/1000.0f;
    
    if(dt <= 0) dt = 0.001;

    float error = target - actual;
    
    // 积分限幅防饱和
    pid->integral += error * dt;
    pid->integral = constrain(pid->integral, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    
    // 微分项滤波
    float derivative = (error - pid->prev_error) / dt;
    
    // 状态更新
    pid->prev_error = error;
    pid->last_time = now;
    
    // 计算输出并限幅
    float output = (pid->Kp * error) + 
                  (pid->Ki * pid->integral) + 
                  (pid->Kd * derivative);
    return constrain(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    
}



/************************************************************************************/


/********************* SYS ***********************/
/*------------------function--------------------------*/

void Handle_normal_mode()
{
  // RTE_Protocol_LUX_Val();
   
   RTE_Get_AD_Val();

   //Light_One
    if(HAL_ReadButton(Light_One)==Light_ON)
    {
        RTE_Light_basic_control((int)AD_filter_val,LIght_Normal_LUX-(int)AD_filter_val,Light_One);
        //Serial.print("On");
         
    }
    else if(HAL_ReadButton(Light_One)==Light_OFF)
    {
        //Serial.print("OFF");
       RTE_Light_basic_control(LIght_OFF_LUX,LIght_OFF_LUX,Light_One);
    }

    
    //Light_Two
     if(HAL_ReadButton(Light_Two)==Light_ON)
    {
        RTE_Light_basic_control((int)AD_filter_val,LIght_Normal_LUX-(int)AD_filter_val,Light_Two);
    }
    else if(HAL_ReadButton(Light_Two)==Light_OFF)
    {
       RTE_Light_basic_control(LIght_OFF_LUX,LIght_OFF_LUX,Light_Two);
    }
    
  
}

void Handle_energy_saving()
{
   // RTE_Protocol_LUX_Val();
   RTE_Get_AD_Val();
    //Light_One
    if(HAL_ReadOccupancy(Light_One)==Light_ON)
    {
        RTE_Light_basic_control(LIght_Normal_LUX,LIght_Normal_LUX,Light_One);
    }
    else if(HAL_ReadOccupancy(Light_One)==Light_OFF)
    {
       RTE_Light_basic_control(LIght_OFF_LUX,LIght_OFF_LUX,Light_One);
    }
    //Light_Two
     if(HAL_ReadOccupancy(Light_Two)==Light_ON)
    {
        RTE_Light_basic_control(LIght_Normal_LUX,LIght_Normal_LUX,Light_Two);
    }
    else if(HAL_ReadOccupancy(Light_Two)==Light_OFF)
    {
       RTE_Light_basic_control(LIght_OFF_LUX,LIght_OFF_LUX,Light_Two);
    }
}



void Handle_auto_profile(Light_System* sys)
{
    if (!sys) return;

    double alpha = 0.3;          // 动态模型惯性系数
    int Np = 15;                  // 预测时域
    double lambda = 0.2;         // 控制平滑权重

    RTE_Protocol_LUX_Val();
    double target_AD = 50.00;  
    double Current_AD = (double)Lux_filter_val;


    //float pid_output =  PID_Compute( &sys->PID_Controller_Structure,(float)target_pwm,(float)y);
    double pid_output = mpc_control(Current_AD,target_AD,alpha, Np, lambda);

    RTE_Light_basic_control(pid_output, pid_output, Light_One);

   // RTE_Light_basic_control(pid_output, pid_output, Light_Two);
    Serial.print(" PWM:");
    Serial.println(pid_output);

    // Serial.print(" Actual:");
   // Serial.println(Lux_filter_val);
 
}


//system Init
void Light_system_Init(Light_System* sys)
{
    if (!sys) return;   
     memset(sys, 0, sizeof(Light_System));  // 清零初始化
     sys->Sys_mode = SYS_NORMAL;
}


// State change
void set_system_mode(Light_System* sys, SystemMode new_mode)
 {
    if (!sys) return;
     static uint16_t step = 0;
    sys->Sys_mode = new_mode;  
    // Initialize after switching
    switch(new_mode) {
        case SYS_NORMAL:
            for(step = 0;step<=200;step+=5 )
            {
                RTE_Light_basic_control(step,step,Light_One) ; 
                RTE_Light_basic_control(step,step,Light_Two) ; 
                delay(20);
            }

            for(step = 200;step>=25;step-=5 )
            {
                RTE_Light_basic_control(step,step,Light_One) ; 
                RTE_Light_basic_control(step,step,Light_Two) ; 
                delay(20);   
            }

            break;
        case SYS_ENERGY_SAVING:
               for(step = 0;step<=200;step+=5 )
            {
                RTE_Light_basic_control(step,step,Light_One) ; 
                RTE_Light_basic_control(step,step,Light_Two) ; 
                delay(20);
            }

          

            break;
         case SYS_AUTO_PROFILE:
               for(step = 0;step<=200;step+=5 )
            {
                RTE_Light_basic_control(step,step,Light_One) ; 
                RTE_Light_basic_control(step,step,Light_Two) ; 
                delay(10);
            }

            for(step = 200;step>=25;step-=5 )
            {
                RTE_Light_basic_control(step,step,Light_One) ; 
                RTE_Light_basic_control(step,step,Light_Two) ; 
                delay(10);   
            }
             RTE_Light_basic_control(0,0,Light_One) ; 
            RTE_Light_basic_control(0,0,Light_Two) ; 
            PID_Init(&sys->PID_Controller_Structure);
            break;

    }
}

// The main  machine operation

void Task_process_lighting_state(Light_System* sys)
{
    if (!sys) return;

  
    switch(sys->Sys_mode) 
    {
        case SYS_NORMAL:
            Handle_normal_mode();
            break;
        case SYS_AUTO_PROFILE:
            Handle_auto_profile(sys);
            break;
        case SYS_ENERGY_SAVING:
            Handle_energy_saving();
           // RTE_Light_basic_control(LIght_Normal_LUX,LIght_Normal_LUX,Light_One);
            break;
        default:
           
            set_system_mode(sys, SYS_NORMAL);
    }
}


/****************************SYS Task*****************************/
// Task start
void Task_Init()
{
    HAL_PWM_Init();
    keystroke_Init();
    Extern_Interrupt_Init();
    Uart_Init(9600);
    Watchdog_Init();
    Light_system_Init(&LightSystem_Structure);
   
}
//Task Security
void Task_Security()
{
    static uint32_t last_wdt_feed = 0;

    // wdt
    if(millis() - last_wdt_feed > 1000)
     {
        Feed_watchdog();
        last_wdt_feed = millis();
    }
}
//Task Mode check
void Task_Event_Mode_Check()
{
    if(interrupt_Flag!=True) return;
    switch(HAL_Light_Control_Structure.Hal_Mode_Change)
    {
   
        case Normal_State: set_system_mode(&LightSystem_Structure ,SYS_NORMAL);        interrupt_Flag = False; break;
        case ENERGY_State: set_system_mode(&LightSystem_Structure ,SYS_ENERGY_SAVING);  interrupt_Flag = False; break;
        case Auto_State: set_system_mode(&LightSystem_Structure , SYS_AUTO_PROFILE);    interrupt_Flag = False; break;
    }
}

//Task Run
void Task_Process_Begin()
{

    Task_process_lighting_state(&LightSystem_Structure);
}

/*
架构主要逻辑：
系统初始化->系统查询状态->系统运行状态
            三个运行状态模式
                            ->正常模式->轮训检测不同区域按钮状态->控制灯光
                            ->节能模式->轮询检测不同区域人员情况->控制灯光
                            ->情景模式->指定区域->检测环境光强度->pid控制->目标光强度
            1个触发状态机制->改变系统状态
            1个安全机制->轮询检测不同区域温度->到达指定温度关断电源并发出警报

    若增加上层接口，那再议
*/
#endif