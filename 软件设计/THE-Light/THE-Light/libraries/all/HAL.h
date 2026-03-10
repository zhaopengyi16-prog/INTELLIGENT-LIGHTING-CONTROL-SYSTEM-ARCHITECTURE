
/*******************************************************************************
  * Name    :HAL.h
  * Author  : XiaoYI
  * Function:****
  * Version :V1.0.0
  * Data    :2025.4.9
*******************************************************************************/
#ifndef _HAL_H__
#define _HAL_H__

/*-------------include file---------------------------*/
#include <avr/wdt.h>

//This is the hardware abstraction layer，Provide users with underlying hardware interfaces

#include <RTE.h>
/*-------------variable statement---------------------*/
#define False 0
#define True 1

#define Light_ON 99
#define Light_OFF 88
//Anti shake time threshold (adjusted according to actual switch)
#define DEBOUNCE_TIME 800 
/*
*******GPIO define***********
*/

// define Intereupt IN0~D2,IN1~D3
#define Extern_PIN 3

// define keystroke GPIO
#define Keystroke_G_D4 4
#define Keystroke_G_D7 7
#define Keystroke_G_D12 12
#define Keystroke_G_D13 13

//define ADC GPIO
#define AD_SENSOR_PIN A0 
#define LUX_SENSOR_PIN A1


//define PWM GPIO/* 3 5 6 9 10 11 PWM pin，here use 5 6 940hz，10 11 450hz*/
//Light_One
#define Light_Pin1Y 5  //W
#define Light_Pin1W 6  //C
//Light_Two 
#define Light_Pin2Y 10
#define Light_Pin2W 11

/*
*/
//define Extern Interrupt State to change SYS State
volatile uint32_t last_interrupt_time = 0;  
volatile uint8_t SYS_Extern_state = False;  
volatile uint8_t SYS_Extern_count = 1;
volatile uint8_t interrupt_Flag = False;

typedef void (*LightActionHandler) (uint8_t warm,uint8_t cold);
/*
******* enum define ***********
*/

// Event
typedef enum
{
  Normal_State=0,
  ENERGY_State,
  Auto_State
}HalModeChange;
/*
******* struct define ***********
*/
//PWM set
typedef struct
{
  /*  */
 LightActionHandler Hal_LightOne;
 LightActionHandler Hal_LIghtTwo;

 HalModeChange Hal_Mode_Change;

}HAL_Light_Control;
/*
******* extern Init fuc ***********
*/
extern void HAL_PWM_Init(void);
extern void keystroke_Init(void);
extern void Extern_Interrupt_Init(void);
extern void Uart_Init(uint32_t baud);
extern void Watchdog_Init(void);

/*
******* extern use fuc ***********
*/
extern uint8_t HAL_ReadButton(uint8_t Light_cmd);
extern uint8_t HAL_ReadOccupancy(uint8_t Light_cmd);
extern uint32_t HAL_GetLux(void);
extern uint32_t HAL_GetAdVal(void);
//extern uint8_t HAL_GetSYS_State(void);
extern void Light_ExecuteAction(LightActionHandler Action,uint8_t warm,uint8_t cold);
extern void Feed_watchdog(void);

/*
*************here fuc*********
*/
static void Light1_Duty_Cycle(uint8_t warm,uint8_t cold);
static void Light2_Duty_Cycle(uint8_t warm,uint8_t cold);

//Init struct
HAL_Light_Control HAL_Light_Control_Structure =
{
  .Hal_LightOne = Light1_Duty_Cycle,
  .Hal_LIghtTwo = Light2_Duty_Cycle,
  .Hal_Mode_Change = Normal_State
};
/*------------------function--------------------------*/
/*********************IO Init*************************/
//PWM
void HAL_PWM_Init() 
{
     pinMode(Light_Pin1Y,OUTPUT);
     pinMode(Light_Pin1W,OUTPUT);
     pinMode(Light_Pin2Y,OUTPUT);
     pinMode(Light_Pin2W,OUTPUT);
}
//IO input
void  keystroke_Init()
{
  pinMode(Keystroke_G_D4,INPUT_PULLUP);
  pinMode(Keystroke_G_D7,INPUT_PULLUP);
  pinMode(Keystroke_G_D12,INPUT_PULLUP);
  pinMode(Keystroke_G_D13,INPUT_PULLUP);

}
//Extern
void  Extern_Interrupt_Init()
{
   pinMode(Extern_PIN, INPUT_PULLUP);
    EICRA |= (1 << ISC11);    // Falling 
    EIMSK |= (1 << INT1);     // start INT1 Interrupt
    EIFR  |= (1 << INTF1);
}
//Uart
void Uart_Init(uint32_t baud)
{
  Serial.begin(baud);// TX 1 RX 0
}
//watchdog
void Watchdog_Init()
{
  // Timeout 2s
  wdt_enable(WDTO_2S); 
}
/*********************IO Init*************************/
//all IO Init shuold be up here
/******************Extern Action*************************/
//Button
uint8_t HAL_ReadButton(uint8_t Light_cmd)
{
  switch(Light_cmd)
  {
    case 0:
        if(digitalRead(Keystroke_G_D4)==LOW)
        {
          return Light_ON;
        }
        else if(digitalRead(Keystroke_G_D4)==HIGH)
        return Light_OFF;
        break;

    case 1:
         if(digitalRead(Keystroke_G_D7)==LOW)
        {
          return Light_ON;
        }
         else if(digitalRead(Keystroke_G_D7)==HIGH)
        return Light_OFF;
        break;
  }
    
}
//Sensor
uint8_t HAL_ReadOccupancy(uint8_t Light_cmd)
{
  switch(Light_cmd)
  {
    case 0:
        if(digitalRead(Keystroke_G_D12)==LOW)
        {
          return Light_OFF;
        }
        else {return Light_ON;}
        break;

    case 1:
         if(digitalRead(Keystroke_G_D13)==LOW)
        {
          return Light_OFF;
        }
        else {return Light_ON;}
        break;
  }
}
//LUX
uint32_t HAL_GetLux()
{
   uint32_t adc_sum = analogRead(LUX_SENSOR_PIN);
   return adc_sum;
}
//AD val
uint32_t HAL_GetAdVal(void)
{
   uint32_t adc_sum = analogRead(AD_SENSOR_PIN);
   return adc_sum;
}
// Light duty cycle 
static void Light1_Duty_Cycle(uint8_t warm,uint8_t cold)
{
  analogWrite(Light_Pin1W, warm);
  analogWrite(Light_Pin1Y, cold);
}

static void Light2_Duty_Cycle(uint8_t warm,uint8_t cold)
{
   analogWrite(Light_Pin2W, warm);
   analogWrite(Light_Pin2Y, cold);
}
//External call interface function To Set Pwm Duty Cycle

void Light_ExecuteAction(LightActionHandler Action,uint8_t warm,uint8_t cold)
{
  
  if(Action!=NULL)
  {
    Action( warm, cold);
  }
}
//Extern Interrupt
ISR(INT1_vect)
{
  uint32_t now = millis();
  if(now - last_interrupt_time < DEBOUNCE_TIME) return;
  last_interrupt_time = now;
  interrupt_Flag = True; 
  switch(SYS_Extern_count)
   {
    case 0: HAL_Light_Control_Structure.Hal_Mode_Change = Normal_State;break;
    case 1: HAL_Light_Control_Structure.Hal_Mode_Change = ENERGY_State;break;
    case 2: HAL_Light_Control_Structure.Hal_Mode_Change = Auto_State;break;
   }
   SYS_Extern_count=(SYS_Extern_count+1)%3;//0~2
}

#if 0
//Get sys state
uint8_t HAL_GetSYS_State()
{
  return HAL_Light_Control_Structure.Hal_Mode_Change;
}
#endif


// watchdog
void Feed_watchdog()
{
  wdt_reset();
}
#endif
