/*******************************************************************************
  * Name    :THE-Light.ino
  * Author  : XiaoYI
  * Function:****
  * Version :V1.0.0
  * Data    :2025.4.9
*******************************************************************************/

/*-------------include file---------------------------*/

#include <RTE.h>
#include <HAL.h>

/*-------------variable statement---------------------*/


/*------------------function--------------------------*/





//Task initialization location
void setup() 
{
   Task_Init();
}

//Task start location
void loop() 
{ 
  
        
 Task_Event_Mode_Check();
 Task_Process_Begin();
 Task_Security();
}
