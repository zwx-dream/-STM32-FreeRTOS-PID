#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Key.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Moter.h"
#include "Encoder.h"
#include "PID.h"
#include "Key.h"
#include "queue.h"
#include "AD.h"
uint8_t KeyNum;
float InnerActual;
int16_t Speed,Location;
PID_t Inner =
{
	.KP = 0.0004,
	.KI = 0.031332,
	.KD = 0.05,
	.OutMax = 100,
	.OutMin = -100,
	.Target = 75.0,
	
};
PID_t Outer =
{
		.KP = 0.2,
    .KI = 0.05,
    .KD = 0.01,
    .OutMax = 100,
    .OutMin = -100
};
uint8_t pTxMsgArray = 0;
//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		1024  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define LED1_TASK_PRIO		2
//任务堆栈大小	
#define LED1_STK_SIZE 		128  
//任务句柄
TaskHandle_t LED1Task_Handler;
TaskHandle_t PID_Speed_Handler;
TaskHandle_t PID_Position_Handler;
TaskHandle_t GetKey_Handler;
TaskHandle_t Moter_run_Hander;
TaskHandle_t Moter_Positing_Hander;
//任务函数
void led1_task(void *pvParameters);
void PID_Speed(void *pvParame);
void Get_Key(void *pvParame);
void Moter_run(void *pvParame);
QueueHandle_t OledQueueHandler;
void PID_Positing(void *pvParame);
int main(void)
{
	OLED_Init();
	Key_Init();
	Moter_Init();
	Ad_Init();
	Encoder_Init();
	Outer.Target= 1500;
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
								
								
}

//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
      
    //创建LED1任务
    xTaskCreate((TaskFunction_t )led1_task,     
                (const char*    )"led1_task",   
                (uint16_t       )LED1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )3,
                (TaskHandle_t*  )&LED1Task_Handler);
								

								
		xTaskCreate((TaskFunction_t )Moter_run,     
                (const char*    )"Moter_run",   
                (uint16_t       )LED1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )6,
                (TaskHandle_t*  )&Moter_run_Hander);

								
			xTaskCreate((TaskFunction_t )PID_Speed,     
                (const char*    )"PID_Speed",   
                (uint16_t       )LED1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )5,
                (TaskHandle_t*  )&PID_Speed_Handler);

			xTaskCreate((TaskFunction_t )Get_Key,     
                (const char*    )"Get_Key",   
                (uint16_t       )LED1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )3,
                (TaskHandle_t*  )&GetKey_Handler);
								
		//xTaskCreate(led2_task,"OLED_Show",100,NULL,1,&OLED_Handler);	
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
} 

//LED1任务函数
void led1_task(void *pvParameters)
{
    while(1)
    {
			//目标值
		OLED_ShowNum(1,1,Inner.Target,4);
					
			//实际值
		OLED_ShowNum(2,1,AD_Value[0],4);
			
			if(Inner.Error0>0)
			{
						OLED_ShowNum(2,7,Inner.Error0,6);
			}
			else
			{
				OLED_ShowString(2,7,"-");
					OLED_ShowNum(2,8,-Inner.Error0,6);
			}
			//输出
			if(Inner.Out>0)
			{
				OLED_ShowNum(3,1,Inner.Out,4);
			}
			else
			{
				OLED_ShowString(3,1,"-");
				OLED_ShowNum(3,2,-Inner.Out,4);
			}
		
			//实际值
	//	OLED_ShowNum(4,1,Inner.Actual,4);
			
			if(Inner.Actual>0)
			{
				OLED_ShowNum(4,1,Inner.Actual,5);
			}
			else{
				OLED_ShowString(4,1,"-");
				OLED_ShowNum(4,2,-Inner.Actual,5);
			}
				
			
		vTaskDelay(50);
    }
}

void PID_Speed(void *pvParame)
{
    while(1)
    {
        // 1. 先进行单位转换（假设1926对应25°C，需要根据你的实际温度转换公式）
        float temperature = AD_Value[0] * (25.0/1926.0); // 示例转换
        
        // 2. 或者直接使用归一化值（推荐）
        float normalized_value = (float)AD_Value[0]; // 归一化到0-100%
        
			
        Inner.Actual = normalized_value; // 使用归一化值
        
        Pid_Update(&Inner);
        vTaskDelay(pdMS_TO_TICKS(10)); // 至少10ms的控制周期
    }
}

void Moter_run(void *pvParame)
{
	while(1)
	{
		Moter_SetSpeed(Inner.Out);
		vTaskDelay(1);
	}
}

void PID_Positing(void *pvParame)
{
	while(1)
	{
		Outer.Actual = Location;
		Pid_Update(&Outer);
		
		Inner.Target = Outer.Out;
		vTaskDelay(10);
	}
}
void Get_Key(void *pvParame)
{
	while(1)
	{
		KeyNum = Key_GetNum();
		
		if(KeyNum == 1)
		{
			//Inner.Target += 1; 
			Inner.Target +=200;
		}
		if(KeyNum == 2)
		{
			Inner.Target -= 200; 
		}
		
		Pid_Update(&Inner);
		 vTaskDelay(3);
		
	}
}

