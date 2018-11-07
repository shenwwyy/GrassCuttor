/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "led.h"
#include "motor.h"
#include "tim.h"
#include "adc.h"
#include "ublox.h"
#include "uart.h"
#include "ultrasonic.h"

#include "control.h"

#include "stm32f4xx_hal.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "protocol.h"

#include "mymath.h"

#include "flash.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId ControltTaskHandle;
osThreadId DataLinkTaskHandle;
osThreadId SenserTaskHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of ControltTask */
  osThreadDef(ControltTask, StartDefaultTask, osPriorityNormal, 0, 256);
  ControltTaskHandle = osThreadCreate(osThread(ControltTask), NULL);

  /* definition and creation of DataLinkTask */
  osThreadDef(DataLinkTask, StartTask02, osPriorityNormal, 0, 256);
  DataLinkTaskHandle = osThreadCreate(osThread(DataLinkTask), NULL);

  /* definition and creation of SenserTask */
  osThreadDef(SenserTask, StartTask03, osPriorityNormal, 0, 256);
  SenserTaskHandle = osThreadCreate(osThread(SenserTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	
//	Parameter_R_PID();//读取参数
	
	
	uint32_t Bat[4];
	
	//电池电压测量读取调用
	HAL_ADC_Start_DMA(&hadc1,Bat,sizeof(Bat));
	
  //PWM初始化
	HAL_TIM_IC_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_IC_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start(&htim1,TIM_CHANNEL_4);
	
	HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_4);
	
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);

	//超声波接收调用
	//HAL_UART_Receive_DMA(&huart1,Control.Senser.Sonar.forward.rxbuff,sizeof(Control.Senser.Sonar.forward.rxbuff));
	HAL_UART_Receive_DMA(&huart2,Control.Senser.Sonar.left.rxbuff,sizeof(Control.Senser.Sonar.left.rxbuff));
	HAL_UART_Receive_DMA(&huart1,HAL_IO.U5.rxbuf,sizeof(HAL_IO.U5.rxbuf));
	HAL_UART_Receive_DMA(&huart3,Control.Senser.Sonar.right.rxbuff,sizeof(Control.Senser.Sonar.right.rxbuff));
	
	//数据链接收调用
	//normal
	//HAL_UART_Receive_DMA(&huart5,HAL_IO.U5.rxbuf,sizeof(HAL_IO.U5.rxbuf));
	//test
	//HAL_UART_Receive_DMA(&huart2,HAL_IO.U5.rxbuf,sizeof(HAL_IO.U5.rxbuf));
	
	
	GPS_USART6_UART_Init(9600);
	
	
	//GPS接收调用
	HAL_UART_Receive_DMA(&huart6,Control.Senser.GPS.rbuff,sizeof(Control.Senser.GPS.rbuff));
	ubloxInitGps();
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);//50ms
		//LED_Toggle(0);
		
		//当前动作运行完成后，开始切换任务，让其他任务得以运行		
		//数传发送
    Protocol_T_Parameter();
		Protocol_T_WayPoint();
		//数传解码

		Protocol_Rev();
		
		//保存数据
		if(Control.Parameter.isSaveParameter == 0x01)
		{
			 Control.Parameter.isSaveParameter = 0;
//			 Parameter_S_PID();
		}
		
		
		osThreadYield();
  }
  /* USER CODE END StartDefaultTask */
}

/* StartTask02 function */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */

	

	Control.Senser.Voltage.Battery1.Max = 8.4f;
	Control.Senser.Voltage.Battery1.Min = 7.0f;
	Control.Senser.Voltage.Battery1.Battery = 8.0f;
	
	Control.Senser.Voltage.Battery2.Max = 8.4f;
	Control.Senser.Voltage.Battery2.Min = 7.0f;
	Control.Senser.Voltage.Battery2.Battery = 8.0f;
	
	Control.Senser.Voltage.Battery3.Max = 8.4f;
	Control.Senser.Voltage.Battery3.Min = 7.0f;
	Control.Senser.Voltage.Battery3.Battery = 8.0f;
	
	Control.Senser.Voltage.Battery4.Max = 8.4f;
	Control.Senser.Voltage.Battery4.Min = 7.0f;
	Control.Senser.Voltage.Battery4.Battery = 8.0f;
	
	MOTOR_PWR_EN();
	MOTOR_EN(0,0x01);//使能是0x01,
	
	
	Control.Car.BaisThrottle = 0;//设定基本油门
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(20);
		
		Control_TaskManage(0.02f,Control.Task.Task_id);
		
		
		//电机输出控制
		/*
		
		
		
		
		*/
		
		if(Control.Command.EmergencyStop == 0x6d)
		{
			Control.Task.Task_id = 0;//清除任务ID
			Control.Car.isunLock = 0x00;//机器上锁
		}
		
		if(Control.Car.isunLock == 0x57)
		{
			/*
			   Control.Task.PositionOutPut
			   Control.Task.HeadingOutPut
			
			   往左为负
			
			
			
			*/
			TIM2->CCR1 = LIMIT(Control.Task.SpeedOutPut + Control.Task.PositionOutPut + Control.Task.HeadingOutPut,0,1000);//左后 1>2正向
			TIM2->CCR2 = 0;//左后 1<2反向
			
			TIM8->CCR3 = LIMIT(Control.Task.SpeedOutPut - Control.Task.PositionOutPut - Control.Task.HeadingOutPut,0,1000);//右后 3>4正向
			TIM8->CCR4 = 0;//右后 3<4反向
		}
		else
		{
			TIM2->CCR1 = 0;//左后 1>2正向
			TIM2->CCR2 = 0;//左后 1<2反向
			
			TIM4->CCR1 = 0;//右前 1>2正向
			TIM4->CCR2 = 0;//右前 1<2反向
			
			TIM4->CCR3 = 0;//左前 3>4正向
			TIM4->CCR4 = 0;//左前 3<4反向
			
			TIM8->CCR3 = 0;//右后 3>4正向
			TIM8->CCR4 = 0;//右后 3<4反向
		}
		
		
		
		osThreadYield();
  }
  /* USER CODE END StartTask02 */
}

/* StartTask03 function */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	
	uint16_t timecount = 0;
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
		timecount+=10;
		//读取超声波
		/*
		if((timecount == 250)&&(Control.Senser.Sonar.forward.isUpdated == 0x00))
		   Ultrasonic_ReadFront();
		else if((timecount == 500)&&(Control.Senser.Sonar.left.isUpdated == 0x00))
       Ultrasonic_ReadLeft();
		else if((timecount == 750)&&(Control.Senser.Sonar.right.isUpdated == 0x00))
		{
       Ultrasonic_ReadRight();
			 timecount = 0;
		}
		*/
		//解码超声波
		/*
		if(Control.Senser.Sonar.forward.isUpdated)
		{
			 Control.Senser.Sonar.forward.isUpdated = 0x00;
			 calculate1(Control.Senser.Sonar.forward.rxbuff);
		}
		
		if(Control.Senser.Sonar.left.isUpdated)
		{
			 Control.Senser.Sonar.left.isUpdated = 0x00;
			 calculate2(Control.Senser.Sonar.left.rxbuff);
		}
		
		if(Control.Senser.Sonar.right.isUpdated)
		{
			 Control.Senser.Sonar.right.isUpdated = 0x00;
			 calculate3(Control.Senser.Sonar.right.rxbuff);
		}
		*/
		//解析GPS
		while(Control.Senser.GPS.r_tail != Control.Senser.GPS.r_head)
		{
			 ublox_Protocol_Prepare(Control.Senser.GPS.rxbuff[Control.Senser.GPS.r_tail]);
			
			 Control.Senser.GPS.r_tail++;
			 if(Control.Senser.GPS.r_tail >= sizeof(Control.Senser.GPS.rxbuff))
			 {
				 Control.Senser.GPS.r_tail = 0;
			 }
		}
		
		osThreadYield();
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
