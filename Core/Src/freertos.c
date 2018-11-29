/* USER CODE BEGIN Header */
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
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

#include "uart_fifo.h"
#include "gy952.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
UART_RX_FIFO_t sonar1_rx;
UART_TX_FIFO_t sonar1_tx;
UART_RX_FIFO_t sonar2_rx;
UART_TX_FIFO_t sonar2_tx;
UART_RX_FIFO_t sonar3_rx;
UART_TX_FIFO_t sonar3_tx;
UART_RX_FIFO_t imu_rx;
UART_TX_FIFO_t imu_tx;
UART_RX_FIFO_t dlink_rx;
UART_TX_FIFO_t dlink_tx;
UART_RX_FIFO_t gps_rx;
UART_TX_FIFO_t gps_tx;

/* USER CODE END Variables */
osThreadId ControltTaskHandle;
osThreadId DataLinkTaskHandle;
osThreadId SenserTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
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
	
	//fifo
	UART_RX_FIFO_open(&sonar1_rx,&huart5,100,UART_FIFO_DMA);
	UART_TX_FIFO_open(&sonar1_tx,&huart5,100,UART_FIFO_DMA);
	UART_RX_FIFO_open(&sonar2_rx,&huart2,100,UART_FIFO_DMA);
	UART_TX_FIFO_open(&sonar2_tx,&huart2,100,UART_FIFO_DMA);
	UART_RX_FIFO_open(&sonar3_rx,&huart3,100,UART_FIFO_DMA);
	UART_TX_FIFO_open(&sonar3_tx,&huart3,100,UART_FIFO_DMA);
	
	UART_RX_FIFO_open(&imu_rx,&huart4,100,UART_FIFO_DMA);
	UART_TX_FIFO_open(&imu_tx,&huart4,100,UART_FIFO_DMA);
	
	UART_RX_FIFO_open(&dlink_rx,&huart1,100,UART_FIFO_DMA);
	UART_TX_FIFO_open(&dlink_tx,&huart1,100,UART_FIFO_DMA);
	
	UART_RX_FIFO_open(&gps_rx,&huart6,100,UART_FIFO_DMA);
	UART_TX_FIFO_open(&gps_tx,&huart6,100,UART_FIFO_DMA);
	
	
	//初始化gps
	GY952_Init();
	
	ubloxInit();
	
	
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
  osThreadDef(DataLinkTask, StartTask02, osPriorityBelowNormal, 0, 256);
  DataLinkTaskHandle = osThreadCreate(osThread(DataLinkTask), NULL);

  /* definition and creation of SenserTask */
  osThreadDef(SenserTask, StartTask03, osPriorityAboveNormal, 0, 256);
  SenserTaskHandle = osThreadCreate(osThread(SenserTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the ControltTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	
	Parameter_R_PID();//读取参数
	

	//超声波接收调用
	//HAL_UART_Receive_DMA(&huart1,Control.Senser.Sonar.forward.rxbuff,sizeof(Control.Senser.Sonar.forward.rxbuff));
//	HAL_UART_Receive_DMA(&huart2,Control.Senser.Sonar.left.rxbuff,sizeof(Control.Senser.Sonar.left.rxbuff));
//	HAL_UART_Receive_DMA(&huart1,HAL_IO.U5.rxbuf,sizeof(HAL_IO.U5.rxbuf));
//	HAL_UART_Receive_DMA(&huart3,Control.Senser.Sonar.right.rxbuff,sizeof(Control.Senser.Sonar.right.rxbuff));
	
	//数据链接收调用
	//normal
	//HAL_UART_Receive_DMA(&huart5,HAL_IO.U5.rxbuf,sizeof(HAL_IO.U5.rxbuf));
	//test
	//HAL_UART_Receive_DMA(&huart2,HAL_IO.U5.rxbuf,sizeof(HAL_IO.U5.rxbuf));
	
	
//	GPS_USART6_UART_Init(9600);
//	
//	
//	//GPS接收调用
//	HAL_UART_Receive_DMA(&huart6,Control.Senser.GPS.rbuff,sizeof(Control.Senser.GPS.rbuff));
	
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);//50ms
		//LED_Toggle(0);
		
		//当前动作运行完成后，开始切换任务，让其他任务得以运行		
		//数传发送
    Protocol_T_Parameter();
		Protocol_T_WayPoint();
		Protocol_T_Status(10);
	  Protocol_T_GPS(10);
		Protocol_T_Par(10);
		//数传解码
		Protocol_Rev();
		
		//保存数据
		if(Control.Parameter.isSaveParameter == 0x01)
		{
			 Control.Parameter.isSaveParameter = 0;
			 Flash_EraseSectors();//写之前擦除
			 Parameter_S_PID();
		}
		
		//超声波解码
		
		
		//IMU解码
		GY952_Rev();
		//GPS解码
		ublox_Rev();
		
		osThreadYield();
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the DataLinkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	
	uint8_t DelayTime = 2;
	uint32_t PreviousWakeTime;
	
	
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
	
	HAL_IO.Satuts.voltage1 = Control.Senser.Voltage.Battery1.Battery;
	HAL_IO.Satuts.voltage2 = Control.Senser.Voltage.Battery2.Battery;
	HAL_IO.Satuts.voltage3 = Control.Senser.Voltage.Battery3.Battery;
	HAL_IO.Satuts.voltage4 = Control.Senser.Voltage.Battery4.Battery;
	
	
	MOTOR_PWR_EN();
	MOTOR_EN(0,0x01);//使能是0x01,
	
	
	Control.Car.BaisThrottle = 0;//设定基本油门
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(20);//50hz
		
		Control_TaskManage(0.02f,Control.Task.Task_id);
		
		//HAL_IO.Satuts.currentwaypoint++;
		
		if(Control.Command.EmergencyStop == 0x6d)//进入紧急停止模式
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
//			TIM2->CCR1 = LIMIT(Control.Task.SpeedOutPut + Control.Task.PositionOutPut + Control.Task.HeadingOutPut,0,1000);//左后 1>2正向
//			TIM2->CCR2 = 0;//左后 1<2反向
//			
//			TIM8->CCR3 = LIMIT(Control.Task.SpeedOutPut - Control.Task.PositionOutPut - Control.Task.HeadingOutPut,0,1000);//右后 3>4正向
//			TIM8->CCR4 = 0;//右后 3<4反向

      TIM2->CCR1 = LIMIT(Control.Task.SpeedOutPut - Control.Task.PositionOutPut - Control.Task.HeadingOutPut,0,1000);//左后 1>2正向
			TIM2->CCR2 = 0;//左后 1<2反向
			
			TIM4->CCR1 = 0;//右前 1>2正向
			TIM4->CCR2 = 0;//右前 1<2反向
			
			TIM4->CCR3 = 0;//左前 3>4正向
			TIM4->CCR4 = 0;//左前 3<4反向
			
			TIM8->CCR3 = LIMIT(Control.Task.SpeedOutPut + Control.Task.PositionOutPut + Control.Task.HeadingOutPut,0,1000);//右后 3>4正向
			TIM8->CCR4 = 0;//右后 3<4反向			

		}
		else
		{
			
			switch(Control.Car.WheelTest)
			{
				case 0x00:
								TIM2->CCR1 = 0;//左后 1>2正向
								TIM2->CCR2 = 0;//左后 1<2反向
								
								TIM4->CCR1 = 0;//右前 1>2正向
								TIM4->CCR2 = 0;//右前 1<2反向
								
								TIM4->CCR3 = 0;//左前 3>4正向
								TIM4->CCR4 = 0;//左前 3<4反向
								
								TIM8->CCR3 = 0;//右后 3>4正向
								TIM8->CCR4 = 0;//右后 3<4反向
				break;
				case 0x01:
					      TIM2->CCR1 = 0;//左后 1>2正向
								TIM2->CCR2 = 0;//左后 1<2反向
								
								TIM4->CCR1 = 0;//右前 1>2正向
								TIM4->CCR2 = 0;//右前 1<2反向
								
								TIM4->CCR3 = 1000;//左前 3>4正向
								TIM4->CCR4 = 0;//左前 3<4反向
								
								TIM8->CCR3 = 0;//右后 3>4正向
								TIM8->CCR4 = 0;//右后 3<4反向
				break;
				case 0x10:
					      TIM2->CCR1 = 0;//左后 1>2正向
								TIM2->CCR2 = 0;//左后 1<2反向
								
								TIM4->CCR1 = 0;//右前 1>2正向
								TIM4->CCR2 = 0;//右前 1<2反向
								
								TIM4->CCR3 = 0;//左前 3>4正向
								TIM4->CCR4 = 1000;//左前 3<4反向
								
								TIM8->CCR3 = 0;//右后 3>4正向
								TIM8->CCR4 = 0;//右后 3<4反向
				break;
				case 0x02:
					      TIM2->CCR1 = 1000;//左后 1>2正向
								TIM2->CCR2 = 0;//左后 1<2反向
								
								TIM4->CCR1 = 0;//右前 1>2正向
								TIM4->CCR2 = 0;//右前 1<2反向
								
								TIM4->CCR3 = 0;//左前 3>4正向
								TIM4->CCR4 = 0;//左前 3<4反向
								
								TIM8->CCR3 = 0;//右后 3>4正向
								TIM8->CCR4 = 0;//右后 3<4反向
				break;
				case 0x20:
					      TIM2->CCR1 = 0;//左后 1>2正向
								TIM2->CCR2 = 1000;//左后 1<2反向
								
								TIM4->CCR1 = 0;//右前 1>2正向
								TIM4->CCR2 = 0;//右前 1<2反向
								
								TIM4->CCR3 = 0;//左前 3>4正向
								TIM4->CCR4 = 0;//左前 3<4反向
								
								TIM8->CCR3 = 0;//右后 3>4正向
								TIM8->CCR4 = 0;//右后 3<4反向
				break;
				case 0x04:
					      TIM2->CCR1 = 0;//左后 1>2正向
								TIM2->CCR2 = 0;//左后 1<2反向
								
								TIM4->CCR1 = 1000;//右前 1>2正向
								TIM4->CCR2 = 0;//右前 1<2反向
								
								TIM4->CCR3 = 0;//左前 3>4正向
								TIM4->CCR4 = 0;//左前 3<4反向
								
								TIM8->CCR3 = 0;//右后 3>4正向
								TIM8->CCR4 = 0;//右后 3<4反向
				break;
				case 0x40:
					      TIM2->CCR1 = 0;//左后 1>2正向
								TIM2->CCR2 = 0;//左后 1<2反向
								
								TIM4->CCR1 = 0;//右前 1>2正向
								TIM4->CCR2 = 1000;//右前 1<2反向
								
								TIM4->CCR3 = 0;//左前 3>4正向
								TIM4->CCR4 = 0;//左前 3<4反向
								
								TIM8->CCR3 = 0;//右后 3>4正向
								TIM8->CCR4 = 0;//右后 3<4反向
				break;
				case 0x08:
					      TIM2->CCR1 = 0;//左后 1>2正向
								TIM2->CCR2 = 0;//左后 1<2反向
								
								TIM4->CCR1 = 0;//右前 1>2正向
								TIM4->CCR2 = 0;//右前 1<2反向
								
								TIM4->CCR3 = 0;//左前 3>4正向
								TIM4->CCR4 = 0;//左前 3<4反向
								
								TIM8->CCR3 = 1000;//右后 3>4正向
								TIM8->CCR4 = 0;//右后 3<4反向
				break;
				case 0x80:
					      TIM2->CCR1 = 0;//左后 1>2正向
								TIM2->CCR2 = 0;//左后 1<2反向
								
								TIM4->CCR1 = 0;//右前 1>2正向
								TIM4->CCR2 = 0;//右前 1<2反向
								
								TIM4->CCR3 = 0;//左前 3>4正向
								TIM4->CCR4 = 0;//左前 3<4反向
								
								TIM8->CCR3 = 0;//右后 3>4正向
								TIM8->CCR4 = 1000;//右后 3<4反向
				break;
			}
		}
			//记录数据
			HAL_IO.Satuts.leftfront_p = TIM4->CCR3; 
			HAL_IO.Satuts.leftfront_n = TIM4->CCR4; 
			HAL_IO.Satuts.leftback_p  = TIM2->CCR1; 
			HAL_IO.Satuts.leftback_n  = TIM2->CCR2; 
			
			HAL_IO.Satuts.rightfront_p = TIM4->CCR1; 
			HAL_IO.Satuts.rightfront_n = TIM4->CCR2; 
			HAL_IO.Satuts.rightback_p  = TIM8->CCR3; 
			HAL_IO.Satuts.rightback_n  = TIM8->CCR4;
			
			
			

		
		osThreadYield();
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the SenserTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
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
//		while(Control.Senser.GPS.r_tail != Control.Senser.GPS.r_head)
//		{
//			 ublox_Protocol_Prepare(Control.Senser.GPS.rxbuff[Control.Senser.GPS.r_tail]);
//			
//			 Control.Senser.GPS.r_tail++;
//			 if(Control.Senser.GPS.r_tail >= sizeof(Control.Senser.GPS.rxbuff))
//			 {
//				 Control.Senser.GPS.r_tail = 0;
//			 }
//		}
		
		osThreadYield();
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
