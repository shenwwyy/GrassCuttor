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

#include "ublox.h"
#include "uart.h"
#include "ultrasonic.h"

#include "control.h"

#include "stm32f4xx_hal.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include "protocol.h"

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
	
  //PWM��ʼ��
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
	
	
	//��Ӵ�������ʼ��
//	ubloxInitGps();
//	
//	HAL_UART_RxCpltCallback(&huart1);
//	HAL_UART_TxCpltCallback(&huart1);
//	
//	HAL_UART_RxCpltCallback(&huart2);
//	HAL_UART_TxCpltCallback(&huart2);
//	
//	HAL_UART_RxCpltCallback(&huart3);
//	HAL_UART_TxCpltCallback(&huart3);
	
	//���������յ���
	HAL_UART_Receive_DMA(&huart1,Control.Senser.Sonar.forward.rxbuff,sizeof(Control.Senser.Sonar.forward.rxbuff));
	HAL_UART_Receive_DMA(&huart2,Control.Senser.Sonar.left.rxbuff,sizeof(Control.Senser.Sonar.left.rxbuff));
	HAL_UART_Receive_DMA(&huart3,Control.Senser.Sonar.right.rxbuff,sizeof(Control.Senser.Sonar.right.rxbuff));
	
	//���������յ���
	HAL_UART_Receive_DMA(&huart5,Control.Car.HLink.rbuff,sizeof(Control.Car.HLink.rbuff));
	
	
	GPS_USART6_UART_Init(9600);
	
	
	//GPS���յ���
	HAL_UART_Receive_DMA(&huart6,Control.Senser.GPS.rbuff,sizeof(Control.Senser.GPS.rbuff));
	ubloxInitGps();
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);//50ms
		LED_Toggle(0);
		
		//��ǰ����������ɺ󣬿�ʼ�л����������������������
		
		
		//���봫������ȡ
		//������1��ȡ
		//��ʱ250ms
		//osDelay(250);
		
		//������2��ȡ
		//��ʱ250ms
		//osDelay(250);
		
		//������3��ȡ
		//��ʱ250ms
		//osDelay(250);
		
		
		//��������
		Protocol_Transmit(0.1f);
		//��������
		while(Control.Car.HLink.r_tail != Control.Car.HLink.r_head)
		{
			 Protocol_Prepare(Control.Car.HLink.rxbuff[Control.Car.HLink.r_tail]);
			
			 Control.Car.HLink.r_tail++;
			 if(Control.Car.HLink.r_tail >= sizeof(Control.Car.HLink.rxbuff))
			 {
				 Control.Car.HLink.r_tail = 0;
			 }
		}
		
		
		osThreadYield();
  }
  /* USER CODE END StartDefaultTask */
}

/* StartTask02 function */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	
	

	

	

	
//	MOTOR_PWR_EN();
//	
//	MOTOR_EN(0,0x00);//ʹ����0x01,
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(20);
		Control_TaskManage(0.02f,Control.Task.Task_id);
		
		
		
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
		//��ȡ������
		if((timecount == 250)&&(Control.Senser.Sonar.forward.isUpdated == 0x00))
		   Ultrasonic_ReadFront();
		else if((timecount == 500)&&(Control.Senser.Sonar.left.isUpdated == 0x00))
       Ultrasonic_ReadLeft();
		else if((timecount == 750)&&(Control.Senser.Sonar.right.isUpdated == 0x00))
		{
       Ultrasonic_ReadRight();
			 timecount = 0;
		}
		//���볬����
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
		
		//����GPS
		while(Control.Senser.GPS.r_tail != Control.Senser.GPS.r_head)
		{
			 ublox_Protocol_Prepare(Control.Senser.GPS.rxbuff[Control.Senser.GPS.r_tail]);
			
			 Control.Senser.GPS.r_tail++;
			 if(Control.Senser.GPS.r_tail >= sizeof(Control.Senser.GPS.rxbuff))
			 {
				 Control.Senser.GPS.r_tail = 0;
			 }
		}
		
		
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
