#include "motor.h"
#include "tim.h"


uint32_t uwDirection = 0;
uint32_t pulsecount = 0;
uint32_t cur_distance = 0;
uint32_t set_pulsecount = 0;
uint32_t reverse_pulsecount = 0;
uint32_t ahead_pulsecount = 0;
uint8_t i ;

void MOTOR_Init(void)
{
	  
}


void MOTOR_PWR_EN(void)
{
	  HAL_GPIO_WritePin(PWR_EN1_GPIO_Port,PWR_EN1_Pin,GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(PWR_EN2_GPIO_Port,PWR_EN2_Pin,GPIO_PIN_RESET);
}

void MOTOR_PWR_DIS(void)
{
	  HAL_GPIO_WritePin(PWR_EN1_GPIO_Port,PWR_EN1_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(PWR_EN2_GPIO_Port,PWR_EN2_Pin,GPIO_PIN_SET);
}

//#define MOTOR_EN1_Pin GPIO_PIN_2
//#define MOTOR_EN1_GPIO_Port GPIOB
//#define MOTOR_EN2_Pin GPIO_PIN_7
//#define MOTOR_EN2_GPIO_Port GPIOE
//#define MOTOR_EN3_Pin GPIO_PIN_10
//#define MOTOR_EN3_GPIO_Port GPIOD
//#define MOTOR_EN4_Pin GPIO_PIN_11
//#define MOTOR_EN4_GPIO_Port GPIOD


void MOTOR_EN(uint8_t Wheel,uint8_t Status)
{
	if(Status == 0x01)
	{
		switch(Wheel)
		{
			case 0:
			{
				 HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(MOTOR_EN3_GPIO_Port,MOTOR_EN3_Pin,GPIO_PIN_RESET);
				 HAL_GPIO_WritePin(MOTOR_EN4_GPIO_Port,MOTOR_EN4_Pin,GPIO_PIN_RESET);
			}break;
			case 1:
			{
				 HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,GPIO_PIN_RESET);
			}break;
			case 2:
			{
				 HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,GPIO_PIN_RESET);
			}break;
			case 3:
			{
				 HAL_GPIO_WritePin(MOTOR_EN3_GPIO_Port,MOTOR_EN3_Pin,GPIO_PIN_RESET);
			}break;
			case 4:
			{
				 HAL_GPIO_WritePin(MOTOR_EN4_GPIO_Port,MOTOR_EN4_Pin,GPIO_PIN_RESET);
			}break;
			default:break;

		}
	}
	else
	{
		 switch(Wheel)
		 {
				case 0:
				{
				 HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,GPIO_PIN_SET);
				 HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,GPIO_PIN_SET);
				 HAL_GPIO_WritePin(MOTOR_EN3_GPIO_Port,MOTOR_EN3_Pin,GPIO_PIN_SET);
				 HAL_GPIO_WritePin(MOTOR_EN4_GPIO_Port,MOTOR_EN4_Pin,GPIO_PIN_SET);
			  }break;
				case 1:
				{
					 HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port,MOTOR_EN1_Pin,GPIO_PIN_SET);
				}break;
				case 2:
				{
					 HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port,MOTOR_EN2_Pin,GPIO_PIN_SET);
				}break;
				case 3:
				{
					 HAL_GPIO_WritePin(MOTOR_EN3_GPIO_Port,MOTOR_EN3_Pin,GPIO_PIN_SET);
				}break;
				case 4:
				{
					 HAL_GPIO_WritePin(MOTOR_EN4_GPIO_Port,MOTOR_EN4_Pin,GPIO_PIN_SET);
				}break;
				default:break;

		 }
	}
}






void MOTOR_Speed()
{
	
}

void MOTOR_DIR(uint8_t dir)
{
	switch (dir)
	{
		case 0:
			
			break;
		case 1:
			
			break;
		default:
			break;
	}
}

void MOTOR_Turn(uint16_t circle_num,uint8_t dir,uint16_t CCR)
{	
	set_pulsecount = circle_num * per_signal_num;  					//设置所转圈数对应的pulsecount值
	reverse_pulsecount = TIM1_Counter_Period - pulsecount;	//反转时，设置所转圈数对应的pulsecount值
	ahead_pulsecount = set_pulsecount - per_signal_num;
	
	uwDirection = __HAL_TIM_DIRECTION_STATUS(&htim1);
	pulsecount = __HAL_TIM_GetCounter(&htim1);


	switch (dir)
	{
		case 0:{//正转
			HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_RESET);			
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);//关闭PWM2
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//开启PWM1
			TIM3->CCR1 = CCR;
			
		
			if(pulsecount >= ahead_pulsecount)
			{
//			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_ALL);//关闭PW
//			//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_ALL);//关闭PWM
//			//HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_ALL);//关闭PWM
////			for (i=CCR;i<=0;i-=80)
////			{
////				TIM3->CCR1 = i;
////		  }
//			//HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_SET);//制动
		
			Seepd_Reduction(0,&CCR);
		
				if(pulsecount >= set_pulsecount - 80)
				{  
					HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_SET);//制动
					//HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_SET);//制动
					//HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_ALL);//关闭PWM
				//__HAL_TIM_SetCounter(&htim8,0);
				}
			//__HAL_TIM_SetCounter(&htim8,0);
		 }
	}
			break;
			
		case 1:{//反转
			HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);//关闭PWM1
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);//开启PWM2
			TIM3->CCR2 = CCR;
			
			if(reverse_pulsecount == set_pulsecount)
			{  
					HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_SET);//制动
					HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_SET);//制动
				__HAL_TIM_SetCounter(&htim1,0);
			}
		}
			break;
		
		case 2:{//制动
			HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_SET);//制动
			HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_SET);//制动
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_ALL);}//关闭PWM
		default:
			break;
	}
}

void Driving_Distance(uint32_t set_distance)
{
//	cur_distance =pulsecount / per_signal_num * per_circle_distance ;//计算当前行驶具体
}


void Seepd_Reduction(uint8_t flag,uint16_t *CCR)//点刹
{
	switch (flag)
	{
		case 0://停止
		{
			for(i=0;i<=0;i-=40)
			{
				TIM8->CCR1 = i;
			}
		}
		 break;
		
		case 1://启动
		{
		
		
		}
		 break;
		
		default:
			break;
	}
}
	



