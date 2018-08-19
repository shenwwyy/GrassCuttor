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
	set_pulsecount = circle_num * per_signal_num;  					//������תȦ����Ӧ��pulsecountֵ
	reverse_pulsecount = TIM1_Counter_Period - pulsecount;	//��תʱ��������תȦ����Ӧ��pulsecountֵ
	ahead_pulsecount = set_pulsecount - per_signal_num;
	
	uwDirection = __HAL_TIM_DIRECTION_STATUS(&htim1);
	pulsecount = __HAL_TIM_GetCounter(&htim1);


	switch (dir)
	{
		case 0:{//��ת
			HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_RESET);			
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);//�ر�PWM2
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);//����PWM1
			TIM3->CCR1 = CCR;
			
		
			if(pulsecount >= ahead_pulsecount)
			{
//			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_ALL);//�ر�PW
//			//HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_ALL);//�ر�PWM
//			//HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_ALL);//�ر�PWM
////			for (i=CCR;i<=0;i-=80)
////			{
////				TIM3->CCR1 = i;
////		  }
//			//HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_SET);//�ƶ�
		
			Seepd_Reduction(0,&CCR);
		
				if(pulsecount >= set_pulsecount - 80)
				{  
					HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_SET);//�ƶ�
					//HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_SET);//�ƶ�
					//HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_ALL);//�ر�PWM
				//__HAL_TIM_SetCounter(&htim8,0);
				}
			//__HAL_TIM_SetCounter(&htim8,0);
		 }
	}
			break;
			
		case 1:{//��ת
			HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_RESET);
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);//�ر�PWM1
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);//����PWM2
			TIM3->CCR2 = CCR;
			
			if(reverse_pulsecount == set_pulsecount)
			{  
					HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_SET);//�ƶ�
					HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_SET);//�ƶ�
				__HAL_TIM_SetCounter(&htim1,0);
			}
		}
			break;
		
		case 2:{//�ƶ�
			HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_SET);//�ƶ�
			HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_SET);//�ƶ�
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_ALL);}//�ر�PWM
		default:
			break;
	}
}

void Driving_Distance(uint32_t set_distance)
{
//	cur_distance =pulsecount / per_signal_num * per_circle_distance ;//���㵱ǰ��ʻ����
}


void Seepd_Reduction(uint8_t flag,uint16_t *CCR)//��ɲ
{
	switch (flag)
	{
		case 0://ֹͣ
		{
			for(i=0;i<=0;i-=40)
			{
				TIM8->CCR1 = i;
			}
		}
		 break;
		
		case 1://����
		{
		
		
		}
		 break;
		
		default:
			break;
	}
}
	



