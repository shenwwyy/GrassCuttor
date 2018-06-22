#include "motor.h"



uint16_t Wheel_Right_fore,Wheel_Right_Back;
uint16_t Wheel_Left_fore,Wheel_Left_Back;


void (*MOTOR_BRAKE)(void *);
void (*MOTOR_FOREWORD)(void *);
void (*MOTOR_BACK)(void *);
void (*MOTOR_LEFT)(void *);
void (*MOTOR_RIGHT)(void *);



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






void MOTOR_Speed(float wheel1,float wheel2,float wheel3,float wheel4)
{
	 Wheel_Right_fore = wheel1;
	 Wheel_Right_Back = wheel2;
   Wheel_Left_fore  = wheel3;
	 Wheel_Left_Back  = wheel4;
}

void MOTOR_DIR()
{
	 
}

void MOTOR_Update(void)
{
	   MOTOR_DIR();
}






