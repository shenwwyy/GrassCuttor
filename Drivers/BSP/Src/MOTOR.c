#include "motor.h"




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






void MOTOR_Speed()
{
	
}

void MOTOR_DIR()
{
	
}








