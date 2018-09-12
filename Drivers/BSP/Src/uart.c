#include "uart.h"
#include "ublox.h"
#include "ultrasonic.h"
#include "usart.h"
#include "control.h"

#include<stdio.h>
#include<string.h> 

uint8_t TxBuffer1[256];
uint8_t TxCounter1=0;
uint8_t count1=0; 
uint8_t RxBuf1[2],RxCount1=0; 

uint8_t TxBuffer2[256];
uint8_t TxCounter2=0;
uint8_t count2=0; 
uint8_t RxBuf2[2],RxCount2=0; 

uint8_t TxBuffer3[256];
uint8_t TxCounter3=0;
uint8_t count3=0; 
uint8_t RxBuf3[2],RxCount3=0; 

uint8_t TxBuffer4[256];
uint8_t TxCounter4=0;
uint8_t count4=0; 
uint8_t RxBuf4,RxCount4=0; 

uint8_t TxBuffer5[256];
uint8_t TxCounter5=0;
uint8_t count5=0; 
uint8_t RxBuf5,RxCount5=0; 

uint8_t TxBuffer6[256];
uint8_t TxCounter6=0;
uint8_t count6=0; 
uint8_t RxBuf6,RxCount6=0; 

uint8_t aRxBuffer = 0x55;
uint8_t Protocol_TxBuff[256];//发送的缓存

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//uint8_t com_data;
  if (huart->Instance == USART1)  {//U1
      Control.Senser.Sonar.forward.isUpdated = 0x01;		
    }
	else if (huart->Instance == USART2)  {//U2
		  Control.Senser.Sonar.left.isUpdated = 0x01;	
    }
	else if (huart->Instance == USART3)  {//U3
		  Control.Senser.Sonar.right.isUpdated = 0x01;	
		
		  	/*for(uint16_t i = 0;i<sizeof(Control.Car.HLink.rbuff);i++)
				{
					Control.Car.HLink.rxbuff[Control.Car.HLink.r_head] = Control.Car.HLink.rbuff[i];
					
					Control.Car.HLink.r_head++;
					if(Control.Car.HLink.r_head>=sizeof(Control.Car.HLink.rxbuff))
					{
						 //如果超过缓冲区大小，那么从头开始存放
						 Control.Car.HLink.r_head = 0;
					}
			  
				}
		*/
		
    }
	else if (huart->Instance == UART4)  {//NONE
		
        //HAL_UART_Receive_IT(huart, &RxBuf4, 1);
    }
	else if (huart->Instance == UART5)  {//NONE
		
		for(uint16_t i = 0;i<sizeof(Control.Car.HLink.rbuff);i++)
		{
			  Control.Car.HLink.rxbuff[Control.Car.HLink.r_head] = Control.Car.HLink.rbuff[i];
			  
			  Control.Car.HLink.r_head++;
			  if(Control.Car.HLink.r_head>=sizeof(Control.Car.HLink.rxbuff))
				{
					 //如果超过缓冲区大小，那么从头开始存放
					 Control.Car.HLink.r_head = 0;
				}
			  
		}
		
		

    }
	else if (huart->Instance == USART6)  {//GPS
				
		for(uint16_t i = 0;i<sizeof(Control.Senser.GPS.rbuff);i++)
		{
			  Control.Senser.GPS.rxbuff[Control.Senser.GPS.r_head] = Control.Senser.GPS.rbuff[i];
			  
			  Control.Senser.GPS.r_head++;
			  if(Control.Senser.GPS.r_head>=sizeof(Control.Senser.GPS.rxbuff))
				{
					 //如果超过缓冲区大小，那么从头开始存放
					 Control.Senser.GPS.r_head = 0;
				}
			  
		}
		
		
		
		
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	  if (huart->Instance == USART1)  {
				//Protocol_T_ultrasonic();
    }
	  else if (huart->Instance == USART2)  {
       // Protocol_T_ultrasonic();
    }
	  else if (huart->Instance == USART3)  {
			 // Protocol_T_ultrasonic();
    }
	  else if (huart->Instance == UART4)  {
        
    }
	  else if (huart->Instance == UART5)  {
					
		}
		else if (huart->Instance == USART6)  {
					
		}
}

void Protocol_SendData(uint8_t *pData,uint16_t Size)  
{
	  for(uint16_t i = 0;i<Size;i++)
	  {
			 Protocol_TxBuff[i] = pData[i];
   	}
	  
	  //HAL_UART_Transmit_DMA(&huart1,Protocol_TxBuff,Size);
		//HAL_UART_Transmit_DMA(&huart2,Protocol_TxBuff,Size);
		//HAL_UART_Transmit_DMA(&huart3,Protocol_TxBuff,Size);
		//HAL_UART_Transmit_DMA(&huart6,Protocol_TxBuff,Size);
}

void UART5_SendData(void)  
{	  
	 if(Control.Car.HLink.t_head != 0)
	 if(HAL_OK == HAL_UART_Transmit_DMA(&huart5,Control.Car.HLink.txbuff,Control.Car.HLink.t_head))
	 {
		 Control.Car.HLink.t_head = 0;
	 }
}



void ublox_Protocol_Send(uint8_t *pData,uint16_t Size)
{
		for(uint16_t i = 0;i<Size;i++)
	  {
			 Protocol_TxBuff[i] = pData[i];
   	}
		HAL_UART_Transmit_DMA(&huart6,Protocol_TxBuff,Size);
}


//void Protocol_T_ultrasonic(void)
//{
//	    uint8_t DataToSend[1];
//	    uint8_t DataCount = 0;
//	
//			//Function;
//	    DataToSend[DataCount++] = 0x55;
//		
//			//Send
//			Protocol_SendData(DataToSend,DataCount);
//}





