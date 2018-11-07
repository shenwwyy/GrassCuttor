#include "uart.h"
#include "ublox.h"
#include "ultrasonic.h"
#include "usart.h"
#include "control.h"
#include "protocol.h"
#include <stdio.h>
#include <string.h> 

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
			
    }
	else if (huart->Instance == UART4)  {//NONE
		
    }
	else if (huart->Instance == UART5)  {//DLINK

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
				HAL_IO.U1.isReadyToSend = 0x00;
			  HAL_IO.U5.isReadyToSend = 0x00;
    }
	  else if (huart->Instance == USART2)  {
       HAL_IO.U2.isReadyToSend = 0x00;
    }
	  else if (huart->Instance == USART3)  {
			 HAL_IO.U3.isReadyToSend = 0x00;
    }
	  else if (huart->Instance == UART4)  {
       HAL_IO.U4.isReadyToSend = 0x00;
    }
	  else if (huart->Instance == UART5)  {
			 HAL_IO.U5.isReadyToSend = 0x00;
		}
		else if (huart->Instance == USART6)  {
			 HAL_IO.U6.isReadyToSend = 0x00;
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



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(&huart1 == huart)
	{		
//		HAL_UART_Receive_DMA(&huart1,HAL_IO.U1.rxbuf,sizeof(HAL_IO.U1.rxbuf));
//	  HAL_IO.U1.rxtail = 0;
//		
//		HAL_IO.U1.isReadyToSend = 0x00;
				HAL_UART_Receive_DMA(&huart1,HAL_IO.U5.rxbuf,sizeof(HAL_IO.U5.rxbuf));
	  HAL_IO.U5.rxtail = 0;
		
		if(HAL_IO.U5.isReadyToSend == 0x01)
		{
		  HAL_IO.U5.isReadyToSend = 0x00;
		}
	}
	else if(&huart2 == huart)
	{		
		
//		HAL_UART_Receive_DMA(&huart2,HAL_IO.U2.rxbuf,sizeof(HAL_IO.U2.rxbuf));
//	  HAL_IO.U2.rxtail = 0;
//		
//		if(HAL_IO.U2.isReadyToSend == 0x01)
//		{
//		  HAL_IO.U2.isReadyToSend = 0x00;
//		}

		
	}
	else if(&huart3 == huart)
	{		
//		HAL_UART_Receive_DMA(&huart3,HAL_IO.U3.rxbuf,sizeof(HAL_IO.U3.rxbuf));
//	  HAL_IO.U3.rxtail = 0;
//		
//		HAL_IO.U3.isReadyToSend = 0x00;
	}
	else if(&huart4 == huart)
	{		
		
		HAL_UART_Receive_DMA(&huart4,HAL_IO.U4.rxbuf,sizeof(HAL_IO.U4.rxbuf));
	  HAL_IO.U4.rxtail = 0;
		
		if(HAL_IO.U4.isReadyToSend == 0x01)
		{
		  HAL_IO.U4.isReadyToSend = 0x00;
		}
	}
	else if(&huart5 == huart)
	{		
		HAL_UART_Receive_DMA(&huart5,HAL_IO.U5.rxbuf,sizeof(HAL_IO.U5.rxbuf));
	  HAL_IO.U5.rxtail = 0;
		
		if(HAL_IO.U5.isReadyToSend == 0x01)
		{
		  HAL_IO.U5.isReadyToSend = 0x00;
		}
		
	}
	else if(&huart6 == huart)
	{		
		
//		HAL_UART_Receive_DMA(&huart6,HAL_IO.U6.rxbuf,sizeof(HAL_IO.U6.rxbuf));
//	  HAL_IO.U6.rxtail = 0;
//		
//		if(HAL_IO.U6.isReadyToSend == 0x01)
//		{
//		  HAL_IO.U6.isReadyToSend = 0x00;
//		}
	}
	
	
}


