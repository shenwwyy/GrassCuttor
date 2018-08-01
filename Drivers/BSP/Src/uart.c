#include "uart.h"
#include "ublox.h"
#include "ultrasonic.h"
#include "usart.h"


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
  if (huart->Instance == USART1)  {
      
			HAL_UART_Receive_DMA(&huart1, (uint8_t *)RxBuf1, 2);
			calculate1(RxBuf1);			
    }
	else if (huart->Instance == USART2)  {
		    
      HAL_UART_Receive_DMA(&huart2, (uint8_t *)RxBuf2, 2);//继续下一个字节
			calculate2(RxBuf2);
    }
	else if (huart->Instance == USART3)  {
		
      HAL_UART_Receive_DMA(&huart3, (uint8_t *)RxBuf3, 2);//继续下一个字节
			calculate3(RxBuf3);
    }
	else if (huart->Instance == UART4)  {
		
        HAL_UART_Receive_IT(huart, &RxBuf4, 1);
    }
	else if (huart->Instance == UART5)  {

    }
	else if (huart->Instance == USART6)  {
				
		HAL_UART_Receive_IT(huart, &RxBuf6, 1);
		ublox_Protocol_Prepare(RxBuf6);
			}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	  if (huart->Instance == USART1)  {
				Protocol_T_ultrasonic();
    }
	  else if (huart->Instance == USART2)  {
       Protocol_T_ultrasonic();
    }
	  else if (huart->Instance == USART3)  {
			Protocol_T_ultrasonic();
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
	  
	  HAL_UART_Transmit_DMA(&huart1,Protocol_TxBuff,Size);
		HAL_UART_Transmit_DMA(&huart6,Protocol_TxBuff,Size);
}

void ublox_Protocol_Send(uint8_t *pData,uint16_t Size)
{
		 for(uint16_t i = 0;i<Size;i++)
	  {
			 Protocol_TxBuff[i] = pData[i];
   	}
		HAL_UART_Transmit_DMA(&huart6,Protocol_TxBuff,Size);
}


void Protocol_T_ultrasonic(void)
{
	    uint8_t DataToSend[1];
	    uint8_t DataCount = 0;
	
			//Function;
	    DataToSend[DataCount++] = 0x55;
		
			//Send
			Protocol_SendData(DataToSend,DataCount);
}





