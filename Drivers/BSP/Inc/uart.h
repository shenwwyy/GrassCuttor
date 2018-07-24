#ifndef __UART_H
#define __UART_H	 
#include "stm32f4xx_hal.h"
#include "main.h"

void Usart3_Init(uint32_t br_num);
void Usart3_IRQ(void);
//void Usart3_Send(unsigned char *DataToSend ,uint16_t data_num);	  	
void Protocol_T_ultrasonic(void);
void Protocol_SendData(uint8_t *pData,uint16_t Size) ;
void ublox_Protocol_Send(uint8_t *pData,uint16_t Size);

//extern uint8_t Protocol_TxBuff[256];

#endif



