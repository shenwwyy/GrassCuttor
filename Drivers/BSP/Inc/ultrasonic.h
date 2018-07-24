#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_
#include "stm32f4xx_hal.h"

void Protocol_Send(uint8_t *data,uint8_t Len);
void bsp_init(void);
void calculate1(uint8_t *data);
void calculate2(uint8_t *data);
void calculate3(uint8_t *data);
void Protocol_Prepare(uint8_t data);
extern uint8_t TxData;

#define TURE_Distance 1
#define ERROR_Distance 0

#endif





