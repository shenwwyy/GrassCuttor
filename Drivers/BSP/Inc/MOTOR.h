

#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "stm32f4xx_hal.h"


typedef struct {
	
	uint8_t dir;
	float   speed;
	
}_motorDef;



void MOTOR_Init(void);

void MOTOR_PWR_EN(void);
void MOTOR_PWR_DIS(void);

void MOTOR_EN(uint8_t Wheel,uint8_t Status);

















#endif

