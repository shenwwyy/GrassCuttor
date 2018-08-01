#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "stm32f4xx_hal.h"



void MOTOR_Init(void);

void MOTOR_PWR_EN(void);
void MOTOR_PWR_DIS(void);

void MOTOR_EN(uint8_t Wheel,uint8_t Status);
void MOTOR_Turn(uint16_t circle_num,uint8_t dir,uint16_t CCR);

extern uint32_t uwDirection;
extern uint32_t pulsecount;
extern uint32_t cur_distance;

#define per_signal_num 			 50//一圈信号数48-50 暂定50
#define per_circle_distance	 65*3.14 //轮子直径65mm，周长单位mm
#define TIM1_Counter_Period 	per_signal_num*100//重装载值;

void Seepd_Reduction(uint8_t flag,uint16_t *CCR);//点刹


#endif

