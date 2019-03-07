
#ifndef __speed_H
#define __speed_H


#include "main.h"


void SPEED_CAL(void);
void TIM1_Encoder_Write(int data);
int  TIM1_Encoder_Read(void);
int  TIM1_Encoder_dir(int encode_num);

void TIM2_Encoder_Write(int data);
int	 TIM2_Encoder_Read(void);
int  TIM2_Encoder_dir(int encode_num);

extern float TIM1_total_distance ;
extern float TIM2_total_distance ;

extern float TIM1_speed ;
extern float TIM2_speed ;

extern char TIM1_dir ;
extern char TIM2_dir ;

extern	int encode_TIM1_num ;
extern	int encode_TIM2_num ;

extern	float circle_TIM1_num ;
extern	float circle_TIM2_num ;

extern float per_distance;
	
#endif













