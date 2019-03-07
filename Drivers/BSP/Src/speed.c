#include "speed.h"


#define foreward 1 //正转
#define inversion 0 //反转
#define stop 2 //反转

#define dia 65 //直径65mm（包括胎皮）
#define pi 3.1415 
#define per_signal 220 //一圈11个信号
#define reset_time 10 //检测时间，可调节


float per_distance ;
float per_distance =  pi * dia ;//计算一圈长度，单位mm

float TIM1_total_distance ;
float TIM2_total_distance ;

char TIM1_dir ;
char TIM2_dir ;

float TIM1_speed ;
float TIM2_speed ;

int encode_TIM1_num ;
int encode_TIM2_num ;

float circle_TIM1_num ;
float circle_TIM2_num ;

void SPEED_CAL(void)
{		
	encode_TIM1_num  = TIM1_Encoder_Read();
	encode_TIM2_num  = TIM2_Encoder_Read();
	
	per_distance =  pi * dia ;//计算一圈长度，单位mm
	
	TIM1_Encoder_dir(encode_TIM1_num);	//判断正反转
	TIM2_Encoder_dir(encode_TIM2_num);	//判断正反转
	
	if (encode_TIM1_num < 0) {
		encode_TIM1_num = ~encode_TIM1_num +1;} //转换正数
	
	if (encode_TIM2_num < 0) {
		encode_TIM2_num = ~encode_TIM2_num +1;} //转换正数
	
	circle_TIM1_num =(float)encode_TIM1_num / per_signal; //reset_time时间内转的圈数
	TIM1_total_distance = circle_TIM1_num * per_distance;//reset_time时间内的距离，单位mm
	TIM1_speed = TIM1_total_distance / reset_time ;//计算速度，单位mm/ms = m/s
	
	circle_TIM2_num =(float)encode_TIM2_num / per_signal; //reset_time时间内转的圈数
	TIM2_total_distance = circle_TIM2_num * per_distance;//reset_time时间内的距离，单位mm
	TIM2_speed = TIM2_total_distance / reset_time ;//计算速度，单位mm/ms = m/s
}


//判断正反转
int  TIM1_Encoder_dir(int encode_num)
{
	if(encode_num < 0) TIM1_dir = inversion;
	else if (encode_num> 0)	TIM1_dir = foreward;
	else if (encode_num == 0)	TIM1_dir = stop;
}

int  TIM2_Encoder_dir(int encode_num)
{
	if(encode_num < 0) TIM2_dir = inversion;
	else if (encode_num> 0)	TIM2_dir = foreward;
	else if (encode_num == 0)	TIM2_dir = stop;
}

//计数寄存器赋值
void TIM1_Encoder_Write(int data)
{
		TIM1->CNT = data;
}

void TIM2_Encoder_Write(int data)
{
		TIM2->CNT = data;
}

//读计数个数
int TIM1_Encoder_Read(void)
{
		TIM1_Encoder_Write(0);        //计数器清0
		HAL_Delay(reset_time);          //检测时间，可调节
		return (int)((int16_t)(TIM1->CNT));           //数据类型转换
														 //记录边沿变化次数（一个栅格被记录4次）
}

int TIM2_Encoder_Read(void)
{
		TIM2_Encoder_Write(0);        //计数器清0
		HAL_Delay(reset_time);          //检测时间，可调节
		return (int)((int16_t)(TIM2->CNT));           //数据类型转换
														 //记录边沿变化次数（一个栅格被记录4次）
}









