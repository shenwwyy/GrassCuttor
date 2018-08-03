#include "ultrasonic.h"
#include "usart.h"
#include "uart.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "CONTROL.H"

uint8_t HighLen1 = 0;
uint8_t LowLen1 = 0;
uint16_t Len_mm1 = 0;
uint8_t flag1;

uint8_t HighLen2 = 0;
uint8_t LowLen2= 0;
uint16_t Len_mm2 = 0;
uint8_t flag2;

uint8_t HighLen3 = 0;
uint8_t LowLen3 = 0;
uint16_t Len_mm3 = 0;
uint8_t flag3;


uint8_t rxbuf[256];
uint8_t TxData=0x55;

uint16_t ture_distance1;
uint16_t ture_distance2;
uint16_t ture_distance3;

void bsp_init()
{
//	Front_USART1_Init(9600);
//	Left_USART2_Init(9600);
//	Right_USART3_Init(9600);
//	GPS_UART4_Init(9600);
//	Bluetooth_UART5_Init(9600);
	
//	HAL_UART_RxCpltCallback(&huart1);
//	HAL_UART_RxCpltCallback(&huart2);
//	HAL_UART_RxCpltCallback(&huart3);
//	HAL_UART_RxCpltCallback(&huart4);
//	HAL_UART_RxCpltCallback(&huart5);
	
	HAL_GPIO_WritePin(MOTOR_EN1_GPIO_Port, MOTOR_EN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_EN2_GPIO_Port, MOTOR_EN2_Pin, GPIO_PIN_RESET);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
   //初始化计数器值为0
   __HAL_TIM_SetCounter(&htim1,0);
}


void calculate1(uint8_t *data)
{
		union {uint8_t B[2]; uint8_t Hdata; uint8_t Ldata;}src;	
		
		src.B[0] = data[0];
		src.B[1] = data[1];
		
		HighLen1 = src.B[0];
		LowLen1 = src.B[1];
		
		Len_mm1 = HighLen1*256 + LowLen1; //计算距离值
		if((Len_mm1 > 1) && (Len_mm1 < 10000)) //有效的测距的结果在 1mm 到10m 之间
		{
			ture_distance1 = Len_mm1;
			flag1 = TURE_Distance;
		}
		else 
		{
			ture_distance1 = 0;
			flag1 = ERROR_Distance;
		}
		Control.Senser.Sonar.forward.distance = ture_distance1;
		Control.Senser.Sonar.forward.isValid  = flag1;
}

void calculate2(uint8_t *data)
{
		union {uint8_t B[2]; uint8_t Hdata; uint8_t Ldata;}src;	
		
		src.B[0] = data[0];
		src.B[1] = data[1];
		
		HighLen2 = src.B[0];
		LowLen2 = src.B[1];
		
		Len_mm2 = HighLen2*256 + LowLen2; //计算距离值
		if((Len_mm2 > 1) && (Len_mm2 < 10000)) //有效的测距的结果在 1mm 到10m 之间
		{
			ture_distance2 = Len_mm2;
			flag2 = TURE_Distance;
		}
		else 
		{
			ture_distance2 = 0;
			flag2 = ERROR_Distance;
		}
		Control.Senser.Sonar.left.distance = ture_distance2;
		Control.Senser.Sonar.left.isValid  = flag2;
		
}

void calculate3(uint8_t *data)
{
		union {uint8_t B[2]; uint8_t Hdata; uint8_t Ldata;}src;	
		
		src.B[0] = data[0];
		src.B[1] = data[1];
		
		HighLen3 = src.B[0];
		LowLen3 = src.B[1];
		
		Len_mm3 = HighLen3*256 + LowLen3; //计算距离值
		if((Len_mm1 > 1) && (Len_mm1 < 10000)) //有效的测距的结果在 1mm 到10m 之间
		{
			ture_distance3 = Len_mm3;
			flag3 = TURE_Distance;
		}
		else 
		{
			ture_distance3 = 0;
			flag3 = ERROR_Distance;
		}
		Control.Senser.Sonar.right.distance = ture_distance3;
		Control.Senser.Sonar.right.isValid  = flag3;
}

