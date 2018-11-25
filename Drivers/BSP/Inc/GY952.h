

#ifndef _GY952_H_
#define _GY952_H_

#include "stm32f4xx_hal.h"

void GY952_Init(void);


typedef struct {
	
	struct {
		  struct {
				int16_t ax;
				int16_t ay;
				int16_t az;
			}RAW;
			
			struct {
				float ax;
				float ay;
				float az;
			}NEW;
	}ACC;
	
	struct {
		  struct {
				int16_t gx;
				int16_t gy;
				int16_t gz;
			}RAW;
			
			struct {
				float gx;
				float gy;
				float gz;
			}DEG;
	}GYRO;
	
	struct {
				float Roll;
				float Pitch;
				float Yaw;
	}EULER;
	
	uint32_t SUM_Error;
	uint32_t F_5A_1_Error;
	uint32_t F_5A_2_Error;
	

}_gy925;

extern _gy925 GY925;





void GY952_Rev(void);
void GY952_R_Prepare(uint8_t c);

void GY952_Decode(uint8_t *data);







#endif



