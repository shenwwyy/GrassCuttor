

#ifndef _FLASH_H_
#define _FLASH_H_

#include "stm32f4xx_hal.h"


void Flash_EraseSectors(void);

void Flash_Write(uint32_t addr,uint16_t *data,uint32_t len);
void Flash_Read(uint32_t addr,uint16_t *data,uint32_t len);


void Parameter_S_MaxPWM(void);
void Parameter_S_MinPWM(void);
void Parameter_S_SetPWM(void);
void Parameter_S_CurrentPWM(void);


void Parameter_R_MaxPWM(void);
void Parameter_R_MinPWM(void);
void Parameter_R_SetPWM(void);
void Parameter_R_CurrentPWM(void);

void Parameter_S_PID(void);
void Parameter_R_PID(void);

#endif









