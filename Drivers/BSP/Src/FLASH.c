
#include "flash.h"
#include "string.h"
#include "protocol.h"
#include "control.h"


/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */



#define PARA_ADDR    ((uint32_t)(ADDR_FLASH_SECTOR_11 + 0x00001000))//参数保存地址






//由于使用11扇区的内存，因此擦除11扇区的数据
void Flash_EraseSectors()
{	
	  uint32_t error;
	
	  HAL_FLASH_Unlock();	
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|
	                         FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|
	                         FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	
	  FLASH_EraseInitTypeDef EraseInitStruct;
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector        = FLASH_SECTOR_11;
	  EraseInitStruct.Banks         = 1;
    EraseInitStruct.NbSectors     = 1;
	
		if(HAL_FLASHEx_Erase(&EraseInitStruct,&error)!= HAL_OK) //擦除
	  {
			 Error_Handler();
	  }
		
		HAL_FLASH_Lock();
	
}

void Flash_Write(uint32_t addr,uint16_t *data,uint32_t len)
{
    HAL_FLASH_Unlock();
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|
	                         FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|
	                         FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	  for(int i = 0;i<(len*2);i+=2)
		{
       HAL_FLASH_Program(TYPEPROGRAM_HALFWORD, addr+i, *data++);
		}
    HAL_FLASH_Lock();
}

void Flash_Read(uint32_t addr,uint16_t *data,uint32_t len)
{
	while(len--)
	{		
		uint32_t temp = *(__IO uint32_t*)(addr++);
		
		addr ++;
		*data++ = temp;
	}
}


//save

uint16_t SAVED[200];
uint32_t cou = 0;
void Parameter_S_PID(void)
{
    uint16_t D[200];
	  cou = sizeof(HAL_IO.Parameter)/2;
	  memcpy(D,&HAL_IO.Parameter,sizeof(HAL_IO.Parameter));
	
    Flash_Write(PARA_ADDR,D,sizeof(HAL_IO.Parameter)/2);
}

uint16_t READD[200];
void Parameter_R_PID(void)
{
	  uint16_t D[200];

		Flash_Read(PARA_ADDR,D,sizeof(HAL_IO.Parameter)/2);
		memcpy(&HAL_IO.Parameter,D,sizeof(HAL_IO.Parameter));

		
		/*
		Control.Task.Speed_Kp    = src.F[0];
		Control.Task.Speed_Ki    = src.F[1];
		Control.Task.Position_Kp = src.F[2];
		Control.Task.Position_Ki = src.F[3];
		Control.Task.Heading_Kp  = src.F[4];
		Control.Task.Heading_Ki  = src.F[5];
		*/
		
    
}

//void Parameter_S_MinPWM(void)
//{
//	  Flash_Write(Addr_MinLimit,HAL_PWM.MinPWM,sizeof(HAL_PWM.MinPWM));
//}

//void Parameter_S_SetPWM(void)
//{
//	  Flash_Write(Addr_SetPWM,HAL_PWM.SetPWM,sizeof(HAL_PWM.SetPWM));
//}

//void Parameter_S_CurrentPWM(void)
//{
//	  Flash_Write(Addr_Current,HAL_PWM.CurrentPWM,sizeof(HAL_PWM.CurrentPWM));
//}
////read
//void Parameter_R_MaxPWM(void)
//{
//	  Flash_Read(Addr_MaxLimit,HAL_PWM.MaxPWM,sizeof(HAL_PWM.MaxPWM));
//}

//void Parameter_R_MinPWM(void)
//{
//	  Flash_Read(Addr_MinLimit,HAL_PWM.MinPWM,sizeof(HAL_PWM.MinPWM));
//}

//void Parameter_R_SetPWM(void)
//{
//	  Flash_Read(Addr_SetPWM,HAL_PWM.SetPWM,sizeof(HAL_PWM.SetPWM));
//}

//void Parameter_R_CurrentPWM(void)
//{
//	  Flash_Read(Addr_Current,HAL_PWM.CurrentPWM,sizeof(HAL_PWM.CurrentPWM));
//}

