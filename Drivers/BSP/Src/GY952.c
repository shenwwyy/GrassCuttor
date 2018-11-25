
#include "gy952.h"
#include "uart_fifo.h"


extern UART_RX_FIFO_t imu_rx;
extern UART_TX_FIFO_t imu_tx;

uint8_t GY952_raw[256];
uint8_t GY952_Count = 0;



_gy925 GY925;


//周期调用，把数据放入缓存
void GY952_Rev(void)
{  
	  uint8_t c = 0;
	  size_t len = UART_RX_FIFO_getlen(&imu_rx);
	  if (len)
		{
			while(UART_RX_FIFO_getc(&imu_rx,&c))
			{
				GY952_R_Prepare(c);
			}
		}

}

//协议解析状态机，检查缓存是否有数据
void GY952_R_Prepare(uint8_t c)
{
	GY952_raw[GY952_Count++] =c;
	
	 static uint8_t stage = 0;
	 static uint8_t  RawData[256+9];
	 static uint8_t  SUM = 0;
	 static uint8_t  LEN = 0;
	 static uint8_t  idx = 0; 
	
	
				switch (stage)
				{
					case 0:
						if (c == 0x5A)
						{ 
							SUM = 0;
							RawData[0] = c;
							SUM += c;
							++stage;
            }
						else
						{
							
						}
						break;
					case 1:
						if (c == 0x5A)
						{
							RawData[1] = c;
							SUM += c;
							++stage;
            }
						else
						{
							GY925.F_5A_2_Error++;
							stage = 0;
						}
						break;
					case 2:
						RawData[2] = c;
					  SUM += c;
					  ++stage;
						break;
					case 3:
						LEN = c;
						RawData[3] = c;
					  SUM += c;
					  idx = 0;
					  ++stage;
						break;
					case 4:
						if (idx < LEN)
						{
						  RawData[4+idx] = c;
							SUM += c;
							++idx;
            }
						else
						{
							RawData[4+idx] = c;//sum
							if(SUM == c)
							{
								GY952_Decode(RawData);
							}
							else
							{
								GY925.SUM_Error++;
							}
							stage = 0;
            }
						break;
					default:
						stage = 0;
        }
}









void GY952_Decode(uint8_t *data)
{
	  uint8_t ID;
    ID = data[2];

    switch(ID)
		{
			case 0x15: 
			  GY925.ACC.RAW.ax = (int16_t)(data[4] << 8 )| data[5];
			  GY925.ACC.RAW.ay = (int16_t)(data[6] << 8 )| data[7];
			  GY925.ACC.RAW.az = (int16_t)(data[8] << 8 )| data[9];
			
			  GY925.ACC.NEW.ax = GY925.ACC.RAW.ax * 0.00006103515625f;
			  GY925.ACC.NEW.ay = GY925.ACC.RAW.ay * 0.00006103515625f;
			  GY925.ACC.NEW.az = GY925.ACC.RAW.az * 0.00006103515625f;
				break;
			case 0x25: 
			  GY925.GYRO.RAW.gx = (int16_t)(data[4] << 8 )| data[5];
			  GY925.GYRO.RAW.gy = (int16_t)(data[6] << 8 )| data[7];
			  GY925.GYRO.RAW.gz = (int16_t)(data[8] << 8 )| data[9];
			
				GY925.GYRO.DEG.gx = GY925.GYRO.RAW.gx * 2000.000000f/32768.000000f;
			  GY925.GYRO.DEG.gy = GY925.GYRO.RAW.gy * 2000.000000f/32768.000000f ;
			  GY925.GYRO.DEG.gz = GY925.GYRO.RAW.gz * 2000.000000f/32768.000000f;
				break;
			case 0x45: 
				GY925.EULER.Roll  = (float)((int16_t)(data[4] << 8 )| data[5])/100.0f;
				GY925.EULER.Pitch = (float)((int16_t)(data[6] << 8 )| data[7])/100.0f;
				GY925.EULER.Yaw   = (float)((int16_t)(data[8] << 8 )| data[9])/100.0f;	
			break;
			case 0x65: break;
		}
	}

//加入发送的缓存中
static void GY952_T_Combin(uint8_t* data, uint16_t size)
{
	UART_TX_FIFO_write(&imu_tx,data,size);
}

void GY952_T_CMD(uint8_t CMD)
{
	uint8_t DataToSend[3] = {0xa5,0x00,0x00};
	
	DataToSend[1] = CMD;
	DataToSend[2] = (uint8_t)(0xa5 + CMD);
	
	GY952_T_Combin(DataToSend,3);
}


void GY952_Init(void)
{
	GY952_T_CMD(0x15);//acc
	GY952_T_CMD(0x25);//gyro
	GY952_T_CMD(0x45);//euler
	GY952_T_CMD(0xa5);//100hz
	GY952_T_CMD(0xaa);//save
	GY952_T_CMD(0x15);
	GY952_T_CMD(0x25);
}







