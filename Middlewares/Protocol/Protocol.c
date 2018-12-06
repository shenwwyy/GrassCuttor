
#include "main.h"
#include "stm32f4xx_hal.h"
#include "protocol.h"
#include "uart.h"
#include "control.h"
#include "string.h"
#include "usart.h"

#include "uart_fifo.h"


extern UART_RX_FIFO_t dlink_rx;
extern UART_TX_FIFO_t dlink_tx;

/*==========================
命令定义
===========================*/
_hal_io HAL_IO;
_waypoint WayPointList[500];

/*====================================================================
 *CRC Function From Here
 *@HM
 *
 *====================================================================
 * Frame typedef
 * |EB 90|CIDL CIDH|SYN|LENL LENH|Payload|CKL CKH|
 * frame head is EB 90
 * CID is identify of a frame
 * SYN is synchronous code of frame ,rate form 0 to 255
 * LEN is only the payload's len
 * Payload is the data we need
 * Sum check is CRC16
 */

//加入发送的缓存中
static void Protocol_T_Combin(uint8_t* data, uint16_t size)
{
	UART_TX_FIFO_write(&dlink_tx,data,size);
}



//校验和
static uint8_t DrvSum(uint8_t* data, uint16_t size)
{
    uint8_t Sum = 0;
	  uint16_t i=0;
    
    for(i = 0; i < size; i++)
        Sum += data[i];
    return Sum;
}


void InvertUint8(unsigned char *dBuf,unsigned char *srcBuf)
{
	int i;
	unsigned char tmp[4];
	tmp[0] = 0;
	for (i=0; i< 8; i++) 
	{
		if (srcBuf[0]& (1 << i))
		tmp[0]|=1<<(7-i);
	}
	dBuf[0] = tmp[0];
}

 

void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf)  
{
	int i;
	unsigned short tmp[4];
	tmp[0] = 0;
	for (i=0; i< 16; i++) 
	{
		if (srcBuf[0]& (1 << i))
		tmp[0]|=1<<(15 - i);
	}
	dBuf[0] = tmp[0];
}

typedef struct {
    uint16_t poly;
    uint16_t crc;
} CRC_CheckSum_t;

void CRC_init(CRC_CheckSum_t *c)
{
  c->poly = 0x8408;
	c->crc = 0;
}

void CRC_update(CRC_CheckSum_t *c, uint8_t p)
{
	uint8_t carry;
	uint8_t i_bits;

	c->crc = (uint16_t)(c->crc ^ p);
	for (i_bits=0; i_bits<8; i_bits++)
	{
			carry = (uint8_t)(c->crc & 1);
			c->crc = (uint16_t)(c->crc / 2);
			if (carry)
			{
					c->crc = (uint16_t)(c->crc^c->poly);
			}
	}
}



static uint16_t CRC_CheckSum (uint8_t *pBuffer,uint8_t Size)
{    
	uint16_t poly = 0x8408;	
	uint16_t crc = 0;
	uint8_t carry;
	uint8_t i_bits;
	uint8_t j;
   
  for (j=0; j<Size; j++)
	{
		crc = (uint16_t)(crc ^ (uint8_t)pBuffer[j]);
		for (i_bits=0; i_bits<8; i_bits++)
		{
			carry = (uint8_t)(crc & 1);
			crc = (uint16_t)(crc / 2);
			if (carry)
			{
				crc = (uint16_t)(crc^poly);
			}
		}
	}
	return crc;
	 
}  
	



/*====================================================================
 *Transmit Function From Here
 *@HM
 *
 *====================================================================
 */


bool getchar_uart5(uint8_t* c)
{
	unsigned uart_rx_head_idx = sizeof(HAL_IO.U5.rxbuf)-__HAL_DMA_GET_COUNTER(huart1.hdmarx);
	if (HAL_IO.U5.rxtail == uart_rx_head_idx)
	{
		return false;
	}
	*c = HAL_IO.U5.rxbuf[HAL_IO.U5.rxtail++];
	if (HAL_IO.U5.rxtail >= sizeof(HAL_IO.U5.rxbuf))
	{
		HAL_IO.U5.rxtail = 0u;
	}
	return true;
}

//周期调用，把数据放入缓存
void Protocol_Rev(void)
{  
	  uint8_t c = 0;
	  size_t len = UART_RX_FIFO_getlen(&dlink_rx);
	  if (len)
		{
			while(UART_RX_FIFO_getc(&dlink_rx,&c))
			{
				Protocol_R_Prepare(c);
			}
		}

}

//协议解析状态机，检查缓存是否有数据
void Protocol_R_Prepare(uint8_t c)
{
	 static uint8_t stage = 0;
   static uint8_t  RawData[256+9];
	 static uint16_t idx = 0;
	 static uint8_t  LENL = 0,LENH = 0;
	 static uint8_t  CIDL = 0,CIDH = 0;
	 static uint8_t  SYN = 0;
	 static CRC_CheckSum_t checksum;
	 static uint8_t crc_low;
	
	 static uint16_t LEN = 0;
	 static uint16_t CID = 0;
	
				switch (stage)
				{
					case 0:
						if (c == 0xEB)
						{
							CRC_init(&checksum);
							RawData[0] = c;
							CRC_update(&checksum, c);
							++stage;
            }
						break;
					case 1:
						if (c == 0x90)
						{
							CRC_update(&checksum, c);
							RawData[1] = c;
							++stage;
            }
						else
						{
							stage = 0;
						}
						break;
					case 2:
						CIDL = c;
						RawData[2] = c;
						CRC_update(&checksum, c);
					  ++stage;
						break;
					case 3:
						CIDH = c;
						RawData[3] = c;
						CRC_update(&checksum, c);
					  ++stage;
						break;
					case 4:
						SYN = c;
						RawData[4] = c;
						CRC_update(&checksum, c);
					  ++stage;
						break;
					case 5:
						LENL = c;
						RawData[5] = c;
						idx = 0;
						CRC_update(&checksum, c);
					  ++stage;
						break;
					case 6:
						LENH = c;
					  LEN = (uint16_t)(LENH << 8)+ LENL;
						RawData[6] = c;
						idx = 0;
						CRC_update(&checksum, c);
					  ++stage;
						break;
					case 7:
						if (idx < LEN)
						{
						  RawData[7+idx] = c;
							CRC_update(&checksum, c);
							++idx;
            }
						else
						{
							crc_low = c;
							++stage;
            }
						break;
					case 8:
						if (checksum.crc == ((c<<8)+crc_low))
						{
							//LED_ON(3);//blue
							CID = (uint16_t)(CIDH << 8)+ CIDL;
							switch(CID)
							{
								case 0x0001: Protocol_R_LED(RawData);break;
								case 0x0005: Protocol_R_CMD(RawData);break;
								case 0X0010: Protocol_R_Parameter(RawData);break;
								case 0x0021: Protocol_R_SimuPos(RawData);break;
								case 0X0040: Protocol_R_WayPoint(RawData);break;
								
							}
							//LED_OFF(3);//blue
            }
						stage = 0;
						break;
					default:
						stage = 0;
        }
}
		 


//Protocol type
/*
   |0  1 |2     3  |4  |5    6   | 7~LEN |7+LEN|8+LEN|
   |EB 90|CIDL CIDH|SYN|LENL LENH|Payload|CKL     CKH| 	 
*/

void Protocol_R_LED(uint8_t *data)//0x01
{

}

void Protocol_R_CMD(uint8_t *data)//0x05
{
	union{uint8_t B[4];uint16_t D[2];uint32_t W;}src;
	 uint32_t ID = 0;
	 uint32_t Value = 0;
	
	src.B[0] = data[7];
	src.B[1] = data[8];
	src.B[2] = data[9];
	src.B[3] = data[10];
	ID = src.W;
	
	src.B[0] = data[11];
	src.B[1] = data[12];
	src.B[2] = data[13];
	src.B[3] = data[14];
	Value = src.W;
	
	 switch(ID)
	 {
		 case 0x05:{ 
			 if(Value  == 0xa9) HAL_IO.CMD.ReadParameter = 0x01;break;
		 }break;//end 05 
		 case 0x06:{ 
			 if(Value  == 0x05) 
			 { 
				 HAL_IO.CMD.StartMission = 0x01;
				 HAL_IO.CMD.StopMission  = 0x00;
				 Control.Car.isunLock = 0x57;//解锁开始任务
				 Control.Command.EmergencyStop = 0x00;//解除紧急停止
				 Control.Task.Task_id = 1;//工作任务
				 
				 
				 //给目标点赋值
				 Control.Task.TargetPoint.Number    = WayPointList[0].id;
				 Control.Task.TargetPoint.altitude  = WayPointList[0].altitude;
				 Control.Task.TargetPoint.latitude  = WayPointList[0].latitude;
				 Control.Task.TargetPoint.longitude = WayPointList[0].longitude;
				 Control.Task.TargetPoint.speed     = WayPointList[0].speed;
				 Control.Task.TargetPoint.course    = WayPointList[0].course;
				 
				 //给家的点赋值，
				 Control.Task.ChargePoint.Number    = 0;
				 Control.Task.ChargePoint.altitude  = Control.Task.CurrentPoint.altitude;
				 Control.Task.ChargePoint.latitude  = Control.Task.CurrentPoint.latitude;
				 Control.Task.ChargePoint.longitude = Control.Task.CurrentPoint.longitude;
				 Control.Task.ChargePoint.speed     = Control.Task.TargetPoint.speed;
				 Control.Task.ChargePoint.course    = Control.Task.CurrentPoint.course;
				 //给上一点赋值
				 Control.Task.LastPoint.Number    = Control.Task.CurrentPoint.Number;
				 Control.Task.LastPoint.altitude  = Control.Task.CurrentPoint.altitude;
				 Control.Task.LastPoint.latitude  = Control.Task.CurrentPoint.latitude;
				 Control.Task.LastPoint.longitude = Control.Task.CurrentPoint.longitude;
				 Control.Task.LastPoint.speed     = Control.Task.TargetPoint.speed;
				 Control.Task.LastPoint.course    = Control.Task.CurrentPoint.course;
				 
				 
				 
				 
				 //控制参数清零
				 Control.Task.Speed_Err = 0;
				 Control.Task.Speed_i = 0;
				 Control.Task.Speed_Out = 0;
				 
				 Control.Task.Position_Err = 0;
				 Control.Task.Position_i = 0;
				 Control.Task.Position_Out = 0;
				 
				 Control.Task.Heading_Err = 0;
				 Control.Task.Heading_i = 0;
				 Control.Task.Heading_d = 0;
				 Control.Task.Heading_Out = 0;
				 
				 
			 }
			 else if(Value  == 0x06)
			 {
				 HAL_IO.CMD.StartMission = 0x00;
				 HAL_IO.CMD.StopMission  = 0x01;
				 HAL_IO.CMD.BackHome     = 0x00;
				 
				 Control.Command.EmergencyStop = 0x6d;//紧急停止
				 Control.Task.Task_id = 0;//待机模式
				 
				 
				 Control.Task.Speed_Err = 0;
				 Control.Task.Speed_i = 0;
				 Control.Task.Speed_Out = 0;
				 
				 Control.Task.Position_Err = 0;
				 Control.Task.Position_i = 0;
				 Control.Task.Position_Out = 0;
				 
				 Control.Task.Heading_Err = 0;
				 Control.Task.Heading_i = 0;
				 Control.Task.Heading_d = 0;
				 Control.Task.Heading_Out = 0;
				 
				 
			 }
			 else if(Value  == 0x07)
			 { 
				 if(HAL_IO.CMD.StopMission  == 0x00)
				 {
				    HAL_IO.CMD.BackHome     = 0x01;
					  Control.Car.isunLock = 0x57;//解锁开始任务
				    Control.Command.EmergencyStop = 0x00;//解除紧急停止
					 Control.Task.Task_id = 3;//返航模式
				 }
			 }
			 
		 }break;//end 06
		 case 0x40:{//航线专用 
			 if(Value  == 0x07)//发送航点
			 { 
						if(HAL_IO.SendWayPointCount < HAL_IO.MaxWayPointCount)
						{
							 HAL_IO.CMD.TranferWayPoint = 0x01;
							 memcpy(&HAL_IO.WayPoint,&WayPointList[HAL_IO.SendWayPointCount],sizeof(HAL_IO.WayPoint));
						   HAL_IO.SendWayPointCount ++;//使用完成后自加，下一次用来检查
						}
						else
						{
							 HAL_IO.CMD.TranferWayPoint = 0x00;
							 HAL_IO.SendWayPointCount = 0;
							 Protocol_T_Echo(0x40,0x08);
						} 
			 }
			 else if(Value  == 0x08)//停止发送航点
			 { 
				    HAL_IO.SendWayPointCount = 0;
				    HAL_IO.CMD.TranferWayPoint = 0x00;
			 }
		 }break;//end 40
		 case 0x41:{//航线专用,开始发送航点 
			      HAL_IO.ReadWayPointCount = 0;                                                                          
			      HAL_IO.MaxWayPointCount = Value;
		 }break;//end 41
		 case 0x60:{
			 if(Value == 0x00)//设置当前点为家
			 {
				  //给充电定点设定为当前经纬度
				  Control.Task.ChargePoint.altitude  = Control.Task.CurrentPoint.altitude;
					Control.Task.ChargePoint.latitude  = Control.Task.CurrentPoint.latitude;
					Control.Task.ChargePoint.longitude = Control.Task.CurrentPoint.longitude;
					Control.Task.ChargePoint.course    = Control.Task.CurrentPoint.course;
					Control.Task.ChargePoint.speed     = Control.Task.CurrentPoint.speed;
			 }
			 else if(Value == 0x01)//设置当前点经纬度和目标经纬度一致
			 {
				  Control.Task.CurrentPoint.altitude  = Control.Task.TargetPoint.altitude;
					Control.Task.CurrentPoint.latitude  = Control.Task.TargetPoint.latitude;
					Control.Task.CurrentPoint.longitude = Control.Task.TargetPoint.longitude;
					Control.Task.CurrentPoint.course    = Control.Task.TargetPoint.course;
					Control.Task.CurrentPoint.speed     = Control.Task.TargetPoint.speed;
			 }
			 else if(Value == 0x02)//左前正向
			 {
				  Control.Car.WheelTest = 0x01;
			 }
			 else if(Value == 0x03)//左前反向
			 {
          Control.Car.WheelTest = 0x10;
			 }
			 else if(Value == 0x04)//左后正向
			 {
				  Control.Car.WheelTest = 0x02;
			 }
			 else if(Value == 0x05)//左后反向
			 {
          Control.Car.WheelTest = 0x20;
			 }
			 else if(Value == 0x06)//右前正向
			 {
				  Control.Car.WheelTest = 0x04;
			 }
			 else if(Value == 0x07)//右前反向
			 {
          Control.Car.WheelTest = 0x40;
			 }
			 else if(Value == 0x08)//右后正向
			 {
				  Control.Car.WheelTest = 0x08;
			 }
			 else if(Value == 0x09)//右后反向
			 {
          Control.Car.WheelTest = 0x80;
			 }
			 else if(Value == 0x0a)//取消测试
			 {
          Control.Car.WheelTest = 0x00;
			 }
			 else if(Value == 0x0b)//进入调试
			 {
          HAL_IO.Satuts.isDebug = 0x01;
			 }
			 else if(Value == 0x0c)//退出调试
			 {
          HAL_IO.Satuts.isDebug = 0x00;
			 }
				 

		 }break;//end 60
		 case 0x90:{//底层测试专用

		 }break;//end 90
		 
		 
		 
		 
		 
	 }
}

void Protocol_R_Parameter(uint8_t *data)//0x10
{
   memcpy(&HAL_IO.Parameter,data+7,sizeof(HAL_IO.Parameter));
	
	 Control.Parameter.isSaveParameter = 0x01;
}

void Protocol_R_WayPoint(uint8_t *data)//0x40
{
	memcpy(&HAL_IO.WayPoint,data+7,sizeof(HAL_IO.WayPoint));
	
	memcpy(&WayPointList[HAL_IO.WayPoint.id],&HAL_IO.WayPoint,sizeof(HAL_IO.WayPoint));
	
	HAL_IO.ReadWayPointCount ++;
	
	Protocol_T_Echo(0x40,0x07);
}

void Protocol_R_SimuPos(uint8_t *data)//0x21
{
	memcpy(&HAL_IO.SIM,data+7,sizeof(HAL_IO.SIM));
}




//发送

void Protocol_T_Echo(uint32_t ID,uint32_t Value)//0x04
{
			  HAL_IO.CMD.ReadParameter = 0;
			
				union{uint8_t B[2];uint16_t D;}src;
				uint8_t  DataToSend[100];
				uint16_t DataCount = 0;

				DataToSend[DataCount++] = 0xEB;
				DataToSend[DataCount++] = 0x90;

				src.D = 0x0004;
				DataToSend[DataCount++] = src.B[0];
				DataToSend[DataCount++] = src.B[1];


				DataToSend[DataCount++] = HAL_IO.U5.txsyn++;

				src.D = 8;
				DataToSend[DataCount++] = src.B[0];
				DataToSend[DataCount++] = src.B[1];

				memcpy(DataToSend + DataCount,&ID,sizeof(ID));
				DataCount += sizeof(ID);
				
				memcpy(DataToSend + DataCount,&Value,sizeof(Value));
				DataCount += sizeof(Value);


				src.D = CRC_CheckSum(DataToSend,DataCount);
				DataToSend[DataCount++] = src.B[0];
				DataToSend[DataCount++] = src.B[1];

				Protocol_T_Combin(DataToSend,DataCount);
}



void Protocol_T_Parameter(void)//0x10
{
	  if(HAL_IO.CMD.ReadParameter)
		{
			  HAL_IO.CMD.ReadParameter = 0;
			
				union{uint8_t B[2];uint16_t D;}src;
				uint8_t  DataToSend[100];
				uint16_t DataCount = 0;

				DataToSend[DataCount++] = 0xEB;
				DataToSend[DataCount++] = 0x90;

				src.D = 0x0010;
				DataToSend[DataCount++] = src.B[0];
				DataToSend[DataCount++] = src.B[1];


				DataToSend[DataCount++] = HAL_IO.U5.txsyn++;

				src.D = sizeof(HAL_IO.Parameter);
				DataToSend[DataCount++] = src.B[0];
				DataToSend[DataCount++] = src.B[1];

				memcpy(DataToSend + DataCount,&HAL_IO.Parameter,sizeof(HAL_IO.Parameter));
				DataCount += sizeof(HAL_IO.Parameter);


				src.D = CRC_CheckSum(DataToSend,DataCount);
				DataToSend[DataCount++] = src.B[0];
				DataToSend[DataCount++] = src.B[1];

				Protocol_T_Combin(DataToSend,DataCount);
		}
}

void Protocol_T_GPS(uint32_t T)//0x20
{
	      //HAL_IO.GPS_Count += T;
	      if(HAL_IO.GPS_Count >= 0x01)
				{
				  HAL_IO.GPS_Count =0;	
					union{uint8_t B[2];uint16_t D;}src;
					uint8_t  DataToSend[100];
					uint16_t DataCount = 0;

					DataToSend[DataCount++] = 0xEB;
					DataToSend[DataCount++] = 0x90;

					src.D = 0x0020;//ID
					DataToSend[DataCount++] = src.B[0];
					DataToSend[DataCount++] = src.B[1];


					DataToSend[DataCount++] = HAL_IO.U5.txsyn++;

					src.D = sizeof(HAL_IO.GPS);//LEN
					DataToSend[DataCount++] = src.B[0];
					DataToSend[DataCount++] = src.B[1];

					memcpy(DataToSend + DataCount,&HAL_IO.GPS,sizeof(HAL_IO.GPS));
					DataCount += sizeof(HAL_IO.GPS);


					src.D = CRC_CheckSum(DataToSend,DataCount);
					DataToSend[DataCount++] = src.B[0];
					DataToSend[DataCount++] = src.B[1];

					Protocol_T_Combin(DataToSend,DataCount);
			 }
}




void Protocol_T_WayPoint(void)//0x40
{
	  if(HAL_IO.CMD.TranferWayPoint)
		{
			  HAL_IO.CMD.TranferWayPoint = 0;

				union{uint8_t B[2];uint16_t D;}src;
				uint8_t  DataToSend[100];
				uint16_t DataCount = 0;

				DataToSend[DataCount++] = 0xEB;
				DataToSend[DataCount++] = 0x90;

				src.D = 0x0040;
				DataToSend[DataCount++] = src.B[0];
				DataToSend[DataCount++] = src.B[1];


				DataToSend[DataCount++] = HAL_IO.U5.txsyn++;

				src.D = sizeof(HAL_IO.WayPoint);
				DataToSend[DataCount++] = src.B[0];
				DataToSend[DataCount++] = src.B[1];

				memcpy(DataToSend + DataCount,&HAL_IO.WayPoint,sizeof(HAL_IO.WayPoint));
				DataCount += sizeof(HAL_IO.WayPoint);


				src.D = CRC_CheckSum(DataToSend,DataCount);
				DataToSend[DataCount++] = src.B[0];
				DataToSend[DataCount++] = src.B[1];

				Protocol_T_Combin(DataToSend,DataCount);
				
		}
}

void Protocol_T_Status(uint32_t T)//0x50
{
	      HAL_IO.STATUS_Count += T;
	      if(HAL_IO.STATUS_Count >= 1000)
				{
					HAL_IO.STATUS_Count = 0;
					union{uint8_t B[2];uint16_t D;}src;
					uint8_t  DataToSend[100];
					uint16_t DataCount = 0;

					DataToSend[DataCount++] = 0xEB;
					DataToSend[DataCount++] = 0x90;

					src.D = 0x0050;//ID
					DataToSend[DataCount++] = src.B[0];
					DataToSend[DataCount++] = src.B[1];


					DataToSend[DataCount++] = HAL_IO.U5.txsyn++;

					src.D = sizeof(HAL_IO.Satuts);//LEN
					DataToSend[DataCount++] = src.B[0];
					DataToSend[DataCount++] = src.B[1];

					memcpy(DataToSend + DataCount,&HAL_IO.Satuts,sizeof(HAL_IO.Satuts));
					DataCount += sizeof(HAL_IO.Satuts);


					src.D = CRC_CheckSum(DataToSend,DataCount);
					DataToSend[DataCount++] = src.B[0];
					DataToSend[DataCount++] = src.B[1];

					Protocol_T_Combin(DataToSend,DataCount);
			 }
}

void Protocol_T_Par(uint32_t T)//0x51
{
	      HAL_IO.PAR_Count += T;
	      if(HAL_IO.PAR_Count >= 1000)
				{
					HAL_IO.PAR_Count = 0;
					union{uint8_t B[2];uint16_t D;}src;
					uint8_t  DataToSend[100];
					uint16_t DataCount = 0;

					DataToSend[DataCount++] = 0xEB;
					DataToSend[DataCount++] = 0x90;

					src.D = 0x0051;//ID
					DataToSend[DataCount++] = src.B[0];
					DataToSend[DataCount++] = src.B[1];


					DataToSend[DataCount++] = HAL_IO.U5.txsyn++;

					src.D = sizeof(HAL_IO.Par);//LEN
					DataToSend[DataCount++] = src.B[0];
					DataToSend[DataCount++] = src.B[1];

					memcpy(DataToSend + DataCount,&HAL_IO.Par,sizeof(HAL_IO.Par));
					DataCount += sizeof(HAL_IO.Par);


					src.D = CRC_CheckSum(DataToSend,DataCount);
					DataToSend[DataCount++] = src.B[0];
					DataToSend[DataCount++] = src.B[1];

					Protocol_T_Combin(DataToSend,DataCount);
			 }
}


