/*************************************************
Copyright (C)  FACCON Tech. Co., Ltd.
File name: ubolx.c
Author:ZH
Version:V1.O
Date:2018/01/09
Description: 初始化GPS,更改数据协议,GPS波特率,数据更新速率,NAV数据使能
Others: 
Function List: 
1.ubloxInitGps();GPS初始化
2.ubloxSet_PRT();修改GPS波特率,协议
3.ubloxSet_Rate();修改GPS数据更新速率
4.ubloxmsg_Enable();使能数据信息
5.ublox_Protocol_Prepare();接收,解析GPS初始化后的数据
6.Protocol_NAV_POSLLH();解析NAV_POSLL数据
7.Protocol_NAV_STATUS();解析NAV_STATUS数据
8.Protocol_NAV_VELNED();解析NAV_VELNED数据
History: 
*************************************************/

#include "ublox.h"
#include "uart.h"
#include "usart.h"

//void ublox_Protocol_Send(uint8_t *data,uint8_t Len)//发送函数封装，使用函数式
//{
//	//HAL_UART_Transmit(&huart6,data,Len,0);
//	HAL_UART_Transmit_DMA(&huart6,data,Len);
//}

/***********************************************************************************************
*函数名 ：ubloxInitGps
*函数功能描述 ：初始化GPS
*函数参数 ：无
*函数返回值 ：无
*作者 ：
*函数创建日期 ： 
*函数修改日期 ： 
*修改人 ：
*修改原因 ： 
*版本 ： 
*历史版本 ： 
***********************************************************************************************/
void ubloxInitGps(void)
{
	HAL_Delay(1000);
	ubloxSet_PRT(9600,UBLOX_PROTOCOL_UBX,UBLOX_PROTOCOL_UBX);	
	//ubloxSet_PRT(115200,UBLOX_PROTOCOL_UBX,UBLOX_PROTOCOL_UBX);	

//	HAL_Delay(100);
//	__HAL_RCC_GPIOC_CLK_DISABLE();
//	__HAL_RCC_USART6_CLK_DISABLE();
//	HAL_UART_MspInit(&huart6);//重新初始化串口
//	//HAL_UART_MspDeInit(&huart6);
//	GPS_USART6_UART_Init(115200);
	
	HAL_Delay(100);
	ubloxSet_Rate(500);
	HAL_Delay(100);	
	ubloxmsg_Enable(UBLOX_NAV_CLASS,UBLOX_NAV_POSLLH);	
	HAL_Delay(100);
	ubloxmsg_Enable(UBLOX_NAV_CLASS,UBLOX_NAV_VELNED);	
	HAL_Delay(100);
	ubloxmsg_Enable(UBLOX_NAV_CLASS,UBLOX_NAV_STATUS);	
}


/***********************************************************************************************
*函数名 ：ubloxSet_PRT
*函数功能描述 ：设置ublox数据协议;波特率;
*函数参数 ：baud;inProtoMask;outProtoMask
*函数返回值 ：无
*作者 ：
*函数创建日期 ： 
*函数修改日期 ： 
*修改人 ：
*修改原因 ： 
*版本 ： 
*历史版本 ： 
***********************************************************************************************/
void ubloxSet_PRT(uint32_t baud,uint16_t inProtoMask,uint16_t outProtoMask){
//B5 62 06 00 14 00 01 00 00 00 00 08 CO 00 00 C2 01 00 01 00 01 00 00 00 00 00 (CK_A CK_B)
	 unsigned char data_to_send[100];
	 unsigned char data_cnt = 0;  
	
   union {unsigned char B[4];uint16_t D;uint32_t T;}src;
	 uint8_t CK_A = 0,CK_B = 0;
	 uint8_t i;
	 
	//==========帧头=========
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR1;	
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR2;	 
	//==========CLASS===============
	data_to_send[data_cnt++] = UBLOX_CFG_CLASS;
	//==========ID================
	data_to_send[data_cnt++] = UBLOX_CFG_PRT;
	//==========LEN================
	data_to_send[data_cnt++] = 0x14;
	data_to_send[data_cnt++] = 0x00;// LEN
	//==========Data===============	 
	data_to_send[data_cnt++] = UBLOX_PortNum_UART1;//portID
	data_to_send[data_cnt++] = 0x00;//reserved1
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;//txready
	data_to_send[data_cnt++] = 0xC0;
	data_to_send[data_cnt++] = 0x08;
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;//mode
	 
	src.T = baud;
		data_to_send[data_cnt++] = src.B[0];
		data_to_send[data_cnt++] = src.B[1];
	 	data_to_send[data_cnt++] = src.B[2];
		data_to_send[data_cnt++] = src.B[3];//baudrate

	src.D = inProtoMask;
		data_to_send[data_cnt++] = src.B[0];
		data_to_send[data_cnt++] = src.B[1];//inProtoMask
		
	src.D = outProtoMask;
		data_to_send[data_cnt++] = src.B[0];
		data_to_send[data_cnt++] = src.B[1];//outProtoMask
		
	data_to_send[data_cnt++]=0x00;
	data_to_send[data_cnt++]=0x00;//flags
	
	data_to_send[data_cnt++]=0x00;
	data_to_send[data_cnt++]=0x00;//reserved2

	//==========LEN================
	src.D = data_cnt - 6;
	data_to_send[4] = src.B[0];
	data_to_send[5] = src.B[1];
	//==========SUM================
	for(i=2;i<data_cnt;i++)
	{
		CK_A = CK_A + data_to_send[i];
		CK_B = CK_B + CK_A;
	}
	data_to_send[data_cnt++]=CK_A;
	data_to_send[data_cnt++]=CK_B;
	//=========Send=================
	ublox_Protocol_Send(data_to_send,data_cnt);	
}

/***********************************************************************************************
*函数名 ： ubloxSet_Rate
*函数功能描述 ：设置数据更新频率eg:100ms=10hz
*函数参数 ： ms 
*函数返回值 ： 
*作者 ：
*函数创建日期 ： 
*函数修改日期 ： 
*修改人 ：
*修改原因 ： 
*版本 ： 
*历史版本 ： 
***********************************************************************************************/
void ubloxSet_Rate(unsigned short int ms) {
//B5 62 06 08 06 00    64 00 01 00 01 00     7A 12
	unsigned char data_to_send[100];
	unsigned char data_cnt = 0;      
    union {unsigned char B[4];uint16_t D;}src;
	 uint8_t CK_A = 0,CK_B = 0;
	 uint8_t i;
	//==========帧头=========
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR1;
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR2;
	//==========CLASS===============
	data_to_send[data_cnt++] = UBLOX_CFG_CLASS;
	//==========ID================
	data_to_send[data_cnt++] = UBLOX_CFG_RATE;
	//==========LEN================
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;
	//==========Data===============	 
	src.D = ms;
		data_to_send[data_cnt++] = src.B[0];
		data_to_send[data_cnt++] = src.B[1];//100ms --- 10hz
	 
		data_to_send[data_cnt++] = 0x01;
		data_to_send[data_cnt++] = 0x00;//navRate
	 
		data_to_send[data_cnt++] = 0x01;
		data_to_send[data_cnt++] = 0x00;//GPS-Time		
	//==========LEN================
	src.D = data_cnt - 6;
	data_to_send[4] = src.B[0];
	data_to_send[5] = src.B[1];
	//==========SUM================
	for(i=2;i<data_cnt;i++)
	{
		CK_A = CK_A + data_to_send[i];
		CK_B = CK_B + CK_A;
	}
	data_to_send[data_cnt++]=CK_A;
	data_to_send[data_cnt++]=CK_B;
	//=========Send=================
	ublox_Protocol_Send(data_to_send,data_cnt);			
}

/***********************************************************************************************
*函数名 ： ubloxmsg_Enable
*函数功能描述 ：使能NAV数据
*函数参数 ： msgClass,msgID
*函数返回值 ： 
*作者 ：
*函数创建日期 ： 
*函数修改日期 ： 
*修改人 ：
*修改原因 ： 
*版本 ： 
*历史版本 ： 
***********************************************************************************************/
void ubloxmsg_Enable(uint8_t msgClass,uint8_t msgID)
{
//NAV-STATUS: B5 62 06 01 08 00  01 03 00 01 00 00 00 00  14 C5  	
//NAV-POSLLH: B5 62 06 01 08 00  01 02 00 01 00 00 00 00  13 BE
//NAV-VELNED: B5 62 06 01 08 00  01 12 00 01 00 00 00 00  23 2E	
	unsigned char data_to_send[100];
	unsigned char data_cnt = 0;      
    union {unsigned char B[2];uint32_t D;}src;
	uint8_t CK_A = 0,CK_B = 0;
	uint8_t i;	
	//==========帧头=========
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR1;
	data_to_send[data_cnt++] = UBLOX_SYNC_CHAR2;
	//==========CLASS===============
	data_to_send[data_cnt++] = UBLOX_CFG_CLASS;
	//==========ID================
	data_to_send[data_cnt++] = UBLOX_CFG_MSG;
	//==========LEN================
	data_to_send[data_cnt++] = 0x06;
	data_to_send[data_cnt++] = 0x00;
	//==========Data===============	 
	data_to_send[data_cnt++] = msgClass;
	data_to_send[data_cnt++] = msgID;
	
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x01;
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;
	data_to_send[data_cnt++] = 0x00;
	//==========LEN================
	src.D = data_cnt - 6;
	data_to_send[4] = src.B[0];
	data_to_send[5] = src.B[1];
	//==========SUM================
	for(i=2;i<data_cnt;i++)
	{
		CK_A = CK_A + data_to_send[i];
		CK_B = CK_B + CK_A;
	}
	data_to_send[data_cnt++]=CK_A;
	data_to_send[data_cnt++]=CK_B;
	//=========Send=================
	ublox_Protocol_Send(data_to_send,data_cnt);	
}

/***********************************************************************************************
*函数名 ：ublox_Protocol_Prepare(); 
*函数功能描述 ：接收、解析收到的GPS数据
*函数参数 ： data
*函数返回值 ： 
*作者 ：
*函数创建日期 ： 
*函数修改日期 ： 
*修改人 ：
*修改原因 ： 
*版本 ： 
*历史版本 ： 
***********************************************************************************************/
uint8_t ublox_ProRxBuffer[256];
uint8_t ublox_ProState = 0;

void ublox_Protocol_Prepare(uint8_t data)
{	
	static uint8_t _data_cnt = 0;
	static uint16_t _data_len;
	uint16_t Sum = 0,Check = 1;
	union {uint8_t B[2];uint16_t D;}src;
	
	if(ublox_ProState==0&&data==UBLOX_SYNC_CHAR1)//0xb5
	{
		ublox_ProState=1;
		ublox_ProRxBuffer[0]=data;
	}
	else if(ublox_ProState==1&&data==UBLOX_SYNC_CHAR2)//0x62
	{
		ublox_ProState=2;   
		ublox_ProRxBuffer[1]=data;
	}
	else if(ublox_ProState==2&&data==UBLOX_NAV_CLASS)//class-0x01
	{
		ublox_ProState=3;
		ublox_ProRxBuffer[2]=data;		
	}
	else if(ublox_ProState==3)//id
	{
		ublox_ProState=4;
		ublox_ProRxBuffer[3]=data;		
	}
	else if(ublox_ProState==4)//len1
	{
		_data_cnt = 4;
		ublox_ProState=5;
		ublox_ProRxBuffer[_data_cnt++]=data;
	}
	else if(ublox_ProState==5)//len2
	{
//		_data_cnt = 5;
		ublox_ProState=6;
		ublox_ProRxBuffer[_data_cnt++]=data;       
		src.B[0] = ublox_ProRxBuffer[4];
		src.B[1] = ublox_ProRxBuffer[5];
		_data_len = src.D + 2;
	}
	else if(ublox_ProState==6)
	{
		_data_len--;//PAYLODE数据长度
		ublox_ProRxBuffer[_data_cnt++]=data;
		if(_data_len==0)
		{
			Sum = CheckSum(ublox_ProRxBuffer,_data_cnt);			
			src.B[0] = ublox_ProRxBuffer[_data_cnt-2];//CK_A
			src.B[1] = ublox_ProRxBuffer[_data_cnt-1];//CK_B			
			Check = src.D;			
			if(Sum == Check)
			{			      
				switch(ublox_ProRxBuffer[3])//判断ID 
				{   
					case UBLOX_NAV_VELNED: Protocol_NAV_VELNED(ublox_ProRxBuffer);//解析VELNED
						break;
					case UBLOX_NAV_STATUS: Protocol_NAV_STATUS(ublox_ProRxBuffer);//解析STATUS
						break;
					case UBLOX_NAV_POSLLH: Protocol_NAV_POSLLH(ublox_ProRxBuffer);//解析POSLLH
						break;																											//case 0x41:{ Protocol_R_GetRoute(ProRxBuffer); }break;//GetRoute 这个是CRC校验
					default  : ublox_ProState=0;
						break;
				}
                ublox_ProState=0;				
			}
			else ublox_ProState = 0;
		}	
	}
	else ublox_ProState = 0;
}

/***********************************************************************************************
*函数名 ： CheckSum();
*函数功能描述 :校验和
*函数参数 ： buff,len
*函数返回值 ： 
*作者 ：
*函数创建日期 ： 
*函数修改日期 ： 
*修改人 ：
*修改原因 ： 
*版本 ： 
*历史版本 ： 
***********************************************************************************************/
uint16_t CheckSum(uint8_t *buff,uint16_t len)
{   
	uint8_t i,CK_A = 0, CK_B = 0;
	
	union {uint8_t B[2];uint16_t D;}src;
	
	for(i=2;i<(len - 2);i++)
	{
		CK_A = CK_A + buff[i];
		CK_B = CK_B + CK_A;
	}
	
	src.B[0] = CK_A;
	src.B[1] = CK_B;
	return src.D;
}

/***********************************************************************************************
*函数名 ： Protocol_NAV_VELNED();Protocol_NAV_STATUS();Protocol_NAV_POSLLH();
*函数功能描述 ： 解析对应NAV数据
*函数参数 ： data
*函数返回值 ： 
*作者 ：
*函数创建日期 ： 
*函数修改日期 ： 
*修改人 ：
*修改原因 ： 
*版本 ： 
*历史版本 ： 
***********************************************************************************************/
int32_t NAV_VELNED[9];
int32_t NAV_STATUS[7];
int32_t NAV_POSLLH[7];
float velN,velE,velD,heading;
uint32_t iTOW;
void Protocol_NAV_VELNED(uint8_t *data)
{
	union {uint8_t B[4];int32_t W;}src;	
	uint8_t src_count = 0,data_count = 6;
	uint32_t speed,gSpeed,sAcc,cAcc;
	
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	iTOW=NAV_VELNED[0];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W * 0.01f;
	velN=NAV_VELNED[1];

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	velE=NAV_VELNED[2] * 1e-2;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	velD=NAV_VELNED[3] * 1e-2;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	speed=NAV_VELNED[4] * 1e-2;

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	gSpeed=NAV_VELNED[5] * 1e-2;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	heading=NAV_VELNED[6] * 1e-5;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	sAcc=NAV_VELNED[7];

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.W;
	cAcc=NAV_VELNED[8];
}

void Protocol_NAV_STATUS(uint8_t *data)
{
	union {uint8_t B[4];uint32_t D;uint8_t T;}src;	
	uint8_t src_count = 0,data_count = 6;
	uint32_t iTOW,ttff,msss;
	uint8_t gpsFix,flags,fixStat,flags2;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_STATUS[src_count++] = src.D;
	iTOW = NAV_STATUS[0];
	
	src.T = data[data_count++];
	NAV_STATUS[src_count++] = src.T;
	gpsFix = NAV_STATUS[1];
	
	src.T  = data[data_count++];
	NAV_STATUS[src_count++] = src.T;
	flags = NAV_STATUS[2];

	src.T  = data[data_count++];
	NAV_STATUS[src_count++] = src.T;
	fixStat = NAV_STATUS[3];
	
	src.T  = data[data_count++];
	NAV_STATUS[src_count++] = src.T;
	flags2 = NAV_STATUS[4];
		
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.D;
	ttff = NAV_STATUS[5];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_VELNED[src_count++] = src.D;
	msss = NAV_STATUS[6];	
}

float lon,lat,height,hMSL;
void Protocol_NAV_POSLLH(uint8_t *data)
{
	union {uint8_t B[4];uint32_t D;}src;	
	uint8_t src_count = 0,data_count = 6;
	uint32_t iTOW,hAcc,vAcc;
	
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	iTOW = NAV_POSLLH[0];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	lon = NAV_POSLLH[1] * 1e-7;

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	lat = NAV_POSLLH[2] * 1e-7;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	height = NAV_POSLLH[3] * 1e-3 ;
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	hMSL = NAV_POSLLH[4];

	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	hAcc = NAV_POSLLH[5];
	
	src.B[0] = data[data_count++];
	src.B[1] = data[data_count++];
	src.B[2] = data[data_count++];
	src.B[3] = data[data_count++];
	NAV_POSLLH[src_count++] = src.D;
	vAcc = NAV_POSLLH[6];	
}

