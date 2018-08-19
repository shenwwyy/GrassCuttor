
#include "main.h"
#include "stm32f4xx_hal.h"
#include "protocol.h"
#include "uart.h"
#include "control.h"
/*==========================
命令定义
===========================*/
_cmd ProtocolCMD;


/*====================================================================
 *CRC Function From Here
 *@HM
 *
 *====================================================================
 */
 uint8_t NewBuffer[83];
 uint16_t SBG_CRC(const void *pBuffer, uint16_t bufferSize)
{
		const uint8_t *pBytesArray = (const uint8_t*)pBuffer;
		uint16_t poly = 0x8408;
		uint16_t crc = 0;
		uint8_t carry;
		uint8_t i_bits;
		uint16_t j;
	
	  for(int i = 0;i<(bufferSize - 5);i++)
	  {
			 NewBuffer[i] = pBytesArray[i+2];
    }
	
		for (j =0; j < (bufferSize-5); j++)
		{
				crc = crc ^ NewBuffer[j];
				for (i_bits = 0; i_bits < 8; i_bits++)
				{
					carry = crc & 1;
					crc = crc / 2;
					if (carry)
					{
					crc = crc^poly;
					}
				}
		}
		return crc;
}
 
 
 
 
uint16_t CRC16_TABLE[] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

uint16_t CyclicRedundancyCheck(unsigned char *data)
{
    uint16_t crc = 0;
	  uint8_t  LEN =  data[3]+4-1;
		int i = 0;
		unsigned char temp;
	
		while (LEN-- != 0)
		{
				temp = (unsigned char)(crc >> 8); 
				crc <<= 8;
				crc ^= CRC16_TABLE[temp ^ data[i]]; 
				i++;
		}
		return crc;
}



/*====================================================================
 *Transmit Function From Here
 *@HM
 *
 *====================================================================
 */


//uint8_t Protocol_TxBuff[256];//发送的缓存

void Protocol_DataCombin(uint8_t *pData,uint16_t Size)  
{	  
	 for(uint16_t i = 0;i<Size;i++)
	{
		  Control.Car.HLink.txbuff[Control.Car.HLink.t_head] = pData[i];
	    Control.Car.HLink.t_head++;
		  if(Control.Car.HLink.t_head >= sizeof(Control.Car.HLink.txbuff))
			{
				Control.Car.HLink.t_head = 0;
			}
	}
}

void Protocol_Transmit(float T)
{
	   static float Time = 0;
	   static uint8_t RSSI = 0;
	   Time +=T;
	
	   //发送经纬度
	   if(Time>=0.5f)
		 {
			   Time = 0;
				 Protocol_T_Remote(0,
													 0, 0, 0,
													 0, 0, 0,
													 0, 0, 0,
													 0, 0, 0,
													 0, 0, 0,
													 0, 0, 0,
													 Control.Senser.GPS.fix,Control.Senser.GPS.latitude,Control.Senser.GPS.longitude,Control.Senser.GPS.altitude,Control.Senser.GPS.course,
													 Control.Senser.GPS.fix,Control.Senser.GPS.latitude,Control.Senser.GPS.longitude,Control.Senser.GPS.altitude,Control.Senser.GPS.course,
													 0,0,0,
													 0, 0,
													 Control.Senser.GPS.vn,Control.Senser.GPS.ve,Control.Senser.GPS.vd,
													 Control.Senser.GPS.vn,Control.Senser.GPS.ve,Control.Senser.GPS.vd,
													 Control.Senser.Voltage.Battery1.Battery,Control.Senser.Voltage.Battery1.Battery,Control.Senser.Voltage.Battery1.Battery,
													 0,0,0,0,0,
													 0,0,0,0,0,
													 0,0,0,0,0,
													 0,0,0,
													 0,0,0,
													 0,0,0,0,
													 0,
													 0,0,
													 0,0);
			}
	
	
	
	
		 //发送航点
		 if(ProtocolCMD.dot.sendDot)
		 {
			 ProtocolCMD.dot.sendDotCount += T;
			 if(ProtocolCMD.dot.sendDotCount >= 0.5f)
			 {
				   ProtocolCMD.dot.sendDotCount = 0.0f;
					 if(Protocol_RouteDot[ProtocolCMD.dot.sendDotGroup][ProtocolCMD.dot.sendDotID][1] == 0)
					 {
						 Protocol_T_Dot(0,//Function
														0,//Group
														0,//TotalPoint
														0,//CurrentPoint
														0,//Longitude
														0,//Latitude
														0,//Altitude
														0,//Velocity
														0,//Radius
														0);//Action 
						 
					 }
					 else
					 {
									Protocol_T_Dot(Protocol_RouteDot[ProtocolCMD.dot.sendDotGroup][ProtocolCMD.dot.sendDotID][0],//Function
																																										 ProtocolCMD.dot.sendDotGroup,//Group
																 Protocol_RouteDot[ProtocolCMD.dot.sendDotGroup][ProtocolCMD.dot.sendDotID][1],//TotalPoint
																																										 ProtocolCMD.dot.sendDotID,//CurrentPoint
																 Protocol_RouteDot[ProtocolCMD.dot.sendDotGroup][ProtocolCMD.dot.sendDotID][2],//Longitude
																 Protocol_RouteDot[ProtocolCMD.dot.sendDotGroup][ProtocolCMD.dot.sendDotID][3],//Latitude
																 Protocol_RouteDot[ProtocolCMD.dot.sendDotGroup][ProtocolCMD.dot.sendDotID][4],//Altitude
																 Protocol_RouteDot[ProtocolCMD.dot.sendDotGroup][ProtocolCMD.dot.sendDotID][5],//Velocity
																 Protocol_RouteDot[ProtocolCMD.dot.sendDotGroup][ProtocolCMD.dot.sendDotID][6],//Radius
																 Protocol_RouteDot[ProtocolCMD.dot.sendDotGroup][ProtocolCMD.dot.sendDotID][7]);//Action 
					 }
			}
		 }
	   
		 //发送航点接收到标志
		 if(ProtocolCMD.dot.GetDot)
		 {
			  union {uint8_t B[2];uint16_t D;}src;
				src.D = ProtocolCMD.dot.GetDotID;
			  ProtocolCMD.dot.GetDot = 0;
			  Protocol_T_Echo(0x04,0x07,src.B[0],src.B[1],0,0,0,0,0);
		 }
			 
		 
		 
	
	
}	


//Protocol Transmit
void Protocol_T_Version(uint16_t Hardware,uint16_t Software,uint16_t Protocol,uint16_t Airplane,uint8_t *AirplaneName)
{
	    union {uint8_t B[2];uint16_t D;}src;
	    uint8_t DataToSend[22];
	    uint8_t DataCount = 0;
	
			//Function;
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0x00;//ID
			DataToSend[DataCount++] = 0x00;//LEN
			DataToSend[DataCount++] = 0x55;//RSSI
	
			//Data
	    src.D = Hardware * 100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
	
			src.D = Software * 100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.D = Protocol * 100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.D = Airplane * 100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			DataToSend[DataCount++] = AirplaneName[0];
			DataToSend[DataCount++] = AirplaneName[1];
			DataToSend[DataCount++] = AirplaneName[2];
			DataToSend[DataCount++] = AirplaneName[3];
			DataToSend[DataCount++] = AirplaneName[4];
			DataToSend[DataCount++] = AirplaneName[5];
			DataToSend[DataCount++] = AirplaneName[6];
			DataToSend[DataCount++] = AirplaneName[7];//20
			
			//LEN
			DataToSend[3] = DataCount-4;//21
	
	    //SUM
			uint8_t SUM=0;
			for(int i = 0;i<DataCount;i++)
			{
				SUM += DataToSend[i];
			}
	    DataToSend[DataCount++] = SUM;
			
			//Send
			Protocol_DataCombin(DataToSend,DataCount);
}


void Protocol_T_Echo(uint8_t ID,uint8_t Echo1,uint8_t Echo2,uint8_t Echo3,uint8_t Echo4,uint8_t Echo5,uint8_t Echo6,uint8_t Echo7,uint8_t Echo8)
{
	    uint8_t DataToSend[50];
	    uint8_t DataCount = 0;
	    
			if((ID<1)&&(ID > 0x0A)) return;//如果出现ID号为0的情况立马返回
			
			//Function;
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = ID;//ID
			DataToSend[DataCount++] = 0x00;//LEN
			DataToSend[DataCount++] = 0x00;//RSSI
	
			//Data
			DataToSend[DataCount++] = Echo1;
			DataToSend[DataCount++] = Echo2;
	    DataToSend[DataCount++] = Echo3;
			DataToSend[DataCount++] = Echo4;
			DataToSend[DataCount++] = Echo5;
			DataToSend[DataCount++] = Echo6;
			DataToSend[DataCount++] = Echo7;
			DataToSend[DataCount++] = Echo8;

			//LEN
			DataToSend[3] = DataCount-4;
	
	    //SUM
			uint8_t SUM=0;
			for(uint8_t i = 0;i<DataCount;i++)
			{
				SUM += DataToSend[i];
			}
	    DataToSend[DataCount++] = SUM;
			
			//Send
			Protocol_DataCombin(DataToSend,DataCount);
}


void Protocol_T_Remote(uint8_t IMUSelect,
	                     float ACC1_X, float ACC1_Y, float ACC1_Z,
                       float GYRO1_X,float GYRO1_Y,float GYRO1_Z,
                       float ROLL1,  float PITCH1, float YAW1,
                       float ACC2_X, float ACC2_Y, float ACC2_Z,
                       float GYRO2_X,float GYRO2_Y,float GYRO2_Z,
                       float ROLL2,  float PITCH2, float YAW2,
                       uint8_t GPS1_Status,double Latitude1,double Longitude1,float Altitude1,float Course1,
											 uint8_t GPS2_Status,double Latitude2,double Longitude2,float Altitude2,float Course2,
											 uint8_t Hour,uint8_t Minute,uint8_t Second,
											 uint8_t SpeedSelect, float AirSpeed,
											 float NorthSpeed1,float EastSpeed1,float DownSpeed1,
											 float NorthSpeed2,float EastSpeed2,float DownSpeed2,
											 float VotageMultiRotor,float VotageFixedWing,float VotageSystem,
											 uint8_t isRCValid,  float ROL,      float PIT,    float THR,     float RUD,
											 float RotorThrottle,float Rotor1,   float Rotor2, float Rotor3,  float Rotor4,
											 float Throttle1,    float Throttle2,float Aileron,float Elevator,float Rudder,
											 uint16_t CurrentTargetPoint,uint16_t WaitToFlight,float DiffrentDistance,
											 float DiffrentAngle,float DiffrentHeigh,float DiffrentSpeed,
											 float TargetRoll,   float TargetPitch,  float TargetYaw,float TargetHeigh,
											 float RelativeHeight,
											 uint8_t Status1,uint8_t Status2,
											 uint8_t Command1,uint8_t Command2)
{
	    static uint8_t RSSI = 0;//帧计数
		  union {uint8_t B[8];uint16_t D[4];int16_t W[4];float F[2];double DD;}src;
	    uint8_t DataToSend[250];
	    uint8_t DataCount = 0;
	
			//Function;
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0x0B;//ID
			DataToSend[DataCount++] = 0x00;//LEN
			DataToSend[DataCount++] = RSSI++;//RSSI
	
			//Data
			DataToSend[DataCount++] = IMUSelect;
			//Senser1
			src.W[0] = ACC1_X*1000;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
	
			src.W[0] = ACC1_Y*1000;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = ACC1_Z*1000;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = GYRO1_X*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
	
			src.W[0] = GYRO1_Y*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = GYRO1_Z*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = ROLL1*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
	
			src.W[0] = PITCH1*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = YAW1*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			//Senser2
			src.W[0] = ACC2_X*1000;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
	
			src.W[0] = ACC2_Y*1000;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = ACC2_Z*1000;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = GYRO2_X*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
	
			src.W[0] = GYRO2_Y*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = GYRO2_Z*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = ROLL2*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
	
			src.W[0] = PITCH2*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = YAW2*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
      //GPS1
			DataToSend[DataCount++] = GPS1_Status;
			
			src.DD = Latitude1;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			DataToSend[DataCount++] = src.B[4];
			DataToSend[DataCount++] = src.B[5];
			DataToSend[DataCount++] = src.B[6];
			DataToSend[DataCount++] = src.B[7];
			
			src.DD = Longitude1;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			DataToSend[DataCount++] = src.B[4];
			DataToSend[DataCount++] = src.B[5];
			DataToSend[DataCount++] = src.B[6];
			DataToSend[DataCount++] = src.B[7];
			
			src.W[0] = Altitude1*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Course1*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			//GPS2
			DataToSend[DataCount++] = GPS2_Status;
			
			src.DD = Latitude2;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			DataToSend[DataCount++] = src.B[4];
			DataToSend[DataCount++] = src.B[5];
			DataToSend[DataCount++] = src.B[6];
			DataToSend[DataCount++] = src.B[7];
			
			src.DD = Longitude2;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			DataToSend[DataCount++] = src.B[4];
			DataToSend[DataCount++] = src.B[5];
			DataToSend[DataCount++] = src.B[6];
			DataToSend[DataCount++] = src.B[7];
			
			src.W[0] = Altitude2*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Course2*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			//Time
			DataToSend[DataCount++] = Hour;
			DataToSend[DataCount++] = Minute;
			DataToSend[DataCount++] = Second;
			//Speed
			DataToSend[DataCount++] = SpeedSelect;
			
			src.W[0] = AirSpeed*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			//NED Speed1
			src.W[0] = NorthSpeed1*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = EastSpeed1*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = DownSpeed1*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			//NED Speed2
			src.W[0] = NorthSpeed2*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = EastSpeed2*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = DownSpeed2*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			//Votage  
			src.D[0] = VotageMultiRotor*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.D[0] = VotageFixedWing*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.D[0] = VotageSystem*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			//RC
			DataToSend[DataCount++] = isRCValid;
			
			src.W[0] = ROL*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = PIT*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = THR*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = RUD*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			//OUT
			src.W[0] = RotorThrottle*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Rotor1*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Rotor2*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Rotor3*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Rotor4*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Throttle1*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Throttle2*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Aileron*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Elevator*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = Rudder*100;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			//Target
			src.D[0] = CurrentTargetPoint;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.D[0] = WaitToFlight;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = DiffrentDistance*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = DiffrentAngle*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = DiffrentHeigh*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = DiffrentSpeed*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = TargetRoll*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = TargetPitch*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = TargetYaw*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = TargetHeigh*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.W[0] = RelativeHeight*10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			DataToSend[DataCount++] = Status1;
			DataToSend[DataCount++] = Status2;
			
			DataToSend[DataCount++] = Command1;
			DataToSend[DataCount++] = Command2;
			
			//LEN
			DataToSend[3] = DataCount-4;
	
	    //SUM
			uint8_t SUM=0;
			for(int i = 0;i<DataCount;i++)
			{
				SUM += DataToSend[i];
			}
	    DataToSend[DataCount++] = SUM;
			
			//Send
			Protocol_DataCombin(DataToSend,DataCount);
}




void Protocol_T_PID(uint8_t ID,float PID1_P,float PID1_I,float PID1_D,
	                             float PID2_P,float PID2_I,float PID2_D,
															 float PID3_P,float PID3_I,float PID3_D,
															 float PID4_P,float PID4_I,float PID4_D,
															 float PID5_P,float PID5_I,float PID5_D,
															 float PID6_P,float PID6_I,float PID6_D)
{
      union {uint8_t B[4];float F;}src;
	    uint8_t DataToSend[80];
	    uint8_t DataCount = 0;
	    
			if((ID<1)&&(ID > 0x10)) return;//如果出现ID号为0的情况立马返回
			
			//Function;
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0X30 + ID;//ID
			DataToSend[DataCount++] = 0x00;//LEN
			DataToSend[DataCount++] = 0x00;//RSSI
	
			//Data
	    src.F = PID1_P;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = PID1_I;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID1_D;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID2_P;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = PID2_I;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID2_D;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID3_P;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = PID3_I;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID3_D;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID4_P;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = PID4_I;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID4_D;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID5_P;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = PID5_I;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID5_D;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID6_P;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = PID6_I;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = PID6_D;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			//LEN
			DataToSend[3] = DataCount-4;
	
	    //SUM
			uint8_t SUM=0;
			for(int i = 0;i<DataCount;i++)
			{
				SUM += DataToSend[i];
			}
	    DataToSend[DataCount++] = SUM;
			
			//Send
			Protocol_DataCombin(DataToSend,DataCount);
}



void Protocol_T_Dot(uint8_t Function,uint8_t Groups,
	                  uint16_t TotalPoint,uint16_t CurrentPoint,
                    double Longitude,double Latitude,float Altitude,
									  uint16_t Velocity,uint16_t Radius,uint16_t Action)
{
	    union {uint8_t B[8];uint16_t D[4];int16_t W[4];double DD;}src;
	    uint8_t DataToSend[37];
	    uint8_t DataCount = 0;
			
			//Function;
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0x41;//ID
			DataToSend[DataCount++] = 0x00;//LEN
			DataToSend[DataCount++] = 0x00;//RSSI

			//Data
			DataToSend[DataCount++] = Function;
			DataToSend[DataCount++] = Groups;
			
			src.D[0] = TotalPoint;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.D[0] = CurrentPoint;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.DD = Longitude;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			DataToSend[DataCount++] = src.B[4];
			DataToSend[DataCount++] = src.B[5];
			DataToSend[DataCount++] = src.B[6];
			DataToSend[DataCount++] = src.B[7];
			
			src.DD = Latitude;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			DataToSend[DataCount++] = src.B[4];
			DataToSend[DataCount++] = src.B[5];
			DataToSend[DataCount++] = src.B[6];
			DataToSend[DataCount++] = src.B[7];
	
	    src.W[0] = Altitude * 10;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.D[0] = Velocity;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.D[0] = Radius;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			src.D[0] = Action;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];

			//LEN
			DataToSend[3] = DataCount -4 + 1;
	
	    //CRC	
			src.D[0] = CyclicRedundancyCheck(DataToSend);
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			
			//Send
			Protocol_DataCombin(DataToSend,DataCount);
}	


void Protocol_T_TaskSetting(uint8_t ID,
	                         float Setting1,float Setting2,float Setting3,float Setting4,
													 float Setting5,float Setting6,float Setting7,float Setting8)
{
			union {uint8_t B[4];float F;}src;
	    uint8_t DataToSend[50];
	    uint8_t DataCount = 0;
	    
			if((ID<1)&&(ID > 0x0F)) return;//如果出现ID号为0的情况立马返回
			
			//Function;
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0X50 + ID;//ID
			DataToSend[DataCount++] = 0x00;//LEN
			DataToSend[DataCount++] = 0x00;//RSSI

	
			//Data
	    src.F = Setting1;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = Setting2;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = Setting3;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = Setting4;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = Setting5;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = Setting6;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = Setting7;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = Setting8;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			//LEN
			DataToSend[3] = DataCount - 4;
	
	    //SUM
			uint8_t SUM=0;
			for(int i = 0;i<DataCount;i++)
			{
				SUM += DataToSend[i];
			}
	    DataToSend[DataCount++] = SUM;
			
			//Send
			Protocol_DataCombin(DataToSend,DataCount);									 				
}
													 

void Protocol_T_Calibration(uint8_t SenserType,
	                          float X,float Y,float Z)
{
			union {uint8_t B[4];float F;}src;
	    uint8_t DataToSend[50];
	    uint8_t DataCount = 0;
	    
			if((SenserType<1)&&(SenserType > 0x0F)) return;//如果出现ID号为0的情况立马返回
			
			//Function;
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0XA0;//ID
			DataToSend[DataCount++] = 0x00;//LEN
			DataToSend[DataCount++] = 0x00;//RSSI

	    DataToSend[DataCount++] = SenserType;//传感器类型
			//Data
	    src.F = X;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = Y;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = Z;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			//LEN
			DataToSend[3] = DataCount - 4;
	
	    //SUM
			uint8_t SUM=0;
			for(int i = 0;i<DataCount;i++)
			{
				SUM += DataToSend[i];
			}
	    DataToSend[DataCount++] = SUM;
			
			//Send
			Protocol_DataCombin(DataToSend,DataCount);									 				
}


void Protocol_T_BiasPrameter(uint8_t SenserType,uint8_t DataType,
	                           float X,float Y,float Z)
{
			union {uint8_t B[4];float F;}src;
	    uint8_t DataToSend[50];
	    uint8_t DataCount = 0;
	    
			if((SenserType<1)&&(SenserType > 0x0F)) return;//如果出现ID号为0的情况立马返回
			
			//Function;
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0xAA;//
	    DataToSend[DataCount++] = 0XA1;//ID
			DataToSend[DataCount++] = 0x00;//LEN
			DataToSend[DataCount++] = 0x00;//RSSI

	    DataToSend[DataCount++] = SenserType;//传感器类型
			DataToSend[DataCount++] = DataType;//数据类型
			//Data
	    src.F = X;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
	
			src.F = Y;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			src.F = Z;
			DataToSend[DataCount++] = src.B[0];
			DataToSend[DataCount++] = src.B[1];
			DataToSend[DataCount++] = src.B[2];
			DataToSend[DataCount++] = src.B[3];
			
			//LEN
			DataToSend[3] = DataCount - 4;
	
	    //SUM
			uint8_t SUM=0;
			for(int i = 0;i<DataCount;i++)
			{
				SUM += DataToSend[i];
			}
	    DataToSend[DataCount++] = SUM;
			
			//Send
			Protocol_DataCombin(DataToSend,DataCount);									 				
}




void Protocol_T_ServoPolar(uint8_t ID, uint8_t PWM1, uint8_t PWM2, uint8_t PWM3,
                            uint8_t PWM4, uint8_t PWM5, uint8_t PWM6, uint8_t PWM7,
                            uint8_t PWM8, uint8_t PWM9, uint8_t PWM10, uint8_t PWM11,
                            uint8_t PWM12, uint8_t PWM13, uint8_t PWM14, uint8_t PWM15,
                            uint8_t PWM16, uint8_t Freq1, uint8_t Freq2, uint8_t Freq3, uint8_t Freq4)
{
    uint8_t DataToSend[250];
    uint16_t DataCount = 0;

    DataToSend[DataCount++] = 0xAA;//0
    DataToSend[DataCount++] = 0xAA;
    DataToSend[DataCount++] = 0XB2;//ID
    DataToSend[DataCount++] = 0x00;//LEN
    DataToSend[DataCount++] = 0x00;//RSSI

    DataToSend[DataCount++] = ID;//ID

    DataToSend[DataCount++] = PWM1;
    DataToSend[DataCount++] = PWM2;
    DataToSend[DataCount++] = PWM3;
    DataToSend[DataCount++] = PWM4;
    DataToSend[DataCount++] = PWM5;
    DataToSend[DataCount++] = PWM6;
    DataToSend[DataCount++] = PWM7;
    DataToSend[DataCount++] = PWM8;
    DataToSend[DataCount++] = PWM9;
    DataToSend[DataCount++] = PWM10;
    DataToSend[DataCount++] = PWM11;
    DataToSend[DataCount++] = PWM12;
    DataToSend[DataCount++] = PWM13;
    DataToSend[DataCount++] = PWM14;
    DataToSend[DataCount++] = PWM15;
    DataToSend[DataCount++] = PWM16;
    DataToSend[DataCount++] = Freq1;
    DataToSend[DataCount++] = Freq2;
    DataToSend[DataCount++] = Freq3;
    DataToSend[DataCount++] = Freq4;

    //==========LEN================
    DataToSend[3] =DataCount - 4 ;
     //==========SUM================
   uint8_t sum = 0;
   for(uint8_t i=0;i<DataCount;i++)
     sum += DataToSend[i];
   DataToSend[DataCount++]=sum;
     //=========Send=================
   //qDebug() << DataToSend;
   Protocol_DataCombin(DataToSend,DataCount);
}

void Protocol_T_ServoPolynomial(uint8_t ID,
                                 float Poly_A_PWM1, float Poly_B_PWM1, float Poly_C_PWM1,
                                 float Poly_A_PWM2, float Poly_B_PWM2, float Poly_C_PWM2,
                                 float Poly_A_PWM3, float Poly_B_PWM3, float Poly_C_PWM3,
                                 float Poly_A_PWM4, float Poly_B_PWM4, float Poly_C_PWM4,
                                 float Poly_A_PWM5, float Poly_B_PWM5, float Poly_C_PWM5,
                                 float Poly_A_PWM6, float Poly_B_PWM6, float Poly_C_PWM6,
                                 float Poly_A_PWM7, float Poly_B_PWM7, float Poly_C_PWM7,
                                 float Poly_A_PWM8, float Poly_B_PWM8, float Poly_C_PWM8,
                                 float Poly_A_PWM9, float Poly_B_PWM9, float Poly_C_PWM9,
                                 float Poly_A_PWM10, float Poly_B_PWM10, float Poly_C_PWM10,
                                 float Poly_A_PWM11, float Poly_B_PWM11, float Poly_C_PWM11,
                                 float Poly_A_PWM12, float Poly_B_PWM12, float Poly_C_PWM12,
                                 float Poly_A_PWM13, float Poly_B_PWM13, float Poly_C_PWM13,
                                 float Poly_A_PWM14, float Poly_B_PWM14, float Poly_C_PWM14,
                                 float Poly_A_PWM15, float Poly_B_PWM15, float Poly_C_PWM15,
                                 float Poly_A_PWM16, float Poly_B_PWM16, float Poly_C_PWM16,
                                 float Poly_A_ROTOR1, float Poly_B_ROTOR1, float Poly_C_ROTOR1,
                                 float Poly_A_ROTOR2, float Poly_B_ROTOR2, float Poly_C_ROTOR2,
                                 float Poly_A_ROTOR3, float Poly_B_ROTOR3, float Poly_C_ROTOR3,
                                 float Poly_A_ROTOR4, float Poly_B_ROTOR4, float Poly_C_ROTOR4,
                                 float Poly_A_ROTOR5, float Poly_B_ROTOR5, float Poly_C_ROTOR5)
{
    union {uint8_t B[2]; int16_t W;}src;
    uint8_t DataToSend[250];
    uint16_t DataCount = 0;

    DataToSend[DataCount++] = 0xAA;//0
    DataToSend[DataCount++] = 0xAA;
    DataToSend[DataCount++] = 0xB3;//ID
    DataToSend[DataCount++] = 0x00;//LEN
    DataToSend[DataCount++] = 0x00;//RSSI

    DataToSend[DataCount++] = ID;//ID

    //=================================
    src.W = Poly_A_PWM1 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM1 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM1 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM2 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM2 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM2 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM3 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM3 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM3 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM4 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM4 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM4 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM5 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM5 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM5 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM6 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM6 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM6 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM7 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM7 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM7 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM8 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM8 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM8 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM9 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM9 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM9 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM10 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM10 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM10 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM11 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM11 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM11 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM12 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM12 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM12 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM13 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM13 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM13 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM14 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM14 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM14 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM15 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM15 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM15 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_PWM16 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_PWM16 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_PWM16 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_ROTOR1 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_ROTOR1 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_ROTOR1 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_ROTOR2 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_ROTOR2 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_ROTOR2 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_ROTOR3 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_ROTOR3 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_ROTOR3 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_ROTOR4 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_ROTOR4 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_ROTOR4 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    //=================================
    src.W = Poly_A_ROTOR5 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_B_ROTOR5 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];
    src.W = Poly_C_ROTOR5 * 1000;
    DataToSend[DataCount++] = src.B[0];
    DataToSend[DataCount++] = src.B[1];


    //==========LEN================
    DataToSend[3] =DataCount - 4 ;
    //==========SUM================
   uint8_t sum = 0;
   for(uint8_t i=0;i<DataCount;i++)
     sum += DataToSend[i];
   DataToSend[DataCount++]=sum;
     //=========Send=================

   //qDebug() << DataToSend;
   Protocol_DataCombin(DataToSend,DataCount);
}







/*====================================================================
 *Receive Function From Here
 *@HM
 *
 *====================================================================
 */

#define BUFFER_MAX  256               //缓冲区大小

typedef struct 
{
  uint16_t headPosition;        //缓冲区头部位置
  uint16_t tailPositon;         //缓冲区尾部位置
  uint8_t ringBuf[BUFFER_MAX]; //缓冲区数组
}ringBuffer_t;

ringBuffer_t Protocol_Ringbuffer; //定义一个结构体

/**
* @brief 写一个字节到环形缓冲区
* @param data：待写入的数据
* @return none
*/
void Protocol_RingBuf_Write(uint8_t data)
{
  Protocol_Ringbuffer.ringBuf[Protocol_Ringbuffer.tailPositon]=data;     //从尾部追加
  if(++Protocol_Ringbuffer.tailPositon>=BUFFER_MAX)         //尾节点偏移
    Protocol_Ringbuffer.tailPositon=0;                      //大于数组最大长度 归零 形成环形队列
  if(Protocol_Ringbuffer.tailPositon == Protocol_Ringbuffer.headPosition)//如果尾部节点追到头部节点，则修改头节点偏移位置丢弃早期数据
    if(++Protocol_Ringbuffer.headPosition>=BUFFER_MAX)
      Protocol_Ringbuffer.headPosition=0;
}

/**
* @brief 读取环形缓冲区的一个字节的数据
* @param *pData:指针，用于保存读取到的数据
* @return 1表示缓冲区是空的，0表示读取数据成功
*/
uint8_t Protocol_RingBuf_Read(uint8_t* pData,uint16_t* pLen)
{
  if(Protocol_Ringbuffer.headPosition == Protocol_Ringbuffer.tailPositon)    //如果头尾接触表示缓冲区为空
  {
		        *pLen = 0;
            return 1;   //返回1，环形缓冲区是空的
  }
  else
  {
    
		if(Protocol_Ringbuffer.tailPositon < Protocol_Ringbuffer.headPosition)//如果尾小于头，说明尾已经从0开始了
		{
			*pLen = Protocol_Ringbuffer.tailPositon + BUFFER_MAX - Protocol_Ringbuffer.headPosition;//计算出长度
		}
		else//正常情况
		{
		  *pLen = Protocol_Ringbuffer.tailPositon - Protocol_Ringbuffer.headPosition;//计算出长度
		}
		
		while(Protocol_Ringbuffer.tailPositon != Protocol_Ringbuffer.headPosition)
		{
			 *pData++ =Protocol_Ringbuffer.ringBuf[Protocol_Ringbuffer.headPosition];    //如果缓冲区非空则取头节点值并偏移头节点
		    
			 if(++Protocol_Ringbuffer.headPosition>=BUFFER_MAX)
       Protocol_Ringbuffer.headPosition=0;
		
		}
    return 0;     //返回0，表示读取数据成功
  }
}


static unsigned char CheckSum(uint8_t *buff)
{   
	  unsigned char i,ReturnVelue;
	  uint16_t sum = 0;
	  for(i = 0;i<(buff[3]+4);i++)
	  {
			  sum += buff[i]; 
    }
		ReturnVelue = (sum&0xff);
		
		return ReturnVelue;
}


uint8_t Protocol_RxBuffer[256];
uint8_t ProState = 0;
void Protocol_Prepare(uint8_t data)
{	
	static uint8_t _data_len = 0,_data_cnt = 0;
	if(ProState==0&&data==0xEB)//第一个字
	{
			   ProState=1;
			   Protocol_RxBuffer[0]=data;
	}
	else if(ProState==1&&data==0x90)//第二个字
	{
			  ProState=2;   
			  Protocol_RxBuffer[1]=data;
	}
	else if(ProState==2)//第三个字id号
	{
		    ProState=3;
		    Protocol_RxBuffer[2]=data;		
	}
	else if(ProState==3)//第四个字,长度小于256
	{
		  _data_len = data + 1;
		  _data_cnt = 4;
		  ProState=4;
		  Protocol_RxBuffer[3]=data;//LEN
	}
	else if(ProState==4&&_data_len>0)
	{
		_data_len--;
		Protocol_RxBuffer[_data_cnt++]=data;
		if(_data_len==0)
		{
				if((Protocol_RxBuffer[2] != 0x41)&&(CheckSum(Protocol_RxBuffer) == Protocol_RxBuffer[_data_cnt - 1]))//
				{
					 //解码
						switch(Protocol_RxBuffer[2])//ID 
							 {   
										case 0x00:{ Protocol_R_Beat(Protocol_RxBuffer);      }break;//Beat
										case 0x01:{ Protocol_R_CMD(Protocol_RxBuffer);       }break;//cmd
										case 0x02:{ Protocol_R_CMD(Protocol_RxBuffer);       }break;//cmd
										case 0x03:{ Protocol_R_CMD(Protocol_RxBuffer);       }break;//cmd
										case 0x04:{ Protocol_R_CMD(Protocol_RxBuffer);       }break;//cmd
										case 0x05:{ Protocol_R_CMD(Protocol_RxBuffer);       }break;//cmd
										case 0x06:{ Protocol_R_CMD(Protocol_RxBuffer);       }break;//cmd
										case 0x07:{ Protocol_R_CMD(Protocol_RxBuffer);       }break;//cmd
										
										
										default  :{ ProState=0;}break;//unfine
							 }		 
					ProState = 0;
				}
				else if(Protocol_RxBuffer[2] == 0x41)
			  {  
					Protocol_R_Dot(Protocol_RxBuffer);//GetRoute 这个是CRC校验
					ProState = 0;
			  }	
		}
	}
	else
	{
		ProState = 0;
	}

}


//======================Beat================================
unsigned char HeartBeat;
void Protocol_R_Beat(uint8_t *data)//00
{
	 //if do not recieve beat in 20sec ,it will be out of the distance of Station
	 HeartBeat  = data[5];
}

//===================CMD==========================
void Protocol_R_CMD(uint8_t *data)//00
{
		uint8_t ID;
		uint8_t CMDA,CMDB,CMDC,CMDD,CMDE,CMDF,CMDG,CMDH;

		uint8_t DataCount = 5;
		
		ID = data[2];
		
		CMDA = data[DataCount++];
		CMDB = data[DataCount++];
		CMDC = data[DataCount++];
		CMDD = data[DataCount++];
		CMDE = data[DataCount++];
		CMDF = data[DataCount++];
		CMDG = data[DataCount++];
		CMDH = data[DataCount++];

	  CMDA = CMDA;
		CMDB = CMDB;
		CMDC = CMDC;
		CMDD = CMDD;
		CMDE = CMDE;
		CMDF = CMDF;
		CMDG = CMDG;
		CMDH = CMDH;
	
	  switch(ID)
		{
			case 0x04:
				{  
					 if((CMDA&0x01) == 0x01)
					 {
						 ProtocolCMD.dot.sendDot   = 1;
						 ProtocolCMD.dot.sendDotGroup = (CMDA&0xFE) >> 1;
						 ProtocolCMD.dot.sendDotID ++;
					 }
					 else
					 {
						 ProtocolCMD.dot.sendDot   = 0;
						 ProtocolCMD.dot.sendDotID = 0;//取消或者完成传输时，把ID号清零，以便于下次传输
					 }
				}break;
		}

	
}


//Route Dot

double Protocol_RouteDot[5][200][8] = {0};//7.168kb大小

void Protocol_R_Dot(uint8_t *data)
{
	   uint8_t Function,Groups;
     uint16_t TotalPoint,CurrentPoint;
     double Longitude,Latitude;
	   float Altitude;
     uint16_t Velocity,Radius,Action;
	
	   union {uint8_t B[8];uint16_t D[4];int16_t W[4];double DD;}src;
		 
		 uint16_t CRC_sum = CyclicRedundancyCheck(data);
		 uint16_t CRC_data;
		 uint16_t DataCount = 5;
		 
		 src.B[0] = data[data[3] + 4 -1];
		 src.B[1] = data[data[3] + 4];
		 CRC_data = src.D[0];
		 
		 if(CRC_data == CRC_sum)
		 {
			 Function = data[DataCount++];
			 Groups   = data[DataCount++];
			 
			 src.B[0] = data[DataCount++];
			 src.B[1] = data[DataCount++];
			 TotalPoint = src.D[0];
			 
			 src.B[0] = data[DataCount++];
			 src.B[1] = data[DataCount++];
			 CurrentPoint = src.D[0];
			 
			 src.B[0] = data[DataCount++];
			 src.B[1] = data[DataCount++];
			 src.B[2] = data[DataCount++];
			 src.B[3] = data[DataCount++];
			 src.B[4] = data[DataCount++];
			 src.B[5] = data[DataCount++];
			 src.B[6] = data[DataCount++];
			 src.B[7] = data[DataCount++];
			 Longitude = src.DD;
			 
			 src.B[0] = data[DataCount++];
			 src.B[1] = data[DataCount++];
			 src.B[2] = data[DataCount++];
			 src.B[3] = data[DataCount++];
			 src.B[4] = data[DataCount++];
			 src.B[5] = data[DataCount++];
			 src.B[6] = data[DataCount++];
			 src.B[7] = data[DataCount++];
			 Latitude = src.DD;
			 
			 src.B[0] = data[DataCount++];
			 src.B[1] = data[DataCount++];
			 Altitude = src.W[0] * 0.1f;
			 
			 src.B[0] = data[DataCount++];
			 src.B[1] = data[DataCount++];
			 Velocity = src.D[0];
			 
			 src.B[0] = data[DataCount++];
			 src.B[1] = data[DataCount++];
			 Radius = src.D[0];
			 
			 src.B[0] = data[DataCount++];
			 src.B[1] = data[DataCount++];
			 Action = src.D[0];
			 
			 
			 Protocol_RouteDot[Groups][CurrentPoint][0] = Function;
			 Protocol_RouteDot[Groups][CurrentPoint][1] = TotalPoint;
			 Protocol_RouteDot[Groups][CurrentPoint][2] = Longitude;
			 Protocol_RouteDot[Groups][CurrentPoint][3] = Latitude;
			 Protocol_RouteDot[Groups][CurrentPoint][4] = Altitude;
			 Protocol_RouteDot[Groups][CurrentPoint][5] = Velocity;
			 Protocol_RouteDot[Groups][CurrentPoint][6] = Radius;
			 Protocol_RouteDot[Groups][CurrentPoint][7] = Action;
			 
			 Control.Task.PointGroupsNumber = TotalPoint;
			 Control.Task.PointGroups[CurrentPoint].Number    = CurrentPoint;
			 Control.Task.PointGroups[CurrentPoint].longitude = Longitude;
			 Control.Task.PointGroups[CurrentPoint].latitude  = Latitude;
			 Control.Task.PointGroups[CurrentPoint].altitude  = Altitude;
			 Control.Task.PointGroups[CurrentPoint].speed     = Velocity;
			 Control.Task.PointGroups[CurrentPoint].course    = Radius;
			 
			 ProtocolCMD.dot.GetDot   = 0x01;
       ProtocolCMD.dot.GetDotID = CurrentPoint;
			 
		 }
		 
}

