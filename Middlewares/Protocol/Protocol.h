

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_


#include "stm32f7xx_hal.h"

/*=================================
Protocol CMD define
==================================*/
typedef struct {
	uint8_t MultiRotorAngle;
	uint8_t MultiRotorPosition;
	uint8_t FixedWingAngle;
	uint8_t FixedWingPosition;
}_cmdpid;

typedef struct {
	uint8_t Acc;
	uint8_t Gyro;
	uint8_t Mag;
	uint8_t Airspeed;
	uint8_t Pressure;
}_cmdsenser;

typedef struct {
	
	uint8_t Takeoff;
	uint8_t Circle;
	uint8_t Cruise;
	uint8_t BackHome;
	uint8_t Landing;
	uint8_t CutOff;
	
	uint8_t SendSetting;
	uint8_t SendCount;
	
}_tasksetting;


typedef struct {
	
	uint8_t  sendDot;
	uint8_t  sendDotGroup;
	uint16_t sendDotID;
	float    sendDotCount;
	
	uint8_t  GetDot;
  uint16_t GetDotID;
}_dotsetting;

typedef struct {
	
	uint8_t  sendpolar;
	uint8_t  sendpolynomial;
	
}_outputsetting;




typedef struct {
	
	uint8_t    sendVersion;
	_cmdpid    sendPID;
	_cmdsenser sendSenser1;
	_cmdsenser completeSenser1;
	_cmdsenser sendSenser2;
	_cmdsenser completeSenser2;
	_dotsetting dot;
	_tasksetting sendTaskSetting;
	
	_outputsetting sendOutPut;
	
}_cmd;

extern _cmd ProtocolCMD;




/*====================================================================
 *Transmit Function From Here
 *@HM
 *
 *====================================================================
 */
void Protocol_Transmit(float T);


//Protocol Transmit
void Protocol_T_Version(uint16_t Hardware,uint16_t Software,uint16_t Protocol,uint16_t Airplane,uint8_t *AirplaneName);
void Protocol_T_Echo(uint8_t ID,uint8_t Echo1,uint8_t Echo2,uint8_t Echo3,uint8_t Echo4,uint8_t Echo5,uint8_t Echo6,uint8_t Echo7,uint8_t Echo8);
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
											 uint8_t Command1,uint8_t Command2);
void Protocol_T_PID(uint8_t ID,float PID1_P,float PID1_I,float PID1_D,
	                             float PID2_P,float PID2_I,float PID2_D,
															 float PID3_P,float PID3_I,float PID3_D,
															 float PID4_P,float PID4_I,float PID4_D,
															 float PID5_P,float PID5_I,float PID5_D,
															 float PID6_P,float PID6_I,float PID6_D);
void Protocol_T_Dot(uint8_t Function,uint8_t Groups,
	                    uint16_t TotalPoint,uint16_t CurrentPoint,
                      double Longitude,double Latitude,float Altitude,
											uint16_t Velocity,uint16_t Radius,uint16_t Action);
void Protocol_T_TaskSetting(uint8_t ID,
	                         float Setting1,float Setting2,float Setting3,float Setting4,
													 float Setting5,float Setting6,float Setting7,float Setting8);

void Protocol_T_Calibration(uint8_t SenserType,
	                          float X,float Y,float Z);

void Protocol_T_BiasPrameter(uint8_t SenserType,uint8_t DataType,
	                           float X,float Y,float Z);
													 

void Protocol_T_ServoPolar(uint8_t ID, uint8_t PWM1, uint8_t PWM2, uint8_t PWM3,
                            uint8_t PWM4, uint8_t PWM5, uint8_t PWM6, uint8_t PWM7,
                            uint8_t PWM8, uint8_t PWM9, uint8_t PWM10, uint8_t PWM11,
                            uint8_t PWM12, uint8_t PWM13, uint8_t PWM14, uint8_t PWM15,
                            uint8_t PWM16, uint8_t Freq1, uint8_t Freq2, uint8_t Freq3, uint8_t Freq4);

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
                                 float Poly_A_ROTOR5, float Poly_B_ROTOR5, float Poly_C_ROTOR5);



/*====================================================================
 *Receive Function From Here
 *@HM
 *
 *====================================================================
 */			


																 
																 
																 
extern double Protocol_RouteDot[5][200][8];


void Protocol_RingBuf_Write(uint8_t data);																 
uint8_t Protocol_RingBuf_Read(uint8_t* pData,uint16_t* pLen);
																 
void Protocol_Prepare(uint8_t data);
													 
void Protocol_R_Beat(uint8_t *data);//00
void Protocol_R_CMD(uint8_t *data);//01~09
void Protocol_R_PID(uint8_t *data);
void Protocol_R_Dot(uint8_t *data);//0x41

void Protocol_R_TaskSetting(uint8_t *data);//0x50


void Protocol_R_Scale(uint8_t *data);//0xA1
void Protocol_R_ServoPolar(uint8_t *data);//0xB2
void Protocol_R_ServoPoly(uint8_t *data);//0XB3





#endif










