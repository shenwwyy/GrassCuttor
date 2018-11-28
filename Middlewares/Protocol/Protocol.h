

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_


#include "stm32f4xx_hal.h"
#include "stdbool.h"

/*=================================
Protocol CMD define
==================================*/

#define TXBUFFSIZE 512
#define RXBUFFSIZE 512

typedef struct {
	//Ö÷»º´æ
	uint8_t txbuf[TXBUFFSIZE];
	uint16_t txhead;
	//±¸·Ý»º´æ
	uint8_t txbuf_bak[TXBUFFSIZE];
	uint16_t txhead_bak;
	uint8_t tx_sel;
	uint16_t tx_max;
	
	uint8_t rxbuf[RXBUFFSIZE];
	uint8_t rxtemp;
	uint16_t rxtail;
	uint16_t rxhead;
	
	uint8_t isReadyToSend;
	uint8_t isRecieveReady;
	
	uint8_t txsyn;
	uint8_t rxsyn;
	
}_uart;

typedef struct {
		uint8_t version;
		uint8_t heardbeat;
		uint8_t id;
}_heardbeat;


typedef struct {
	float speed_Kp;
	float speed_Ki;
	float speed_Kd;

	float distance_Kp;
	float distance_Ki;
	float distance_Kd;

	float heading_Kp;
	float heading_Ki;
	float heading_Kd;

}_parameter;

typedef struct {
		uint8_t fixtype;
		uint8_t svn;
		float  altitude;
		double latitude;
		double longitude;
		float  groundspeed;
		float  course;
}_gps;

typedef struct {
		float left;
		float front;
		float right;
}_ultrasonic;

typedef struct {
		uint16_t id;
		uint16_t type;//0 point 1 line 2 circle
		uint16_t action;
		float altitude;
		double latitude;
		double longitude;
		float speed;
		float course;
}_waypoint;

typedef struct {

		bool ReadParameter;
		bool StartMission;
		bool StopMission;
		bool BackHome;
    bool TranferWayPoint;
}_cmd;

typedef struct {

		uint8_t isGetWayPoint;
		uint8_t StartMission;
		uint8_t StopMission;
		uint8_t BackHome;
    
}_echo;


typedef struct {
    uint16_t currentwaypoint;
	  float voltage1;
		float voltage2;
		float voltage3;
		float voltage4;
	  
	  uint16_t leftfront_p;
	  uint16_t leftfront_n;
	  uint16_t leftback_p;
	  uint16_t leftback_n;
	
	  uint16_t rightfront_p;
	  uint16_t rightfront_n;
	  uint16_t rightback_p;
	  uint16_t rightback_n;
	  
	  float dis2wp;
	  float detaP;
	  float gyroZ;

	  bool isDebug;
	
	
}_status;



typedef  struct {
	
	
	uint32_t LED_Count;
	uint32_t HL_Count;
	uint32_t DIO_Count;
	uint32_t VOLTAGE_Count;
	uint32_t PWM_Count;
	uint32_t ENGINE_Count;
  
	uint32_t STATUS_Count;
	uint32_t GPS_Count; 
	
	uint32_t SendWayPointCount;
	uint32_t ReadWayPointCount;
	uint32_t MaxWayPointCount;

	
	
	
	
	_heardbeat HeardBeat;
	_parameter Parameter;
	_gps GPS;
	
	_gps SIM;//simulation point
	
	_ultrasonic Ultrasonic;
	_waypoint WayPoint;
	_status Satuts;
	_cmd CMD;
	_echo Echo;
	
	_uart U1;
	_uart U2;
	_uart U3;
	_uart U4;
	_uart U5;
	_uart U6;
	
}_hal_io;


extern _hal_io HAL_IO;




/*====================================================================
 *Transmit Function From Here
 *@HM
 *
 *====================================================================
 */
void Protocol_DataCombin(uint8_t *pData,uint16_t Size) ;

void Protocol_Transmit(float T);


//Protocol Transmit


/*====================================================================
 *Receive Function From Here
 *@HM
 *
 *====================================================================
 */			

extern _waypoint WayPointList[500];
																 

void Protocol_T_HL(uint32_t T);
void Protocol_T_VOLTAGE(uint32_t T);
void Protocol_T_DI(uint32_t T);
void Protocol_T_ENGINE(uint32_t T);

bool getchar_uart2(uint8_t* c);
void Protocol_Rev(void);
void Protocol_R_Prepare(uint8_t c);

void Protocol_R_LED(uint8_t *data);
void Protocol_R_CMD(uint8_t *data);
void Protocol_R_Parameter(uint8_t *data);
void Protocol_R_WayPoint(uint8_t *data);
void Protocol_R_SimuPos(uint8_t *data);


void Protocol_T_GPS(uint32_t T);
void Protocol_T_Echo(uint32_t ID,uint32_t Value);//0x04
void Protocol_T_Parameter(void);//0x10;
void Protocol_T_WayPoint(void);//0x40
void Protocol_T_Status(uint32_t T);//0x50
#endif










