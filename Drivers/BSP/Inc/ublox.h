#ifndef __UBLOX_H
#define __UBLOX_H	 

#include "stm32f4xx_hal.h"
#include "main.h"

//UBX Frame Structure:
//B5 62 + CLASS(1byte) + ID(1byte) + LEN(2byte) + PAYLODE + CK_A CK_B(2byte)
#define UBLOX_SYNC_CHAR1		0xB5
#define UBLOX_SYNC_CHAR2		0x62

#define UBLOX_NAV_CLASS	    0x01
#define UBLOX_RXM_CLASS	    0x02
#define UBLOX_CFG_CLASS	    0x06
#define UBLOX_MON_CLASS	    0x0a
#define UBLOX_AID_CLASS	    0x0b
#define UBLOX_TIM_CLASS	    0x0d

#define UBLOX_CFG_CFG	    	0x09
#define UBLOX_CFG_PRT       0x00
#define UBLOX_CFG_MSG	    	0x01
#define UBLOX_CFG_RATE	    0x08


#define UBLOX_NAV_POSLLH    0x02
#define UBLOX_NAV_VELNED    0x12
#define UBLOX_NAV_STATUS    0x03

#define UBLOX_MAX_PAYLOAD   512
#define UBLOX_WAIT_MS	    	20

#define UBLOX_PortNum_DDC 	0x01 
#define UBLOX_PortNum_UART1 0x01
#define UBLOX_PortNum_USB  	0x03
#define UBLOX_PortNum_SPI		0x04

#define UBLOX_PROTOCOL_UBX 				0x0001 
#define UBLOX_PROTOCOL_NMEA 			0x0010
#define UBLOX_PROTOCOL_RTCM2  		0x0100
#define UUBLOX_PROTOCOL_RTCM3			0X1000

enum ubloxStates {
    UBLOX_WAIT_SYNC1 = 0,
    UBLOX_WAIT_SYNC2,
    UBLOX_WAIT_CLASS,
    UBLOX_WAIT_ID,
    UBLOX_WAIT_LEN1,
    UBLOX_WAIT_LEN2,
    UBLOX_PAYLOAD,
    UBLOX_CHECK1,
    UBLOX_CHECK2
};

// Geodetic Position Solution
typedef struct {
    unsigned long iTOW;	   	 // GPS Millisecond Time of Week (ms)
    signed long lon;	   		 // Longitude (deg * 1e-7)
    signed long lat;	  		 // Latitude (deg * 1e-7)
    signed long height;	 	   // Height above Ellipsoid (mm)
    signed long hMSL;	  	   // Height above mean sea level (mm)
    unsigned long hAcc;	     // Horizontal Accuracy Estimate (mm)
    unsigned long vAcc;	     // Vertical Accuracy Estimate (mm)
} _ubloxStructPOSLLH_t;


//function
 void ubloxSet_Rate(unsigned short int ms);
 void ubloxSet_PRT(uint32_t baud,uint16_t inProtoMask,uint16_t outProtoMask);
 void ubloxmsg_Enable(uint8_t msgClass,uint8_t msgID);
 void ubloxInit(void);
 void Protocol_NAV_VELNED(uint8_t *data);
 void Protocol_NAV_STATUS(uint8_t *data);
 void Protocol_NAV_POSLLH(uint8_t *data);
 uint16_t CheckSum(uint8_t *buff,uint16_t len);
 void ublox_Prepare(uint8_t data);

void ublox_Rev(void);
// void ublox_Protocol_Send(uint8_t *data,uint8_t Len);
#endif

