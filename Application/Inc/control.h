#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f4xx_hal.h"




typedef struct {
	
	uint32_t Number;
	double   latitude;
	double   longitude;
	double   altitude;
	float speed;
	float course;
	float vn;
	float ve;
	float vd;
}_point;



typedef struct {
	
	uint8_t fix;
	uint8_t svn;
	
	double latitude;
	double longitude;
	double altitude;
	
	float speed;
	float course;
	float vn;
	float ve;
	float vd;
}_gps;


typedef struct {
	uint8_t isValid;
	float   distance;
}_sonar_t;


typedef struct {
	_sonar_t left;
	_sonar_t forward;
	_sonar_t right;
}_sonar;



typedef struct {
	
	_gps   GPS;
	_sonar Sonar;
	
}_senser;





typedef struct {
	
	uint32_t Task_id;
	_point   CurrentPoint;
	
	
	
	_senser Senser;
	
}_controlDef;


extern _controlDef Control;




void Control_TaskManage(float T,uint32_t id);

void Control_IdleTask(float T);
void Control_WorkingTask(float T);
void Control_ChargingTask(float T);









#endif


