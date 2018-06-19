#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f4xx_hal.h"

#define MaxPoint 200


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
	
	
	
	float out_x;//前向
	float out_y;//侧向
	float out_z;//旋转
	
}_parameter;


typedef struct {
		uint32_t Task_id;
	_point   CurrentPoint;
	_point   TargetPoint;
	_point   PointGroups[MaxPoint];
	
	
	
	
}_taskDef;



typedef struct {
	
	_taskDef Task;
	_senser Senser;
	_parameter Parameter;
	
}_controlDef;


extern _controlDef Control;




void Control_TaskManage(float T,uint32_t id);

void Control_IdleTask(float T);
void Control_WorkingTask(float T);
void Control_ChargingTask(float T);
void Control_BackHomeTask(float T);


float POS_Distance(float lat1,float lon1,float lat2,float lon2);
float POS_Heading(float lat1,float lon1,float lat2,float lon2);

uint8_t Control_PolygonCheck(_point Current,_point Target[],uint16_t TargetNumber,uint8_t Side);
uint8_t Control_CircleCheck(_point Current,_point Centre,float Radius,uint8_t Side);


#endif


