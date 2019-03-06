#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f4xx_hal.h"
#include "stdbool.h"

#define MaxPoint 200






extern enum {
	IdleTask = 0,
	WorkingTask,
	ChargingTask,
	BackHomeTask
	
}_task_id;



extern enum {
	
	Level_00_10 = 0,
	Level_10_20,
	Level_20_30,
	Level_30_40,
	Level_40_50,
	Level_50_60,
	Level_60_70,
	Level_70_80,
	Level_80_90,
	Level_90_100,
}_charge_level;

extern enum {
	uncharge = 0,
	charging,
	charged
}_charge_status;


#define TX_BUFF_SIZE  512
#define RX_BUFF_SIZE  512
typedef struct {
	
	 uint8_t rbuff[10];
	
	 uint8_t txbuff[TX_BUFF_SIZE];
	 uint8_t rxbuff[RX_BUFF_SIZE];
	
	 uint16_t r_tail;
	 uint16_t r_head;
	
	 uint16_t t_head;
	
}_hlink;



typedef struct {
	
	float Width;
	float Length;
	float Height;
	
	float Diameter;//直径
	
	
	_hlink HLink;
	
	
	float BaisThrottle;
	uint8_t isunLock;
	
	
	uint8_t WheelTest;
	
	
}_car;





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
	
	uint8_t isGPSUpdate;
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
	
	uint8_t rbuff[10];
	
  uint8_t txbuff[TX_BUFF_SIZE];
	uint8_t rxbuff[RX_BUFF_SIZE];
	
	uint16_t r_tail;
	uint16_t r_head;
	
	uint16_t t_head;
	
}_gps_;


typedef struct {
	
	uint32_t TimeCount;
	
	uint8_t isValid;
	float   distance;
	
	uint8_t isUpdated;
	uint8_t rxbuff[2];
	uint8_t txbuff[1];
	
	
}_sonar_t;


typedef struct {
	_sonar_t left;
	_sonar_t forward;
	_sonar_t right;
}_sonar;


typedef struct {
	float   Max;
	float   Min;
	float   Battery;
}_voltage_t;

typedef struct {
	uint8_t    ID;
	_voltage_t Battery1;
	_voltage_t Battery2;
	_voltage_t Battery3;
	_voltage_t Battery4;
}_voltage;

typedef struct {//deg
	float ax;
	float zy;
	float az;
	
	float gx;
	float gy;
	float gz;
	
	float roll;
	float pitch;
	float yaw;
}_imu;


typedef struct {
	uint16_t year;
	uint8_t  month;
	uint8_t  day;
	uint8_t  hour;
	uint8_t  minute;
	uint8_t  second;
	uint8_t  valid;
}_rtc;


typedef struct {
	_rtc    gRTC;
	_gps_   GPS;
	_sonar Sonar;
	_voltage Voltage;
	_imu IMU;
}_senser;


typedef struct {
	
   uint8_t isSaveParameter;
	
	
	
}_parameter_;



typedef struct {
	
	int8_t WannaTask;//期望的任务ID
	
	uint8_t EmergencyStop;
	
}_command;


typedef struct {
	
	uint8_t tt;
	
}_idletask;

typedef struct {
	
	uint8_t isInterrupt;
	
	float BatCount;

}_workingtask;

typedef struct {
	
	
	uint8_t ChargeStatus;
	uint8_t ChargeLevel;
	float   ChargeCount;
	
	
}_chargingtask;




typedef struct {
	uint32_t Task_id;
	_point   ChargePoint;
	_point   LastPoint;
	_point   CurrentPoint;
	_point   TargetPoint;
	_point   InterruptPoint;
	
	
	_point   ReferencePoint1;
	_point   ReferencePoint2;
	_point   PointGroups[MaxPoint];
	_point   PointTaskGroups[MaxPoint];
	uint32_t PointGroupsNumber;
	
	_idletask Idle;
	_workingtask Working;
	_chargingtask Charging;
	
	uint8_t FirstTimeIntoRoute;
	
	float PositiveMaxDistance;
	float NegativeMaxDistance;
	
	
	float Diff_Heading;
	float Diff_Position;
	
	float PositionOutPut;
	float HeadingOutPut;
	float SpeedOutPut;
	
	float Speed_Kp;
	float Speed_Ki;
	float Speed_Out;
	float Speed_Err;
	float Speed_i;
	float Speed_d;
	
	
	
	float Heading_Kp;
	float Heading_Ki;
	float Heading_Out;
	float Heading_Err;
	float Heading_i;
	float Heading_d;
	
	float Position_Kp;
	float Position_Ki;
	float Position_Out;
	float Position_Err;
	float Position_i;
	
	//输出
	float LeftForeWheel;
	float LeftBackWheel;
	float RightForeWheel;
	float RightBackWheel;
	
	uint8_t firstTimeIntoCircle;//首次进入圈内
	bool  isCompeletTask;
	
}_taskDef;



typedef struct {
	
	_car Car;
	_taskDef Task;
	_senser Senser;
	_parameter_ Parameter;
	_command Command;
}_controlDef;


extern _controlDef Control;




void Control_TaskManage(float T,uint32_t id);

void Control_IdleTask(float T);
void Control_WorkingTask(float T);
void Control_ChargingTask(float T);
void Control_BackHomeTask(float T);



typedef struct {
	
	float (* Distance)(void *);
	float (* Heading)(void *);
	void (* LatLon)(void *);
}_pos;

extern _pos POS;

float POS_Distance(float lat1,float lon1,float lat2,float lon2);
float POS_Heading(float lat1,float lon1,float lat2,float lon2);
void  POS_LatLon(double lat1,double lon1,double dLat,double dLon,double *lat2,double *lon2);



uint8_t Control_PolygonCheck(_point Current,_point Target[],uint16_t TargetNumber,uint8_t Side);
uint8_t Control_CircleCheck(_point Current,_point Centre,float Radius,uint8_t Side);
uint8_t Control_OverCheck(_point Last,_point Current,_point Target,float Distance);

void Control_Route(float T,_point Last,_point Current,_point Target,_sonar Sonar,float PosOffset);


#endif


