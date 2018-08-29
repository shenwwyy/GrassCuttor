#ifndef _LINEPOINT_H_
#define _LINEPOINT_H_


#include "stm32f4xx_hal.h"



typedef struct {
	
	double k;
	double b;
}_linear;

extern _linear Line_1_2;
extern _linear Line_1_n;
extern _linear Line_2_3;






void LinePoint_KB(double lat1,double lon1,double lat2,double lon2,double *k,double *b);
void LinePoint_LatLon(double Lat1,double Lon1,double dL,double course,double *dLat,double *dLon);
double LinePoint_Distance(double lat,double lon,double K,double B);

void LinePoint_LineLine(double K1,double B1,
	                      double K2,double B2,
											  double *x,double *y);





#endif
