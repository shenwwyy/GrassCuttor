#include "linepoint.h"
#include "control.h"
#include "mymath.h"
#include "math.h"


//通过一条线上的而两个点，计算出来过这两个点的直线的斜率和常数项
//y=kx+b，y=lat，x=lon
//y1 = kx1 + b
//y2 = kx2 + b
//k = (y1 - y2)/(x1 - x2)
//b = y1 - (y1x1 - y2x1)/(x1 -x2) = (y2x1 - y1x2)/(x1 -x2)
void LinePoint_KB(double lat1,double lon1,double lat2,double lon2,double *k,double *b)
{
	if((lon1 == lon2)&&(lat1 == lat2))
	{
		 *k = 0;
		 *b = 0;
	}
	else if((lon1 == lon2)&&(lat1 != lat2))
	{
		 *k = 100000000;//一亿表示无穷大
		 *b = lon1;
	}
	else if((lon1 != lon2)&&(lat1 == lat2))
	{
		 *k = 0;
		 *b = lat1;
	}
	else
	{
	   *k = (lat2 - lat1)/(lon2 - lon1);
		 *b =(lat2 * lon1 - lat1 * lon2)/(lon1 -lon2);
	}
}


/*
  通过距离偏差计算出想要的经纬度

  R = 6371000m
  C = 2 * 6371000m  * Pi = 40030173
  某一个纬度下的长度
  Cx = 40030173 * cosx = Nm

  任意经度的长度
  Cy = 40030173m

  因此，
  某一个纬度下，东西方向的1m长短对应的经度大小为 dLon = 360/Cx;
  任意经度下，南北方向的1m长短对应的纬度大小为   dLat = 360/Cy;
*/
void LinePoint_LatLon(double Lat1,double Lon1,double dL,double course,double *dLat,double *dLon)
{
	double dX = dL * sin(course * 0.0174533f);
	double dY = dL * cos(course * 0.0174533f);
	 
	double R = 6371000;
  double C = 2 * R * 3.4159265358f;

  double Cx = C * cos(Lat1 * 0.0174533f);
  double Cy = C;

  double Lon = 360/Cx;//单位经度
  double Lat = 360/Cy;//单位纬度
	
	*dLat = Lat * dY;
	*dLon = Lon * dX;
}


//计算一个点到一条直线的距离
//坐标 点在线的左边的出距离为负，在右为正
double LinePoint_Distance(double lat,double lon,double K,double B)
{
	  double dL;
	  if(K == 0)
		{
			dL = B - lat;
		}
		else if(K >= 100000000)
		{
			dL = lon - B;
		}
		else
		{
	     dL = -(K*lon + lat - B)/my_sqrt(K*K + 1);
		}
	  return dL;
}

//求解两条直线的交点坐标
//y = k1x + b1
//y = k2x + b2
//x = (b2 - b1)/(k1 - k2)
//y = (k2b1 - k1b2)/(k2 - k1) 
void LinePoint_LineLine(double K1,double B1,
	                      double K2,double B2,
											  double *x,double *y)
{
    double dx,dy;
	
	  if(K1 == K2)
		{
			dx = 0;
			dy = 0;
		}
		else//斜率不一样的线都会有交点
		{
	    dx = (B2-B1)/(K1-K2);
			dy = (K2*B1 - K1*B2)/(K2-K1);
		}
	  *x = dx;
		*y = dy;																																																																																																
}




























