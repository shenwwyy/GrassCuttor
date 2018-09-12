#include "linepoint.h"
#include "control.h"
#include "mymath.h"
#include "math.h"


//ͨ��һ�����ϵĶ������㣬������������������ֱ�ߵ�б�ʺͳ�����
//y=kx+b��y=lat��x=lon
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
		 *k = 100000000;//һ�ڱ�ʾ�����
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
  ͨ������ƫ��������Ҫ�ľ�γ��

  R = 6371000m
  C = 2 * 6371000m  * Pi = 40030173
  ĳһ��γ���µĳ���
  Cx = 40030173 * cosx = Nm

  ���⾭�ȵĳ���
  Cy = 40030173m

  ��ˣ�
  ĳһ��γ���£����������1m���̶�Ӧ�ľ��ȴ�СΪ dLon = 360/Cx;
  ���⾭���£��ϱ������1m���̶�Ӧ��γ�ȴ�СΪ   dLat = 360/Cy;
*/
void LinePoint_LatLon(double Lat1,double Lon1,double dL,double course,double *dLat,double *dLon)
{
	double dX = dL * sin(course * 0.0174533f);
	double dY = dL * cos(course * 0.0174533f);
	 
	double R = 6371000;
  double C = 2 * R * 3.4159265358f;

  double Cx = C * cos(Lat1 * 0.0174533f);
  double Cy = C;

  double Lon = 360/Cx;//��λ����
  double Lat = 360/Cy;//��λγ��
	
	*dLat = Lat * dY;
	*dLon = Lon * dX;
}


//����һ���㵽һ��ֱ�ߵľ���
//���� �����ߵ���ߵĳ�����Ϊ��������Ϊ��
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

//�������ֱ�ߵĽ�������
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
		else//б�ʲ�һ�����߶����н���
		{
	    dx = (B2-B1)/(K1-K2);
			dy = (K2*B1 - K1*B2)/(K2-K1);
		}
	  *x = dx;
		*y = dy;																																																																																																
}




























