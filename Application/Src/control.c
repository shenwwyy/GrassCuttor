
#include "control.h"
#include "math.h"
#include <stdio.h>
#include <stdbool.h>
#include "motor.h"
#include "mymath.h"
#include "LinePoint.h"
#include "protocol.h"
#include "gy952.h"

_controlDef Control;




_linear Line_1_2;
_linear Line_1_n;
_linear Line_2_3;



//ִ������
/*
������ʽ��Ϊ����
0����������
1���������
2���������
3����������
 */

//���������
void Control_TaskManage(float T,uint32_t id)
{
	    //���µ�ǰ��λ����Ϣ
	
	    HAL_IO.GPS.fixtype     = Control.Senser.GPS.fix;
	    HAL_IO.GPS.svn         = Control.Senser.GPS.svn;
	    HAL_IO.GPS.altitude    = Control.Senser.GPS.altitude;
	    HAL_IO.GPS.latitude    = Control.Senser.GPS.latitude;
	    HAL_IO.GPS.longitude   = Control.Senser.GPS.longitude;
	    HAL_IO.GPS.course      = Control.Senser.GPS.course;
	    HAL_IO.GPS.groundspeed = Control.Senser.GPS.speed;
	
	
	
	
	
	
	    Control.Task.CurrentPoint.altitude  = Control.Senser.GPS.altitude;
	    Control.Task.CurrentPoint.latitude  = Control.Senser.GPS.latitude;
	    Control.Task.CurrentPoint.longitude = Control.Senser.GPS.longitude;
	    Control.Task.CurrentPoint.course    = Control.Senser.GPS.course;
	    Control.Task.CurrentPoint.speed     = Control.Senser.GPS.speed;
	
	    //����ʼ
	    switch(id)
			{
				case IdleTask:
				{
					Control_IdleTask(T);
				}break;
				case WorkingTask:
				{
					Control_WorkingTask(T);
				}break;
				case ChargingTask:
				{
					Control_ChargingTask(T);
				}break;
				case BackHomeTask:
				{
					Control_BackHomeTask(T);
				}break;
				
				
				
			}
}

//���񼶱�
void Control_IdleTask(float T)
{

	if(Control.Senser.GPS.speed <= 0.1f)
	{
		  //MOTOR_BRAKE(UNUSED);//��ɲ��
	}
	
	if(Control.Senser.Voltage.Battery1.Battery <= Control.Senser.Voltage.Battery1.Min)
	{
		Control.Task.Task_id = ChargingTask;
	}
	
	if(Control.Command.WannaTask == WorkingTask)
	{
		 if(Control.Senser.Voltage.Battery1.Battery <= ((Control.Senser.Voltage.Battery1.Max - Control.Senser.Voltage.Battery1.Min)*0.3f + Control.Senser.Voltage.Battery1.Min))
		 {
				Control.Command.WannaTask = -1;
			  Control.Task.Task_id = ChargingTask;
			  Control.Task.Charging.ChargeStatus = uncharge;
		 }
		 else
		 {
			  Control.Command.WannaTask = -1;
			  Control.Task.Task_id = WorkingTask;
		 }
	}
	
	
	//������
	Control.Car.isunLock = 0x00;
	Control.Task.PositionOutPut = 0;
	Control.Task.HeadingOutPut  = 0;
	
	
}

double k_1_2,b_1_2;
double k_1_n,b_1_n;
double k_2_3,b_2_3;
double dm = 0.5f;//�趨mΪ������֮��Ĳ�ֵ

double dlat,dlon,dg;

double heading_e,heading_1_2,heading_2_3;

double x1,y1;
double x2,y2;

double distance_1_n,distance_2_3;

uint16_t PointCount = 0;

           

          
void Control_WorkingTask(float T)
{
	//С�����ݸ�ݵ�������й���
  //С����ʱ���ϣ����Ҽ�⵱ǰ�����Ƿ��ڵ���Χ��֮�ڣ�������ڣ���ô����Ҫ�������Χ�����ٽ��й���
	//���С��ǰ�������ϰ��С�����ƹ��ϰ�����и��
	//С����ݹ����У���ʱ�����������Ƿ�ﵽ������ޣ���������ô�ж����񣬲��Ҽ�¼��ǰ��λ�úͺ���Ȼ���л����������
	//���������ɣ�С��������������
	

	if(Control.Senser.Voltage.Battery1.Battery <= Control.Senser.Voltage.Battery1.Min)
	{
		   Control.Task.Working.BatCount += T;
		   if(Control.Task.Working.BatCount >= 60)//1����
			 {
				   Control.Task.Working.BatCount = 0;
				   Control.Task.Working.isInterrupt = 1;
				   Control.Task.Task_id = ChargingTask;
			     Control.Task.Charging.ChargeStatus = uncharge;
				 
				   Control.Task.InterruptPoint = Control.Task.CurrentPoint;
				   Control.Task.TargetPoint = Control.Task.ChargePoint;
			 }
				 
		   
	}
	else
	{  

			

			if(Control_CircleCheck(Control.Task.CurrentPoint,Control.Task.TargetPoint,3.0f,0) == true)
			{
				    Control.Task.LastPoint.Number    = Control.Task.TargetPoint.Number;
						Control.Task.LastPoint.altitude  = Control.Task.TargetPoint.altitude;
						Control.Task.LastPoint.latitude  = Control.Task.TargetPoint.latitude;
						Control.Task.LastPoint.longitude = Control.Task.TargetPoint.longitude;
						Control.Task.LastPoint.speed     = Control.Task.TargetPoint.speed;
						Control.Task.LastPoint.course    = Control.Task.TargetPoint.course;
				
				  uint32_t NextNumber = 0;
				
				  if((Control.Task.TargetPoint.Number+1) >= HAL_IO.MaxWayPointCount)
					{
						NextNumber = 0;
						
						Control.Task.TargetPoint.Number    = Control.Task.ChargePoint.Number;
						Control.Task.TargetPoint.altitude  = Control.Task.ChargePoint.altitude;
						Control.Task.TargetPoint.latitude  = Control.Task.ChargePoint.latitude;
						Control.Task.TargetPoint.longitude = Control.Task.ChargePoint.longitude;
						Control.Task.TargetPoint.speed     = Control.Task.ChargePoint.speed;
						Control.Task.TargetPoint.course    = Control.Task.ChargePoint.course;
						
					}
					else
					{
						NextNumber = Control.Task.TargetPoint.Number+1;
						
						Control.Task.TargetPoint.Number    = WayPointList[NextNumber].id;
						Control.Task.TargetPoint.altitude  = WayPointList[NextNumber].altitude;
						Control.Task.TargetPoint.latitude  = WayPointList[NextNumber].latitude;
						Control.Task.TargetPoint.longitude = WayPointList[NextNumber].longitude;
						Control.Task.TargetPoint.speed     = WayPointList[NextNumber].speed;
						Control.Task.TargetPoint.course    = WayPointList[NextNumber].course;
						
					}

			}
			
		  Control_Route(T,
			              Control.Task.LastPoint,
			              Control.Task.CurrentPoint,
			              Control.Task.TargetPoint,
			              Control.Senser.Sonar,0);
			
			
	}
}

void Control_ChargingTask(float T)
{	
	switch(Control.Task.Charging.ChargeStatus)
	{
		case uncharge:
		{
       Control.Task.TargetPoint = Control.Task.ChargePoint;

			 Control.Car.isunLock = 0x57;
			 Control_Route(T,Control.Task.LastPoint,Control.Task.CurrentPoint,Control.Task.TargetPoint,Control.Senser.Sonar,0);
			
			if(Control_CircleCheck(Control.Task.CurrentPoint,Control.Task.TargetPoint,1.0f,0) == true)
			{
				   Control.Task.Charging.ChargeStatus = charging;
					 Control.Task.Charging.ChargeCount = 0;
			}
			
			
			
		}break;
		
		case charging:
		{
			 if(Control.Senser.Voltage.Battery1.Battery >= Control.Senser.Voltage.Battery1.Max)
			 {
				 Control.Task.Charging.ChargeCount += T;
				 if(Control.Task.Charging.ChargeCount > 300)//ʱ�����300��
				 {
					 Control.Task.Charging.ChargeStatus = charged;
					 Control.Task.Charging.ChargeCount = 0;
				 }
			 }
			 else
			 {
				 Control.Task.Charging.ChargeCount = 0;
			 }
			 Control.Car.isunLock = 0x00;
			 Control.Task.PositionOutPut = 0;
			 Control.Task.HeadingOutPut  = 0;
			 
		}break;

		case charged :
		{
			 if(Control.Command.WannaTask == WorkingTask)
			 {
				 Control.Task.Task_id = WorkingTask;
				 
				 if(Control.Task.Working.isInterrupt == 0x01)
				 {
					   Control.Task.TargetPoint = Control.Task.InterruptPoint;
					   Control.Task.Working.isInterrupt = 0x00;
				 }
				 else
				 {
					   Control.Task.TargetPoint.Number = WayPointList[0].id;
					   Control.Task.Working.isInterrupt = 0x00;
				 }
				 
			 }
			 else//�����л�����������
			 {
				 Control.Task.Task_id = IdleTask;
			 }
			 
			 //������
			 Control.Car.isunLock = 0x00;
			 Control.Task.PositionOutPut = 0;
			 Control.Task.HeadingOutPut  = 0;
			 
		}break;

	}
	
	
}

void Control_BackHomeTask(float T)
{
	
	Control.Task.Task_id = ChargingTask;
	Control.Task.Charging.ChargeStatus = uncharge;
	
}



//�ײ���Ƽ���

//����Χ����⺯��
//Side ��ʾ���⣬Side = 0 ��ʾ�ڲ���Side = 1��ʾ�ⲿ
//Current��ʾ��ǰ������λ�ã�Target/Centre��ʾĿ���
//����ֵisSide

//����μ��
uint8_t Control_PolygonCheck(_point Current,_point Target[],uint16_t TargetNumber,uint8_t Side)
{
	 uint8_t isSide = false;
   uint16_t iSum = 0,iCount;  
	
    double dLon1, dLon2, dLat1, dLat2, dLon;  
    if (TargetNumber < 3) return false;  
    iCount = TargetNumber;  
    for(uint16_t i = 0; i < iCount; i++) 
	  {  
        if (i == iCount - 1) 
				{  
            dLon1 = Target[i].longitude;  
            dLat1 = Target[i].latitude;  
            dLon2 = Target[0].longitude;  
            dLat2 = Target[0].latitude;  
        } 
				else 
				{  
            dLon1 = Target[i].longitude;  
            dLat1 = Target[i].latitude;  
            dLon2 = Target[i + 1].longitude;  
            dLat2 = Target[i + 1].latitude;  
        }  
        //��������ж�A���Ƿ��ڱߵ����˵��ˮƽƽ����֮�䣬��������н��㣬��ʼ�жϽ����Ƿ�����������  
        if (((Current.latitude >= dLat1) && (Current.latitude < dLat2)) || ((Current.latitude >= dLat2) && (Current.latitude < dLat1))) 
				{
            if(fabs(dLat1 - dLat2) > 0)//����0������ǵ�պ������� 
						{  
                //�õ� A������������ߵĽ����x���꣺  
                dLon = dLon1 - ((dLon1 - dLon2) * (dLat1 - Current.latitude)) / (dLat1 - dLat2);  
                if (dLon < Current.longitude)  
                    iSum++;  
            }  
        }  
    }  
		
		
    if(iSum % 2 != 0) 
		{			
			  if(Side == 0)//�ж��ǲ������ڲ�
				{
          isSide =  true;  
				} 
		}
		else
		{
			  if(Side == 1)//�ж��ǲ������ⲿ
				{
          isSide = true;
				}					
		}			
	 return isSide;	
}
//���μ��
uint8_t Control_CircleCheck(_point Current,_point Centre,float Radius,uint8_t Side)
{
	 uint8_t isSide = false;//Ĭ��Ϊ��
	
	 float Current_Centre_distance;
	
	 Current_Centre_distance = POS_Distance(Current.latitude,Current.longitude,Centre.latitude,Centre.longitude);
	
	 if(Side == 0)//��⵱ǰ����Ȧ���ڲ�
	 {
		  if(Current_Centre_distance <= Radius) 
			{
				 isSide = true;
			}
	 }
	 else//��⵱ǰ����Ȧ��
	 {
		  if(Current_Centre_distance >= Radius) 
			{
				 isSide = true;
			}
	 }
	 return isSide;	 
}

//ͨ����γ�ȼ������ͺ���
/*
����������ľ��롣
lat1 lon1  ��1�ľ�γ��  ��λ��
lat2 lon2  ��2�ľ�γ�� 
���ؼ�������ľ���   ��λ ��
*/
float POS_Distance(float lat1,float lon1,float lat2,float lon2){
  float temp;
	float mLat = (lat2 - lat1)*110946.0f;
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f)*111318.0f ;
	temp = 	sqrt(mLat*mLat + mLon*mLon); 	//γ��1�� = ��Լ111km = 111319.5��
	return temp;
}

/*
��������������ߵ� ����ǣ� ������Ϊ0 ��
lat1 lon1  ��1�ľ�γ��  ��λ��
lat2 lon2  ��2�ľ�γ�� 
���� �ĺ���ǣ���λ�ȡ�
��1��ָ��2�� 0-360
*/
float POS_Heading(float lat1,float lon1,float lat2,float lon2){
	float temp;
	if((lat1 == lat2)&&(lon1 == lon2))
	{
		temp = 0.0f;
	}
	else
	{
		float mLat = lat2 - lat1;
		float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f);
		temp = 90.0f + atan2(-mLat, mLon) * 57.2957795f;
	}

	if(temp < 0)temp += 360.0f;
	return temp;
}

/*
���㾭γ��,ͨ��xy�ľ���ֱ������ڶ���ľ�γ��
dLatΪγ�ȷ���ĳ��ȵ�λ�ף�����Ϊ��
dLonΪ���ȷ���ĳ��ȵ�λ�ף�����Ϊ��
*/
void POS_LatLon(double lat1,double lon1,double dLat,double dLon,double *lat2,double *lon2){
	
	*lat2 = lat1 + dLat/110946.0f;
	*lon2 = lon1 + dLon/(cos(((*lat2 + lat1)/2)* 0.0174532925f)*111318.0f);//γ��1�� = ��Լ111km = 111319.5��
	
}





//�����б�ʺ�ƫ��y=kx+b
void KB_Rate(_point P1,_point P2,float *k,float *b)
{
	  //�����ĺ���,x1!=x2,y1!=y2
	  if((P1.latitude != P2.latitude)&&(P1.longitude != P2.longitude))
		{
			  *k = (P2.latitude - P1.latitude)/(P2.longitude - P1.longitude);
			  *b = P1.latitude - (*k) * P1.longitude;
		}
		else if((P1.latitude != P2.latitude)&&(P1.longitude == P2.longitude))//б��Ϊ�����,x1=x2,y1!=y2
		{
			  *k = 1e20;
			  *b = 0;
		}
		else if((P1.latitude == P2.latitude)&&(P1.longitude != P2.longitude))//б��Ϊ0,x1!=x2,y1==y2
		{
			  *k = 0;
			  *b = P1.latitude;
		}
}

//ͨ�������ľ���ƫ�����Ŀ��ο���y=kx+b
//RP1,RP2ΪĿ������һ���㣬
void KB_Line(_point P1,_point P2,float Krate,float Brate,float Offset,_point *Target,_point *Next)
{
	  //ͨ���ο�1,2�㣬�Լ���ƫ���ó�Ŀ�꾭γ��
	  if((P1.latitude != P2.latitude)&&(P1.longitude != P2.longitude))
		{
			  Krate = (P2.latitude - P1.latitude)/(P2.longitude - P1.longitude);
			  Brate = P1.latitude - (Krate) * P1.longitude;
		}
		else if((P1.latitude != P2.latitude)&&(P1.longitude == P2.longitude))//б��Ϊ�����,x1=x2,y1!=y2
		{
			  Krate = 1e20;
			  Brate = 0;
		}
		else if((P1.latitude == P2.latitude)&&(P1.longitude != P2.longitude))//б��Ϊ0,x1!=x2,y1==y2
		{
			  Krate = 0;
			  Brate = P1.latitude;
		}
	
}


//���߿���

	   float Kp = 0;
	   float PositionErr,HeadingErr;
	   float Current_Target_Heading;
	   float Last_Target_Heading;
	   float use_Heading;
	
	   float CrossDistance;
	   float CrossErr;
 uint8_t SonarFlag = 0;

void Control_Route(float T,_point Last,_point Current,_point Target,_sonar Sonar,float PosOffset)
{
		 //�ٶȿ���
		 Control.Task.Speed_Err = LIMIT(Target.speed - Current.speed,-10,10) * HAL_IO.Parameter.speed_Kp;
		 Control.Task.Speed_i  += Control.Task.Speed_Err * HAL_IO.Parameter.speed_Ki * T;
		 Control.Task.Speed_i   = LIMIT(Control.Task.Speed_i,-70,70);
		 Control.Task.Speed_Out = Control.Task.Speed_Err + Control.Task.Speed_i;

		 //���߿���
		 
	    
		 //�����ƫ
	   PositionErr = POS_Distance(Current.latitude,Current.longitude,
	                              Target.latitude,Target.longitude);
	   
	   //����ǰ�򣬲�����ϰ����ϵ�ǰ�ĺ����ǣ��ó�����������
	   Current_Target_Heading = POS_Heading(Current.latitude,Current.longitude,
	                                        Target.latitude,Target.longitude);

	   //�����ƫ
	   //��ǰ��Ŀ�꣬��Ƕȣ����룬�������ƫ������Ŀ��ֵ����Ļ�����ô�ĳ����Ĳ�ƫ��Զ����ͬһ������
	   Last_Target_Heading = POS_Heading(Last.latitude,Last.longitude,
	                                     Target.latitude,Target.longitude);
	
	   use_Heading = Last_Target_Heading - Current_Target_Heading;
		 //ת����180�ȸ�ʽ3
		 use_Heading = To_180_degrees(use_Heading);
	   //�����ƫ��
	   CrossDistance = PositionErr * sin(use_Heading * 0.017453278f);//ת�ɻ�����	 

		 Control.Task.Position_Err = LIMIT(CrossDistance,-10,10) * HAL_IO.Parameter.distance_Kp;
		 Control.Task.Position_i  += Control.Task.Position_Err * HAL_IO.Parameter.distance_Ki * T;
		 Control.Task.Position_i   = LIMIT(Control.Task.Position_i,-10,10);
		 Control.Task.Position_Out = Control.Task.Position_Err + Control.Task.Position_i;//����ǲ�ƫ

		 //����ƫ��
		 if((Current.latitude != Target.latitude)||(Current.longitude != Target.longitude))//�����γ��һ������ô�����
		 {
		    HeadingErr = Current.course -  POS_Heading(Current.latitude,Current.longitude,Target.latitude,Target.longitude);
		 }
		 else
		 {
			  HeadingErr = 0;
		 }
		 //ת����180�ȸ�ʽ
		 HeadingErr = LIMIT(To_180_degrees(HeadingErr),-20,20);
		 
		 Control.Task.Heading_i    += HAL_IO.Parameter.heading_Ki * HeadingErr * T ;//�����ƫ����
		 Control.Task.Heading_i    = LIMIT(Control.Task.Heading_i,-10,10);
		 
		 Control.Task.Heading_d    = GY925.GYRO.DEG.gz * HAL_IO.Parameter.heading_Kd;
		 
		 Control.Task.Heading_Out  = HAL_IO.Parameter.heading_Kp * HeadingErr + Control.Task.Heading_i + Control.Task.Heading_d;//�����ƫ����
		 
		 
		 
		 HAL_IO.Satuts.detaP = CrossDistance;
		 HAL_IO.Satuts.dis2wp = PositionErr * cos(use_Heading * 0.017453278f);
		 HAL_IO.Satuts.gyroZ  = GY925.GYRO.DEG.gz;
	
		 Control.Task.PositionOutPut = LIMIT(Control.Task.Position_Out,-300,300);
		 Control.Task.HeadingOutPut  = LIMIT(Control.Task.Heading_Out,-300,300);
		 Control.Task.SpeedOutPut    = LIMIT(Control.Task.Speed_Out,0,900);
}





