
#include "control.h"
#include "math.h"
#include <stdio.h>
#include <stdbool.h>
#include "motor.h"
#include "mymath.h"

_controlDef Control;



//ִ������
/*
������ʽ��Ϊ����
0����������
1���������
2���������
3) ��������
 */

//���������
void Control_TaskManage(float T,uint32_t id)
{
	    //���µ�ǰ��λ����Ϣ
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
	
	float *UNUSED;
	
	//С���ƶ�ɲ�������С������������ôȡ���ƶ�����ʡ���������Ҵ�������⣬��ֹ��������
	//ʱ�̵ȴ��Ÿ������һֱ�ȴ�����ң�ض˷����������ʱ�л��������
	//����յ�������������ж��Ƿ��е磬�Ƿ��ܹ����й��������������ô����������Ȼ��С�����Ͻ����ݵ������л����������
	
	//����ٶȣ�����ٶ�С��0.1m/s����ôȡ��ɲ��
	if(Control.Senser.GPS.speed <= 0.1f)
	{
		  //MOTOR_BRAKE(UNUSED);//��ɲ��
	}
	
	//����ѹ����ֹ��ѹ���ͣ���ѹ������Сֵʱ���л����������
	if(Control.Senser.Voltage.Battery1.Battery <= Control.Senser.Voltage.Battery1.Min)
	{
		Control.Task.Task_id = ChargingTask;//�л����������
	}
	
	//�ȴ������л��������������ѭ����ɣ����ѭ��һֱ��鴮�ڽ���buff���ͽ������
	
	//����һ����Ҫ�л���ȥ����ô��Ҫ��⵱ǰ��ѹ�Ƿ��ܹ�֧�����������֧�֣��ܾ�ִ���в�����
	if(Control.Command.WannaTask == WorkingTask)
	{
		 if(Control.Senser.Voltage.Battery1.Battery <= (Control.Senser.Voltage.Battery1.Max - Control.Senser.Voltage.Battery1.Min)*0.3f)
		 {
				Control.Command.WannaTask = -1;//����в���������
			  Control.Task.Task_id = ChargingTask;//�л����������
			  Control.Task.Charging.ChargeStatus = uncharge;//�ڳ���·��
		 }
	}
	
	
	//������
	Control.Task.PositionOutPut = 0;
	Control.Task.HeadingOutPut  = 0;
	
	
}

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
				   //���ֵ�ع���������Ҫ�ж�����ȥ���
				   Control.Task.Working.BatCount = 0;
				   Control.Task.Working.isInterrupt = 1;//�ж�����ȥ���
				   Control.Task.Task_id = ChargingTask;//�л����������
			     Control.Task.Charging.ChargeStatus = uncharge;//�ڳ���·��
				 
				   //���ֵ�ǰ�㣬��Ϊ��һ�ν����Ŀ���
				   Control.Task.InterruptPoint = Control.Task.CurrentPoint;
				   //�л�Ŀ��㣬Ŀ���Ϊ���׮
				   Control.Task.TargetPoint = Control.Task.ChargePoint;
			 }
				 
		   
	}
	else//ִ����������
	{  
		  //�����Ƿ�Ӧ���л�����
		  if(Control_PolygonCheck(Control.Task.CurrentPoint,Control.Task.PointGroups,Control.Task.PointGroupsNumber,1) == true)
			{
				   //�л��µĺ�����ΪĿ��
				   //���ֵ�ǰ����Ϊ��һ����
				   Control.Task.LastPoint   = Control.Task.CurrentPoint;
				   //��Ŀ��㸳ֵ
				   Control.Task.TargetPoint = Control.Task.TargetPoint;
			}
/*		
		  //��ǰλ�úͲο��ߵľ���
		  //���㾭γ�ȣ�ÿ����һ�������0.0000002�ȼ�20cm���
		
			if(Control.Task.firstTimeIntoCircle == 0x01)
			{
				 Control.Task.firstTimeIntoCircle = 0x00;
				
				 //��һ��Ŀ��㾭γ�ȱ仯��Ȼ��ֵ��Ŀ��㡣
			}
		*/	
			
		  //���㵱ǰλ�úͲο������ƫ��
		  Control_Route(T,Control.Task.CurrentPoint,Control.Task.TargetPoint,Control.Senser.Sonar,0);
	}
}

void Control_ChargingTask(float T)
{
	//С��ִ�г�����񣬳��������ɣ��ص��ո��жϵ�λ�ã�����ִ�и������
	//�����������Ѱ�ҳ��λ�ã�һ·�����ߵ����λ�ø�������ʼѰ�ҳ���豸��Ȼ��׼ȷ���ϣ���ʼ��硣
	//���������ɣ�����Ƿ��д���δ��ɵĸ����������У���ôȥ���ո������ϵ�λ�ã�Ѱ�Һ�Ŀ�꺽���л������������и��
	//���������δ���������ôͣ���ڳ��׮�ڲ�����Ȼ���л�����������
	
	
	switch(Control.Task.Charging.ChargeStatus)
	{
		case uncharge:
		{
			  //Ѱ�ҳ��׮
			 //����Ŀ���
       Control.Task.TargetPoint = Control.Task.ChargePoint;
			 //���߿���
			 Control_Route(T,Control.Task.CurrentPoint,Control.Task.TargetPoint,Control.Senser.Sonar,0);
			
			//�жϵ�ǰ�������ľ���ڲ�1m֮�ڣ���ô��Ϊ�������
			if(Control_CircleCheck(Control.Task.CurrentPoint,Control.Task.TargetPoint,1.0f,0) == true)
			{
				   //�л������ģʽ
				   Control.Task.Charging.ChargeStatus = charging;
					 Control.Task.Charging.ChargeCount = 0;
			}
			
			
			
		}break;
		
		case charging:
		{
			 //����ڼ䣬�������κ�����ָ��
			 if(Control.Senser.Voltage.Battery1.Battery >= Control.Senser.Voltage.Battery1.Max)
			 {
				 //������ڵ����ˣ���ô��һ��ʱ�䣬��ȷ�Ǵ��ˣ���ô��Ϊ�����ɡ�
				 Control.Task.Charging.ChargeCount += T;
				 if(Control.Task.Charging.ChargeCount > 300)//ʱ�����300��
				 {
					 Control.Task.Charging.ChargeStatus = charged;
					 Control.Task.Charging.ChargeCount = 0;
				 }
			 }
			 else//���û�н�������ѹ
			 {
				 Control.Task.Charging.ChargeCount = 0;
			 }
			 //������
			 Control.Task.PositionOutPut = 0;
			 Control.Task.HeadingOutPut  = 0;
			 
		}break;

		case charged :
		{
			 if(Control.Command.WannaTask == WorkingTask)//�����������ô�л������У���ִ��
			 {
				 Control.Task.Task_id = WorkingTask;
				 
				 if(Control.Task.Working.isInterrupt == 0x01)//��������񱻴�ϣ���ô�������ǰ������
				 {
					   Control.Task.TargetPoint = Control.Task.InterruptPoint;
					   Control.Task.Working.isInterrupt = 0x00;//����жϱ�־
				 }
				 else//����������񱻴�ϣ���ô�л����µ�����
				 {
					   Control.Task.TargetPoint = Control.Task.PointGroups[1];//�л�����һ��
					   Control.Task.Working.isInterrupt = 0x00;//����жϱ�־
				 }
				 
			 }
			 else//�����л�����������
			 {
				 Control.Task.Task_id = IdleTask;
			 }
			 
			 //������
			 Control.Task.PositionOutPut = 0;
			 Control.Task.HeadingOutPut  = 0;
			 
		}break;

	}
	
	
}

void Control_BackHomeTask(float T)
{
	//�յ��������С��Ŀ���Ϊ���׮λ�ã�����һ·���ֱ��ϣ��ص����׮������Ȼ��Ѱ�ҳ���豸����ȷ���Ϻ󣬽�����׮����ɺ��л�����������
	//�������С�����Խ��������������ȥһ���µĵط����и������ֹͣɲ����
	//
	
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
	float mLat = lat2 - lat1;
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f);
	temp = 90.0f + atan2(-mLat, mLon) * 57.2957795f;

	if(temp < 0)temp += 360.0f;
	return temp;
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
void Control_Route(float T,_point Current,_point Target,_sonar Sonar,float PosOffset)
{
	   float Kp = 0;
	   float PositionErr,HeadingErr;
	   float Current_Target_Heading;
	   float CrossErr;
	
	   uint8_t SonarFlag = 0;
	
	   //���㵱ǰ���Ŀ���ľ��룬�ó�ǰ������
	
	   PositionErr = POS_Distance(Current.latitude,Current.longitude,Target.latitude,Target.longitude);
	   
	   //����ǰ�򣬲�����ϰ����ϵ�ǰ�ĺ����ǣ��ó�����������
	   //0~360
	   Current_Target_Heading = POS_Heading(Current.latitude,Current.longitude,Target.latitude,Target.longitude);
	
	   HeadingErr = Current_Target_Heading - Current.course;
	   HeadingErr = To_180_degrees(HeadingErr);
	
	   //�����ƫ
	   
	
	
	   //ͨ��������ϰ����ж��Ƿ�Ҫ���Ӻ�������
	   
	   if((Sonar.left.isValid == 0x01)&&(Sonar.left.distance <= 0.2f)) SonarFlag |= 0x04;//0000 0100
		 else                                                            SonarFlag &= 0xfb;//1111 1011
		 
		 if((Sonar.forward.isValid == 0x01)&&(Sonar.forward.distance <= 0.2f)) SonarFlag |= 0x02;//0000 0010
		 else                                                                  SonarFlag &= 0xfd;//1111 1101
		 
		 if((Sonar.right.isValid == 0x01)&&(Sonar.right.distance <= 0.2f)) SonarFlag |= 0x01;//0000 0001
		 else                                                              SonarFlag &= 0xfe;//1111 1110
		 
		 SonarFlag &= 0x07;//0000 0111
	
	   switch(SonarFlag)
		 {
			 case 0://û���ϰ���
			 {
				   HeadingErr += 0;
			 }break;
			 case 1://�Ҳ����ϰ���
			 {
				   //ƫ��
				   HeadingErr += (-10.0f) * T;
				   
			 }break;
			 case 2://ǰ�����ϰ���
			 {
				   //ֹͣ��ת��
				   //�Ѿ���Ŀ��ֵ���㣬�õ��ֹͣ
				   PositionErr = 0;
				   //ƫ��
				   HeadingErr += (-45.0f) * T;
				 
			 }break;
			 case 3://�Ҳ��ǰ�����ϰ���
			 {
				   //ֹͣ��ת��
				   //�Ѿ���Ŀ��ֵ���㣬�õ��ֹͣ
				   PositionErr = 0;
				   //ƫ��
				   HeadingErr += (-45.0f) * T;
				 
			 }break;
			 case 4://������ϰ���
			 {
				   //ƫ��
				   HeadingErr += (-10.0f) * T;
			 }break;
			 case 5://�����������ϰ���
			 {
				   //����ֱ�У����ҵ�������Ĳ�ƫһ��
				   HeadingErr += (Sonar.right.distance - Sonar.left.distance) * 10.0f * T;
			 }break;
			 case 6://����ǰ�����ϰ���
			 {
				   //ֹͣ��ת��
				   //�Ѿ���Ŀ��ֵ���㣬�õ��ֹͣ
				   PositionErr = 0;
				   //ƫ��
				   HeadingErr += (45.0f) * T;
			 }break;
			 case 7://�����������ϰ���
			 {
				   //ֹͣ����ͷ
				   PositionErr = 0;
				   //ƫ��
				   HeadingErr += (-180.0f) * T;
			 }break;
			
		 }
		 
		 
		 Control.Task.PositionOutPut = LIMIT(10.0f * PositionErr - 2.0f * Current.speed,-1000,1000);
		 Control.Task.HeadingOutPut  = LIMIT(10.0f * HeadingErr + 10.0f * PosOffset,-500,500);
}






