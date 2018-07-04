
#include "control.h"
#include "math.h"
#include <stdio.h>
#include <stdbool.h>
#include "motor.h"

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
	
	if(Control.Senser.GPS.speed <= 0.1f)
	{
		  MOTOR_BRAKE(UNUSED);//ɲ��
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
		 if(Control.Senser.Voltage.Battery1.Battery <= (Control.Senser.Voltage.Battery1.Max - Control.Senser.Voltage.Battery1.Min)*0.5f)
		 {
				Control.Command.WannaTask = -1;//����в���������
			  Control.Task.Task_id = ChargingTask;//�л����������
			  Control.Task.Charging.ChargeStatus = uncharge;//�ڳ���·��
		 }
	}
	
	
}

void Control_WorkingTask(float T)
{
	//С�����ݸ�ݵ�������й���
  //С����ʱ���ϣ����Ҽ�⵱ǰ�����Ƿ��ڵ���Χ��֮�ڣ�������ڣ���ô����Ҫ�������Χ�����ٽ��й���
	//���С��ǰ�������ϰ��С�����ƹ��ϰ�����и��
	//С����ݹ����У���ʱ�����������Ƿ�ﵽ������ޣ���������ô�ж����񣬲��Ҽ�¼��ǰ��λ�úͺ���Ȼ���л����������
	//���������ɣ�С��������������
	
	
	
	
	
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
		}break;

		case charged :
		{
			 if(Control.Command.WannaTask == WorkingTask)//�����������ô�л������У���ִ��
			 {
				 Control.Task.Task_id = WorkingTask;
				 
				 if(Control.Task.Working.isInterrupt == 0x01)//��������񱻴�ϣ���ô�������ǰ������
				 {
					 
				 }
				 else//����������񱻴�ϣ���ô�л����µ�����
				 {
					 
				 }
				 
			 }
			 else//�����л�����������
			 {
				 Control.Task.Task_id = IdleTask;
			 }
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
    for (uint16_t i = 0; i < iCount; i++) 
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








//���߿���
void Control_Route(void)
{
	
}






