
#include "control.h"



_controlDef Control;



//ִ������
/*
������ʽ��Ϊ����
0����������
1���������
2���������
 */

//���������
void Control_TaskManage(float T,uint32_t id)
{
	    switch(id)
			{
				case 0:
				{
					Control_IdleTask(T);
				}break;
				case 1:
				{
					Control_WorkingTask(T);
				}break;
				case 2:
				{
					Control_ChargingTask(T);
				}break;
			}
}

//���񼶱�
void Control_IdleTask(float T)
{
	//С���ƶ�ɲ�������С������������ôȡ���ƶ�����ʡ���������Ҵ�������⣬��ֹ��������
	
	
	
}

void Control_WorkingTask(float T)
{
	//С�����ݸ�ݵ�������й���
}

void Control_ChargingTask(float T)
{
	//С��ִ�г�����񣬳��������ɣ��ص��ո��жϵ�λ�ã�����ִ�и������
	//�����������Ѱ�ҳ��λ�ã�һ·�����ߵ����λ�ø�������ʼѰ�ҳ���豸��Ȼ��׼ȷ���ϣ���ʼ��硣
	//���������ɣ�����Ƿ��д���δ��ɵĸ����������У���ôȥ���ո������ϵ�λ�ã�Ѱ�Һ�Ŀ�꺽���л������������и��
	//���������δ���������ôͣ���ڳ��׮�ڲ�����Ȼ���л�����������
	
	
}


//�ײ���Ƽ���

//����Χ����⺯��
//����μ��
void Control_PolygonCheck(_point Point)
{
	
}
//���μ��
void Control_CircleCheck(_point Point)
{
	
}

//�������
void Control_CutGrass()
{
	
}




