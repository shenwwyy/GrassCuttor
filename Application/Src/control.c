
#include "control.h"



_controlDef Control;



//执行任务
/*
任务形式分为多种
0）待机任务
1）割草任务
2）充电任务
 */

//任务管理级别
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

//任务级别
void Control_IdleTask(float T)
{
	//小车制动刹车，如果小车不滑动，那么取消制动，节省电量，并且带能量检测，防止电量过低
	
	
	
}

void Control_WorkingTask(float T)
{
	//小车根据割草的任务进行工作
}

void Control_ChargingTask(float T)
{
	//小车执行充电任务，充电任务完成，回到刚刚中断的位置，继续执行割草任务
	//触发充电任务，寻找充电位置，一路避障走到充电位置附近，开始寻找充电设备，然后准确对上，开始充电。
	//充电任务完成，检测是否有存在未完成的割草任务，如果有，那么去到刚刚任务打断的位置，寻找好目标航向，切换至割草任务进行割草
	//如果不存在未完成任务，那么停留在充电桩内不动，然后切换至空闲任务。
	
	
}


//底层控制级别

//地理围栏检测函数
//多边形检测
void Control_PolygonCheck(_point Point)
{
	
}
//环形检测
void Control_CircleCheck(_point Point)
{
	
}

//割草任务
void Control_CutGrass()
{
	
}




