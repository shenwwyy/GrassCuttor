
#include "control.h"
#include "math.h"
#include <stdio.h>
#include <stdbool.h>
#include "motor.h"
#include "mymath.h"
#include "LinePoint.h"
#include "protocol.h"


_controlDef Control;




_linear Line_1_2;
_linear Line_1_n;
_linear Line_2_3;



//执行任务
/*
任务形式分为多种
0）待机任务
1）割草任务
2）充电任务
3）返航任务
 */

//任务管理级别
void Control_TaskManage(float T,uint32_t id)
{
	    //更新当前的位置信息
//	    Control.Task.CurrentPoint.altitude  = Control.Senser.GPS.altitude;
//	    Control.Task.CurrentPoint.latitude  = Control.Senser.GPS.latitude;
//	    Control.Task.CurrentPoint.longitude = Control.Senser.GPS.longitude;
//	    Control.Task.CurrentPoint.course    = Control.Senser.GPS.course;
//	    Control.Task.CurrentPoint.speed     = Control.Senser.GPS.speed;
	
	    
	    //任务开始
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

//任务级别
void Control_IdleTask(float T)
{
	
	
	//小车制动刹车，如果小车不滑动，那么取消制动，节省电量，并且带能量检测，防止电量过低
	//时刻等待着割草任务，一直等待接受遥控端发来的命令，随时切换割草任务，
	//如果收到割草任务，首先判断是否有电，是否能够进行工作，如果不够那么发出警报，然后小车避障进入割草的区域，切换至割草任务
	
	//检查速度，如果速度小于0.1m/s，那么取消刹车
	if(Control.Senser.GPS.speed <= 0.1f)
	{
		  //MOTOR_BRAKE(UNUSED);//不刹车
	}
	
	//检测电压，防止电压过低，电压低于最小值时，切换至充电任务
	if(Control.Senser.Voltage.Battery1.Battery <= Control.Senser.Voltage.Battery1.Min)
	{
		Control.Task.Task_id = ChargingTask;//切换至充电任务
	}
	
	//等待任务切换的命令在另外的循环完成，这个循环一直检查串口接收buff，和解码程序
	//任务一旦想要切换过去，那么就要检测当前电压是否能够支持任务如果不支持，拒绝执行切草任务
	if(Control.Command.WannaTask == WorkingTask)
	{
		 if(Control.Senser.Voltage.Battery1.Battery <= ((Control.Senser.Voltage.Battery1.Max - Control.Senser.Voltage.Battery1.Min)*0.3f + Control.Senser.Voltage.Battery1.Min))
		 {
				Control.Command.WannaTask = -1;//清除切草任务命令
			  Control.Task.Task_id = ChargingTask;//切换至充电任务
			  Control.Task.Charging.ChargeStatus = uncharge;//在充电的路上
		 }
		 else
		 {
			  Control.Command.WannaTask = -1;//清除切草任务命令
			  Control.Task.Task_id = WorkingTask;//切换至工作任务
		 }
	}
	
	
	//清空输出
	Control.Car.isunLock = 0x00;
	Control.Task.PositionOutPut = 0;
	Control.Task.HeadingOutPut  = 0;
	
	
}

				   double k_1_2,b_1_2;
					 double k_1_n,b_1_n;
					 double k_2_3,b_2_3;
				   double dm = 0.5f;//设定m为两条线之间的差值
				
				   double dlat,dlon,dg;

           double heading_e,heading_1_2,heading_2_3;

           double x1,y1;
					 double x2,y2;

           double distance_1_n,distance_2_3;
    
           uint16_t PointCount = 0;

           

          
void Control_WorkingTask(float T)
{
	//小车根据割草的任务进行工作
  //小车随时避障，并且检测当前区域是否在地里围栏之内，如果不在，那么首先要进入地里围栏，再进行工作
	//如果小车前方遇到障碍物，小车会绕过障碍物进行割草
	//小车割草过程中，随时检查自身电量是否达到最低门限，如果到达，那么中断任务，并且记录当前的位置和航向，然后切换至充电任务
	//割草任务完成，小车启动返航任务
	

	if(Control.Senser.Voltage.Battery1.Battery <= Control.Senser.Voltage.Battery1.Min)
	{
		   Control.Task.Working.BatCount += T;
		   if(Control.Task.Working.BatCount >= 60)//1分钟
			 {
				   //出现电池过低现象，需要中断任务去充电
				   Control.Task.Working.BatCount = 0;
				   Control.Task.Working.isInterrupt = 1;//中断任务，去充电
				   Control.Task.Task_id = ChargingTask;//切换至充电任务
			     Control.Task.Charging.ChargeStatus = uncharge;//在充电的路上
				 
				   //保持当前点，作为下一次进入的目标点
				   Control.Task.InterruptPoint = Control.Task.CurrentPoint;
				   //切换目标点，目标点为充电桩
				   Control.Task.TargetPoint = Control.Task.ChargePoint;
			 }
				 
		   
	}
	else//执行正常任务
	{  
		  
		  
		  //检查是否在已经到达航线内，如果没有到达，那么执行进入航线的程序
		  if((Control_CircleCheck(Control.Task.CurrentPoint,Control.Task.PointGroups[1],1.0f,0) == true)&&(Control.Task.FirstTimeIntoRoute == 0x01))
			{
				Control.Task.FirstTimeIntoRoute = 0x00;
				
				//首次进入，那么计算正向负向最大值
				double Kr_1_2 = 0,Br_1_2 = 0;
				LinePoint_KB(Control.Task.PointGroups[1].latitude,Control.Task.PointGroups[1].longitude,
				             Control.Task.PointGroups[2].latitude,Control.Task.PointGroups[2].longitude,
				             &Kr_1_2,&Br_1_2);
				
				float MaxDistance = 0;
				for(int i = 3;i <= Control.Task.PointGroupsNumber;i++)
				{
					 MaxDistance = LinePoint_Distance(Control.Task.PointGroups[i].latitude,Control.Task.PointGroups[i].longitude,Kr_1_2,Br_1_2);
					 Control.Task.PositiveMaxDistance = (Control.Task.PositiveMaxDistance >= MaxDistance)?(Control.Task.PositiveMaxDistance):(MaxDistance);
				   Control.Task.NegativeMaxDistance = (Control.Task.NegativeMaxDistance <= MaxDistance)?(Control.Task.NegativeMaxDistance):(MaxDistance);
				}
				
				
				
			}
		
		  //否则执行航线里面的程序
		
			//检查是否到达目标点
			if(Control_CircleCheck(Control.Task.CurrentPoint,Control.Task.TargetPoint,1.0f,0) == true)
			{
				   //计算参考点1,2，的斜率和偏差值
//				   double k_1_2,b_1_2;
//					 double k_1_n,b_1_n;
//					 double k_2_3,b_2_3;
//				   double dm = 0.5f;//设定m为两条线之间的差值
//				
//				   double dlat,dlon,dg;
				   //斜率计算错误,需要修正，第一次进入区域的时候需要判断目标点是什么
		       LinePoint_KB(Control.Task.PointGroups[1].latitude,Control.Task.PointGroups[1].longitude,
				                Control.Task.PointGroups[2].latitude,Control.Task.PointGroups[2].longitude,
				                &k_1_2,&b_1_2);		  
  
				   Line_1_2.k = k_1_2;
				   Line_1_2.b = b_1_2;//应该计算出当前直线的斜率，在当前
				
				   //求出差值为dm，对应的经纬度偏差
//				   double heading_e,heading_1_2,heading_2_3;
				   
				   heading_1_2 = POS_Heading(Control.Task.PointGroups[1].latitude,Control.Task.PointGroups[1].longitude,
				                             Control.Task.PointGroups[2].latitude,Control.Task.PointGroups[2].longitude);
				   heading_2_3 = POS_Heading(Control.Task.PointGroups[2].latitude,Control.Task.PointGroups[2].longitude,
				                             Control.Task.PointGroups[3].latitude,Control.Task.PointGroups[3].longitude);
				   
					 heading_e = heading_2_3 - heading_1_2;
					 
					 //换成-180~180模式
					 heading_e = To_180_degrees(heading_e);
					 //取一条垂直于1-2的线
				   if(heading_e > 0) heading_e = heading_1_2 + 90;
					 else              heading_e = heading_1_2 - 90;
				
//				   //恢复到360度模式
//				   if(heading_e>360)    heading_e = heading_e - 360;
//					 else if(heading_e<0) heading_e = heading_e + 360;
//					 else                 heading_e = heading_e;
						 
				   //计算出下一条直线 x,y方向需要偏移多少值
				   LinePoint_LatLon(Control.Task.PointGroups[1].latitude,Control.Task.PointGroups[1].longitude,
				                    dm,heading_e,&dlat,&dlon);
					 
					 //给偏差增加0.5m					
					 dm += 0.5f;
														
				   dg = my_sqrt(dlat * dlat + dlon *dlon);//计算出角度长度
				   //求出平移后的直线(往上为加)
				   //y=kx+b + dg*sqrt(k*k +1)
		       //b_1_2 应该计算出当前直线的B值
					 Line_1_2.b = b_1_2 + dlat - Line_1_2.k * dlon ;
					 
           //计算，1n，23线段
				
				   LinePoint_KB(Control.Task.PointGroups[1].latitude,Control.Task.PointGroups[1].longitude,
				                Control.Task.PointGroups[Control.Task.PointGroupsNumber].latitude,Control.Task.PointGroups[Control.Task.PointGroupsNumber].longitude,
				                &k_1_n,&b_1_n);	
				   Line_1_n.k = k_1_n;
				   Line_1_n.b = b_1_n;
				
				
				   LinePoint_KB(Control.Task.PointGroups[2].latitude,Control.Task.PointGroups[2].longitude,
				                Control.Task.PointGroups[3].latitude,Control.Task.PointGroups[3].longitude,
				                &k_2_3,&b_2_3);	
												
					 Line_2_3.k = k_2_3;
				   Line_2_3.b = b_2_3;
				
				   //计算两个目标点
//					 double x1,y1;
//					 double x2,y2;
					 LinePoint_LineLine(Line_1_2.k,Line_1_2.b,Line_1_n.k,Line_1_n.b,&x1,&y1);
					 LinePoint_LineLine(Line_1_2.k,Line_1_2.b,Line_2_3.k,Line_2_3.b,&x2,&y2);
					 //计算当前点和两个点的距离，哪个比较近，哪个就是上一点，远的视为目标点。
//					 double distance_1_n,distance_2_3;
					 
					 distance_1_n = POS_Distance(Control.Task.CurrentPoint.latitude,Control.Task.CurrentPoint.longitude,x1,y1);
					 distance_2_3 = POS_Distance(Control.Task.CurrentPoint.latitude,Control.Task.CurrentPoint.longitude,x2,y2);
					 
					 if(distance_1_n > distance_2_3)
					 {
						  Control.Task.TargetPoint.latitude = y1;
						  Control.Task.TargetPoint.longitude = x1;
						  Control.Task.TargetPoint.speed = Control.Task.PointGroups[1].speed;
						 
						  Control.Task.LastPoint.latitude = y2;
						  Control.Task.LastPoint.longitude = x2;
					 }
					 else
					 {
						  Control.Task.TargetPoint.latitude = y2;
						  Control.Task.TargetPoint.longitude = x2;
						  Control.Task.TargetPoint.speed = Control.Task.PointGroups[1].speed;
						 
						  Control.Task.LastPoint.latitude = y1;
						  Control.Task.LastPoint.longitude = x1;
					 }
					 
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+1][0] = 1;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+1][1] = 20;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+1][2] = Control.Task.LastPoint.longitude;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+1][3] = Control.Task.LastPoint.latitude;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+1][4] = 0;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+1][5] = 0;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+1][6] = 0;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+1][7] = 0;
					 
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+2][0] = 1;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+2][1] = 20;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+2][2] = Control.Task.TargetPoint.longitude;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+2][3] = Control.Task.TargetPoint.latitude;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+2][4] = 0;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+2][5] = 0;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+2][6] = 0;
					 Protocol_RouteDot[0][Control.Task.PointGroupsNumber+PointCount+2][7] = 0;
					 
					 PointCount += 2;
					 Protocol_RouteDot[0][1][1] = 20;
					 Protocol_RouteDot[0][2][1] = 20;
					 Protocol_RouteDot[0][3][1] = 20;
					 Protocol_RouteDot[0][4][1] = 20;
					 
					 
			}
			
		  //计算当前位置和参考方向的偏差
			Control.Car.isunLock = 0x57;
		  Control_Route(T,Control.Task.LastPoint,Control.Task.CurrentPoint,Control.Task.TargetPoint,Control.Senser.Sonar,0);
			
			
			//检查是否完成切割任务
			
			
			
	}
}

void Control_ChargingTask(float T)
{
	//小车执行充电任务，充电任务完成，回到刚刚中断的位置，继续执行割草任务
	//触发充电任务，寻找充电位置，一路避障走到充电位置附近，开始寻找充电设备，然后准确对上，开始充电。
	//充电任务完成，检测是否有存在未完成的割草任务，如果有，那么去到刚刚任务打断的位置，寻找好目标航向，切换至割草任务进行割草
	//如果不存在未完成任务，那么停留在充电桩内不动，然后切换至空闲任务。
	
	
	switch(Control.Task.Charging.ChargeStatus)
	{
		case uncharge:
		{
			  //寻找充电桩
			 //设置目标点
       Control.Task.TargetPoint = Control.Task.ChargePoint;
			 //航线控制
			 Control.Car.isunLock = 0x57;
			 Control_Route(T,Control.Task.LastPoint,Control.Task.CurrentPoint,Control.Task.TargetPoint,Control.Senser.Sonar,0);
			
			//判断当前点如果在木点内部1m之内，那么认为到达充电点
			if(Control_CircleCheck(Control.Task.CurrentPoint,Control.Task.TargetPoint,1.0f,0) == true)
			{
				   //切换到充电模式
				   Control.Task.Charging.ChargeStatus = charging;
					 Control.Task.Charging.ChargeCount = 0;
			}
			
			
			
		}break;
		
		case charging:
		{
			 //充电期间，不接受任何任务指令
			 if(Control.Senser.Voltage.Battery1.Battery >= Control.Senser.Voltage.Battery1.Max)
			 {
				 //如果大于等于了，那么等一段时间，的确是大了，那么认为充电完成。
				 Control.Task.Charging.ChargeCount += T;
				 if(Control.Task.Charging.ChargeCount > 300)//时间大于300秒
				 {
					 Control.Task.Charging.ChargeStatus = charged;
					 Control.Task.Charging.ChargeCount = 0;
				 }
			 }
			 else//如果没有进入最大电压
			 {
				 Control.Task.Charging.ChargeCount = 0;
			 }
			 //清空输出
			 Control.Car.isunLock = 0x00;
			 Control.Task.PositionOutPut = 0;
			 Control.Task.HeadingOutPut  = 0;
			 
		}break;

		case charged :
		{
			 if(Control.Command.WannaTask == WorkingTask)//如果有任务，那么切换到其中，并执行
			 {
				 Control.Task.Task_id = WorkingTask;
				 
				 if(Control.Task.Working.isInterrupt == 0x01)//如果是任务被打断，那么继续充电前的任务
				 {
					   Control.Task.TargetPoint = Control.Task.InterruptPoint;
					   Control.Task.Working.isInterrupt = 0x00;//清空中断标志
				 }
				 else//如果不是任务被打断，那么切换到新的任务
				 {
					   Control.Task.TargetPoint = Control.Task.PointGroups[1];//切换到第一点
					   Control.Task.Working.isInterrupt = 0x00;//清空中断标志
				 }
				 
			 }
			 else//否则切换到空闲任务
			 {
				 Control.Task.Task_id = IdleTask;
			 }
			 
			 //清空输出
			 Control.Car.isunLock = 0x00;
			 Control.Task.PositionOutPut = 0;
			 Control.Task.HeadingOutPut  = 0;
			 
		}break;

	}
	
	
}

void Control_BackHomeTask(float T)
{
	//收到返航命令，小车目标改为充电桩位置，并且一路保持避障，回到充电桩附近，然后寻找充电设备，精确对上后，进入充电桩，完成后切换到空闲任务
	//这个过程小车可以接受任务命令，例如去一块新的地方进行割草任务，停止刹车等
	
	Control.Task.Task_id = ChargingTask;
	Control.Task.Charging.ChargeStatus = uncharge;
	
}



//底层控制级别

//地理围栏检测函数
//Side 表示内外，Side = 0 表示内部，Side = 1表示外部
//Current表示当前的坐标位置，Target/Centre表示目标点
//返回值isSide

//多边形检测
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
        //以下语句判断A点是否在边的两端点的水平平行线之间，在则可能有交点，开始判断交点是否在左射线上  
        if (((Current.latitude >= dLat1) && (Current.latitude < dLat2)) || ((Current.latitude >= dLat2) && (Current.latitude < dLat1))) 
				{
            if(fabs(dLat1 - dLat2) > 0)//等于0的情况是点刚好在线上 
						{  
                //得到 A点向左射线与边的交点的x坐标：  
                dLon = dLon1 - ((dLon1 - dLon2) * (dLat1 - Current.latitude)) / (dLat1 - dLat2);  
                if (dLon < Current.longitude)  
                    iSum++;  
            }  
        }  
    }  
		
		
    if(iSum % 2 != 0) 
		{			
			  if(Side == 0)//判断是不是在内部
				{
          isSide =  true;  
				} 
		}
		else
		{
			  if(Side == 1)//判断是不是在外部
				{
          isSide = true;
				}					
		}			
	 return isSide;	
}
//环形检测
uint8_t Control_CircleCheck(_point Current,_point Centre,float Radius,uint8_t Side)
{
	 uint8_t isSide = false;//默认为假
	
	 float Current_Centre_distance;
	
	 Current_Centre_distance = POS_Distance(Current.latitude,Current.longitude,Centre.latitude,Centre.longitude);
	
	 if(Side == 0)//检测当前点在圈的内部
	 {
		  if(Current_Centre_distance <= Radius) 
			{
				 isSide = true;
			}
	 }
	 else//检测当前点在圈外
	 {
		  if(Current_Centre_distance >= Radius) 
			{
				 isSide = true;
			}
	 }
	 return isSide;	 
}

//通过经纬度计算距离和航向
/*
计算两个点的距离。
lat1 lon1  点1的经纬度  单位度
lat2 lon2  点2的经纬度 
返回计算出来的距离   单位 米
*/
float POS_Distance(float lat1,float lon1,float lat2,float lon2){
  float temp;
	float mLat = (lat2 - lat1)*110946.0f;
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f)*111318.0f ;
	temp = 	sqrt(mLat*mLat + mLon*mLon); 	//纬度1度 = 大约111km = 111319.5米
	return temp;
}

/*
计算两个点的连线的 航向角， 以正北为0 。
lat1 lon1  点1的经纬度  单位度
lat2 lon2  点2的经纬度 
返回 的航向角，单位度。
由1点指向2点 0-360
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
计算经纬度,通过xy的距离分别计算出第二点的经纬度
dLat为纬度方向的长度单位米，朝北为正
dLon为经度方向的长度单位米，朝东为正
*/
void POS_LatLon(double lat1,double lon1,double dLat,double dLon,double *lat2,double *lon2){
	
	*lat2 = lat1 + dLat/110946.0f;
	*lon2 = lon1 + dLon/(cos(((*lat2 + lat1)/2)* 0.0174532925f)*111318.0f);//纬度1度 = 大约111km = 111319.5米
	
}





//计算出斜率和偏差y=kx+b
void KB_Rate(_point P1,_point P2,float *k,float *b)
{
	  //正常的函数,x1!=x2,y1!=y2
	  if((P1.latitude != P2.latitude)&&(P1.longitude != P2.longitude))
		{
			  *k = (P2.latitude - P1.latitude)/(P2.longitude - P1.longitude);
			  *b = P1.latitude - (*k) * P1.longitude;
		}
		else if((P1.latitude != P2.latitude)&&(P1.longitude == P2.longitude))//斜率为无穷大,x1=x2,y1!=y2
		{
			  *k = 1e20;
			  *b = 0;
		}
		else if((P1.latitude == P2.latitude)&&(P1.longitude != P2.longitude))//斜率为0,x1!=x2,y1==y2
		{
			  *k = 0;
			  *b = P1.latitude;
		}
}

//通过给定的距离偏差计算目标参考线y=kx+b
//RP1,RP2为目标点和下一个点，
void KB_Line(_point P1,_point P2,float Krate,float Brate,float Offset,_point *Target,_point *Next)
{
	  //通过参考1,2点，以及侧偏，得出目标经纬度
	  if((P1.latitude != P2.latitude)&&(P1.longitude != P2.longitude))
		{
			  Krate = (P2.latitude - P1.latitude)/(P2.longitude - P1.longitude);
			  Brate = P1.latitude - (Krate) * P1.longitude;
		}
		else if((P1.latitude != P2.latitude)&&(P1.longitude == P2.longitude))//斜率为无穷大,x1=x2,y1!=y2
		{
			  Krate = 1e20;
			  Brate = 0;
		}
		else if((P1.latitude == P2.latitude)&&(P1.longitude != P2.longitude))//斜率为0,x1!=x2,y1==y2
		{
			  Krate = 0;
			  Brate = P1.latitude;
		}
	
}


//行走控制

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
		 //速度控制
		 Control.Task.Speed_Err = LIMIT(Target.speed - Current.speed,-10,10) * Control.Task.Speed_Kp;
		 Control.Task.Speed_i  += Control.Task.Speed_Err * Control.Task.Speed_Ki * T;
		 Control.Task.Speed_i   = LIMIT(Control.Task.Speed_i,-20,70);
		 Control.Task.Speed_Out = Control.Task.Speed_Err + Control.Task.Speed_i;

		 //航线控制
		 
	   /* 
		 //计算侧偏
	   PositionErr = POS_Distance(Current.latitude,Current.longitude,Target.latitude,Target.longitude);
	   
	   //计算前向，侧向的障碍物，结合当前的航迹角，得出侧向控制输出
	   //-180~180
	   Current_Target_Heading = To_180_degrees(POS_Heading(Current.latitude,Current.longitude,Target.latitude,Target.longitude));

	   //计算侧偏
	   //当前到目标，额角度，距离，计算出侧偏，而且目标值不变的话，那么的出来的侧偏永远都是同一个符号
	   Last_Target_Heading = POS_Heading(Last.latitude,Last.longitude,Target.latitude,Target.longitude);
	
	   use_Heading = Last_Target_Heading - Current_Target_Heading;
		 //转换到180度格式3
		 use_Heading = To_180_degrees(use_Heading);
	   //算出侧偏距
	   CrossDistance = PositionErr * sin(use_Heading * 0.017453278f);//转成弧度制	 
     */
		 //使用点到直线距离来计算侧偏距比较好
		 LinePoint_Distance(0,0,0,0);




		 Control.Task.Position_Err = LIMIT(CrossDistance,-10,10) * Control.Task.Position_Kp;
		 Control.Task.Position_i  += Control.Task.Position_Err * Control.Task.Position_Ki * T;
		 Control.Task.Position_i   = LIMIT(Control.Task.Position_i,-40,40);
		 Control.Task.Position_Out = Control.Task.Position_Err + Control.Task.Position_i;//这个是侧偏
		 
		 
		 
		 //计算偏航，使用导弹的引导方式
		 if((Current.latitude != Target.latitude)||(Current.longitude != Target.longitude))//如果经纬度一样，那么不求解
		 {
		    HeadingErr = Control.Senser.GPS.course -  POS_Heading(Current.latitude,Current.longitude,Target.latitude,Target.longitude);
		 }
		 else
		 {
			  HeadingErr = 0;
		 }
		 //转化到180度格式
		 HeadingErr = To_180_degrees(HeadingErr);
		 
		 Control.Task.Heading_Out  = Control.Task.Heading_Kp * HeadingErr;//这个是偏航角
		 
		 
		 
		 
		 
		 
		 
		 Control.Task.PositionOutPut = LIMIT(Control.Task.Position_Out,-60,60);
		 Control.Task.HeadingOutPut  = LIMIT(Control.Task.Heading_Out,-60,60);
		 Control.Task.SpeedOutPut    = LIMIT(Control.Task.Speed_Out,0,90);
}





