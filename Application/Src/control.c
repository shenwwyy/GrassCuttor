
#include "control.h"
#include "math.h"
#include <stdio.h>
#include <stdbool.h>
#include "motor.h"
#include "mymath.h"

_controlDef Control;



//执行任务
/*
任务形式分为多种
0）待机任务
1）割草任务
2）充电任务
3) 返航任务
 */

//任务管理级别
void Control_TaskManage(float T,uint32_t id)
{
	    //更新当前的位置信息
	    Control.Task.CurrentPoint.altitude  = Control.Senser.GPS.altitude;
	    Control.Task.CurrentPoint.latitude  = Control.Senser.GPS.latitude;
	    Control.Task.CurrentPoint.longitude = Control.Senser.GPS.longitude;
	    Control.Task.CurrentPoint.course    = Control.Senser.GPS.course;
	    Control.Task.CurrentPoint.speed     = Control.Senser.GPS.speed;
	
	    
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
	
	float *UNUSED;
	
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
		 if(Control.Senser.Voltage.Battery1.Battery <= (Control.Senser.Voltage.Battery1.Max - Control.Senser.Voltage.Battery1.Min)*0.3f)
		 {
				Control.Command.WannaTask = -1;//清除切草任务命令
			  Control.Task.Task_id = ChargingTask;//切换至充电任务
			  Control.Task.Charging.ChargeStatus = uncharge;//在充电的路上
		 }
	}
	
	
	//清空输出
	Control.Task.PositionOutPut = 0;
	Control.Task.HeadingOutPut  = 0;
	
	
}

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
		  //计算是否应该切换航线
		  if(Control_PolygonCheck(Control.Task.CurrentPoint,Control.Task.PointGroups,Control.Task.PointGroupsNumber,1) == true)
			{
				   //切换新的航点作为目标
				   //保持当前点作为上一个点
				   Control.Task.LastPoint   = Control.Task.CurrentPoint;
				   //给目标点赋值
				   Control.Task.TargetPoint = Control.Task.TargetPoint;
			}
/*		
		  //当前位置和参考边的距离
		  //计算经纬度，每次上一个点添加0.0000002度即20cm宽度
		
			if(Control.Task.firstTimeIntoCircle == 0x01)
			{
				 Control.Task.firstTimeIntoCircle = 0x00;
				
				 //下一个目标点经纬度变化，然后赋值给目标点。
			}
		*/	
			
		  //计算当前位置和参考方向的偏差
		  Control_Route(T,Control.Task.CurrentPoint,Control.Task.TargetPoint,Control.Senser.Sonar,0);
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
			 Control_Route(T,Control.Task.CurrentPoint,Control.Task.TargetPoint,Control.Senser.Sonar,0);
			
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
			 Control.Task.PositionOutPut = 0;
			 Control.Task.HeadingOutPut  = 0;
			 
		}break;

	}
	
	
}

void Control_BackHomeTask(float T)
{
	//收到返航命令，小车目标改为充电桩位置，并且一路保持避障，回到充电桩附近，然后寻找充电设备，精确对上后，进入充电桩，完成后切换到空闲任务
	//这个过程小车可以接受任务命令，例如去一块新的地方进行割草任务，停止刹车等
	//
	
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
	float mLat = lat2 - lat1;
	float mLon = (lon2 - lon1)* cos(((lat2 + lat1)/2)* 0.0174532925f);
	temp = 90.0f + atan2(-mLat, mLon) * 57.2957795f;

	if(temp < 0)temp += 360.0f;
	return temp;
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
void Control_Route(float T,_point Current,_point Target,_sonar Sonar,float PosOffset)
{
	   float Kp = 0;
	   float PositionErr,HeadingErr;
	   float Current_Target_Heading;
	   float CrossErr;
	
	   uint8_t SonarFlag = 0;
	
	   //计算当前点和目标点的距离，得出前向的输出
	
	   PositionErr = POS_Distance(Current.latitude,Current.longitude,Target.latitude,Target.longitude);
	   
	   //计算前向，侧向的障碍物，结合当前的航迹角，得出侧向控制输出
	   //0~360
	   Current_Target_Heading = POS_Heading(Current.latitude,Current.longitude,Target.latitude,Target.longitude);
	
	   HeadingErr = Current_Target_Heading - Current.course;
	   HeadingErr = To_180_degrees(HeadingErr);
	
	   //计算侧偏
	   
	
	
	   //通过侧向的障碍物判断是否要增加航向角误差
	   
	   if((Sonar.left.isValid == 0x01)&&(Sonar.left.distance <= 0.2f)) SonarFlag |= 0x04;//0000 0100
		 else                                                            SonarFlag &= 0xfb;//1111 1011
		 
		 if((Sonar.forward.isValid == 0x01)&&(Sonar.forward.distance <= 0.2f)) SonarFlag |= 0x02;//0000 0010
		 else                                                                  SonarFlag &= 0xfd;//1111 1101
		 
		 if((Sonar.right.isValid == 0x01)&&(Sonar.right.distance <= 0.2f)) SonarFlag |= 0x01;//0000 0001
		 else                                                              SonarFlag &= 0xfe;//1111 1110
		 
		 SonarFlag &= 0x07;//0000 0111
	
	   switch(SonarFlag)
		 {
			 case 0://没有障碍物
			 {
				   HeadingErr += 0;
			 }break;
			 case 1://右侧有障碍物
			 {
				   //偏航
				   HeadingErr += (-10.0f) * T;
				   
			 }break;
			 case 2://前向有障碍物
			 {
				   //停止，转向
				   //把距离目标值置零，让电机停止
				   PositionErr = 0;
				   //偏航
				   HeadingErr += (-45.0f) * T;
				 
			 }break;
			 case 3://右侧和前向有障碍物
			 {
				   //停止，转向
				   //把距离目标值置零，让电机停止
				   PositionErr = 0;
				   //偏航
				   HeadingErr += (-45.0f) * T;
				 
			 }break;
			 case 4://左侧有障碍物
			 {
				   //偏航
				   HeadingErr += (-10.0f) * T;
			 }break;
			 case 5://左右两侧有障碍物
			 {
				   //保持直行，并且调整两侧的侧偏一致
				   HeadingErr += (Sonar.right.distance - Sonar.left.distance) * 10.0f * T;
			 }break;
			 case 6://左侧和前向有障碍物
			 {
				   //停止，转向
				   //把距离目标值置零，让电机停止
				   PositionErr = 0;
				   //偏航
				   HeadingErr += (45.0f) * T;
			 }break;
			 case 7://三个方向都有障碍物
			 {
				   //停止，掉头
				   PositionErr = 0;
				   //偏航
				   HeadingErr += (-180.0f) * T;
			 }break;
			
		 }
		 
		 
		 Control.Task.PositionOutPut = LIMIT(10.0f * PositionErr - 2.0f * Current.speed,-1000,1000);
		 Control.Task.HeadingOutPut  = LIMIT(10.0f * HeadingErr + 10.0f * PosOffset,-500,500);
}






