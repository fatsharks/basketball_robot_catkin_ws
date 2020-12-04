#include <ros/ros.h>
#include "decisionmaking/decisionmaking.h"

#define STOP 9999
#define VX1 500
#define VX2 -500

using namespace std;
using namespace ros;

DecisionMaking::DecisionMaking(unsigned char _normalNum, unsigned char _placeNum):Normal_num(_normalNum), place_num(_placeNum)
{
	detect_sub = nh.subscribe("/detect_result", 1000, &DecisionMaking::detect_callback, this);
	//订阅感知部分传过来的数据
	serial_sub = nh.subscribe("/dsp_to_pc", 1000, &DecisionMaking::serial_callback, this);
	//订阅底层部分传过来的数据
	laser_sub =nh.subscribe("/scan", 1, &DecisionMaking::laser_callback, this);
	//订阅激光雷达数据，用于避障
	detect_pub = nh.advertise<std_msgs::Int32>("/target_type", 1000);
	//发布视觉感知命令
	serial_pub = nh.advertise<const_msg::pc_to_dsp>("/pc_to_dsp", 1000);
	//发布底层控制命令

	dsp_status.fAngle = 0;
	dsp_status.XDist = 0;
	dsp_status.YDist = 0;

	//控制命令初始化
	Control.V1 = 0;
	Control.V2 = 0;
	Control.V3 = 0;
	Control.V4 = 0;
	Control.flag_start_stop = 1; //松开电机，允许运行

	//PidAngle控制变量初始化（包括p,i,d三个参数）
	PidAngle_init(2, 0, 0);
	PidDistX_init(2, 0, 0);
	PidDistY_init(2, 0, 0);

	//单片机允许发送指令标志
}

DecisionMaking::~DecisionMaking()
{
}

void DecisionMaking::detect_callback(const const_msg::object_paramConstPtr &obj_param)
{
	objectAngleDistance_Y.first = obj_param->obj_angle;
	objectAngleDistance_Y.second = obj_param->obj_distance;
}

void DecisionMaking::serial_callback(const const_msg::dsp_to_pcConstPtr &dsp_state)
{
	dsp_status.XDist = dsp_state->XDist;
	dsp_status.YDist = dsp_state->YDist;
	dsp_status.Vx = dsp_state->Vx;
	dsp_status.Vy = dsp_state->Vy;
	dsp_status.fAngle = dsp_state->fAngle;
	dsp_status.DW = dsp_state->dw;
	dsp_status.RecData_state = dsp_state->RecData_State;
	Strategy();
	control_pub(Control);
}

void DecisionMaking::laser_callback(const sensor_msgs::LaserScanConstPtr& laser)
{
	data.clear();
	for (size_t i = 0; i < laser->ranges.size(); i++)
	{
		long temp_1 = laser->ranges[i] * 1000; //单位换算，从meter_to_micrometer
		data.push_back(temp_1);
	}
}

void DecisionMaking::object_pub(objectParameter &obj)
{
	std_msgs::Int32 object_msg;
	object_msg.data = (int)obj.whatObject;
	detect_pub.publish(object_msg);
}

void DecisionMaking::control_pub(DSPControl &ctrl)
{
	const_msg::pc_to_dsp control_msg;
	control_msg.V1 = ctrl.V1;
	control_msg.V2 = ctrl.V2;
	control_msg.V3 = ctrl.V3;
	control_msg.V4 = ctrl.V4;
	control_msg.flag_start_stop = ctrl.flag_start_stop;
	control_msg.SendData_State = ctrl.SendData_state;
	Strategy();
	serial_pub.publish(control_msg);
}

void DecisionMaking::Strategy()
{
	/*******等补充******读取控件Normal_num值********/
	//place_num=1;		//（场地号）1是向右出发，y取正值；2是向左出发，y取负值。
	// Normal_num=9;	//当通过mfc的界面控件选择控制程序（pass1-3或bat1-3）时，这句要屏蔽。

	//********************下面补充回合选择**********************
	switch (Normal_num) //Normal_num还未初始化，最终由控件控制
	{
	case 0:
		Normal_pass0();
		break;
	case 1:
		Normal_pass1();
		break;
	case 2:
		Normal_pass2();
		break;
	case 3:
		Normal_pass3();
		break;
	case 4:
		Normal_bat1_new();
		break;
	case 5:
		Normal_bat2_new();
		break;
	case 6:
		Normal_bat3_new();
		break;
	case 7:
		Normal_position();
		break;
	case 8:
		Normal_AtoB_test();
		break;
	case 9:
		Normal_Pttest();
		break;
	case 10:
		Normal_avoidance();
		break;
	case 11:
		Normal_return_test(); 
		break;
	case 12:
		challenge_competition();
		break;
	case 19:
		Normal_test();
		break;
	default:
		break;
	}
}

bool DecisionMaking::find_object_by_laser(vector<long> &data, vector<pair<double, long>> &objectAngleDist)
{
	if (!data.empty())
	{
		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i] > 5000)
			{
				data[i] = 5000;
			}
		}
		int start = 0;
		int end = 0;
		for (size_t i = 260; i < 821; i++)
		{
			long dataSub = data[i + 1] - data[i];
			//if(dataSub<-300 && !start && data[i+1]<9000){
			if (dataSub < -300 && data[i + 2] < 4000)
			{
				start = i + 1; //get object start index
			}
			if (dataSub > 300 && start && data[i - 1] < 4000)
			{
				end = i; //get object end index
			}
			if (start && end)
			{
				if (end - start >= 4)
				{ //eliminate small object
					long centerDist = data[(int)((start + end) / 2)];
					pair<double, long> tempPair;
					tempPair.first = 135 - 0.25 * ((int)(start + end) / 2);
					tempPair.second = centerDist;
					objectAngleDist.push_back(tempPair);
				}
				start = 0; //initialize start and end value so as to find next object
				end = 0;
			}
		}
	}
	if (!objectAngleDist.empty())
	{
		return true;
	}
	else
	{
		return false;
	}
}

int DecisionMaking::find_object(vector<long> &data, vector<pair<double, long>> &objectInfo)
{
	vector<pair<double, long>> objectAngleDist;
	if (find_object_by_laser(data, objectAngleDist))
	{
		size_t counter = 0;
		for (size_t i = 0; i < objectAngleDist.size(); i++)
		{
			if (objectAngleDist[i].first >= 0)
			{
				counter++;
			}
		}
		if (counter > 0 && counter < objectAngleDist.size())
		{
			double maxAngle = 90;
			double minAngle = -90;
			pair<double, long> tempAngleDist;
			for (size_t i = 0; i < objectAngleDist.size(); i++)
			{
				if (objectAngleDist[i].first > 0 && objectAngleDist[i].first < maxAngle)
				{
					tempAngleDist.first = objectAngleDist[i].first;
					tempAngleDist.second = objectAngleDist[i].second;
					maxAngle = objectAngleDist[i].first;
				}
			}
			objectInfo.push_back(tempAngleDist);
			for (size_t i = 0; i < objectAngleDist.size(); i++)
			{
				if (objectAngleDist[i].first < 0 && objectAngleDist[i].first > minAngle)
				{
					tempAngleDist.first = objectAngleDist[i].first;
					tempAngleDist.second = objectAngleDist[i].second;
					minAngle = objectAngleDist[i].first;
				}
			}
			objectInfo.push_back(tempAngleDist);
			if (2 == objectInfo.size())
			{
				return 2;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			if (counter == objectAngleDist.size())
			{
				double maxAngle = 90;
				pair<double, long> tempAngleDist;
				for (size_t i = 0; i < objectAngleDist.size(); i++)
				{
					if (objectAngleDist[i].first < maxAngle)
					{
						tempAngleDist.first = objectAngleDist[i].first;
						tempAngleDist.second = objectAngleDist[i].second;
						maxAngle = objectAngleDist[i].first;
					}
				}
				objectInfo.push_back(tempAngleDist);
				return 1;
			}
			else
			{
				double minAngle = -90;
				pair<double, long> tempAngleDist;
				for (size_t i = 0; i < objectAngleDist.size(); i++)
				{
					if (objectAngleDist[i].first > minAngle)
					{
						tempAngleDist.first = objectAngleDist[i].first;
						tempAngleDist.second = objectAngleDist[i].second;
						minAngle = objectAngleDist[i].first;
					}
				}
				objectInfo.push_back(tempAngleDist);
				return -1;
			}
		}
	}
	else
	{
		return 0;
	}
}

void DecisionMaking::Normal_position() //直行测试
{
	short Vx, Vy, W;
	float jd;

	Vx = 450;
	Vy = 0;
	W = 0;
	jd = 0;
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
}

void DecisionMaking::Normal_avoidance() //避障测试
{
	static int intention_num = 0;	 //回合过程的意图顺序号
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptM, ptC ,ptTemp;
	float temp_angle;
	ptM.x = 5000;
	ptM.y = 0;
	ptC.x = 3200; 
	ptC.y = 800; //用G走折线时坐标4930

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 150) //延时17.1s
		{
			temp_normal_time = normal_time;
			intention_num = 81;
		}
		break;

	case 8: //转向目的点，去找三分球
		ROS_INFO_STREAM("case 888888888888888888888888\n");
		Control.SendData_state = 0; //单片机位置0
		if (ToAngleByMPU6050(ptC))
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 81;
			}
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81 81\n");
		if (ToPostureByAvoidance(ptC)) //避障
		{
			temp_normal_time = normal_time;
			intention_num = 810;
		}
		break;

	case 810:
		ROS_INFO_STREAM("case 810 810 810 810 810 810 810 810 810 810 810 810 810 810 810 810\n");
		if ((normal_time - temp_normal_time) < 100)
				{
					robotforward();
				}
		else
		{
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82: //避障后再次对准目标点
		ROS_INFO_STREAM("case 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82 82\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptC))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 9;
				}
			}
		}
		break;

	case 9:
		ROS_INFO_STREAM("case 999999999999999999999999999999999999999999999999999999999999999999999999\n");
		if (ToPostureByMilemeter_Pid2(ptC))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 10;
		}
		break;

	case 10:
		Stop();
		break;

	default:
		break;
	}

	/*if(ToPostureByAvoidance(ptM))
	{
		Stop();
		ROS_INFO_STREAM(" ToPostureByAvoidance(ptM) ToPostureByAvoidance(ptM)ToPostureByAvoidance(ptM)\n");
	}*/

	Control.flag_start_stop = 1; //允许电机转动
}

void DecisionMaking::Normal_return_test() //归位测试
{
	static int intention_num = 0;	 //回合过程的意图顺序号
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptB;

	ptOO.x = 1000.0;
	ptOO.y = 0.0; //回合结束前，用于回位定点

	ptA.x = 3000.0;
	ptA.y = 0.0; //A点

	ptB.x = 3000.0;
	ptB.y = 3000.0; //B点：由A→B需要：右转90度

	//设计两种归位方式：1、点对点直接归位，不掉头； 2、掉头归位，中途避障
	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 45) //延时2s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA)) //到达A点
		{
			Stop();
			ROS_INFO_STREAM("到 AAAAAAAAAAAAAAAAAAAAAAAAA\n");
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			temp_normal_time = normal_time;
			if (ToAngleByMPU(90)) //旋转90度
			{
				Stop();
				ROS_INFO_STREAM("转 90 90 90 90 90 90 90 90 9\n");
				intention_num = 3;
			}
		}
		break;

	case 3:
		ROS_INFO_STREAM("case 3333333333333333333333333333333\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			temp_normal_time = normal_time;
			if (ToPostureByMilemeter_Pid2(ptB)) //到达B点
			{
				Stop();
				ROS_INFO_STREAM("到 BBBBBBBBBBBBBBBBBBBBBBBBBBBB\n");
				intention_num = 11;
			}
		}
		break;

	case 11: //旋转对准pt00点
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 111\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 3)
		{
			if (ToPostureByMilemeter_Pid2(ptOO)) //边走边调整角度先回位OO点，再微调退回框
			//if (ToAngleForHomeByMPU6050())//走折线回位
			//if(ToAngleByMPU6050(ptOO))//先对准OO，再去OO，然后微调回位
			{
				ROS_INFO_STREAM("到 00 00 00 00 00 00 00 00 00\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 12;
				}
			}
		}
		break;

		//case 1200:

		//    if (fabs(dsp_status.fAngle)>2.0)
		//	{
		//		ToAngleByMPU(0);
		//	}
		//	else
		//	{
		//		Stop();
		//		temp_normal_time=normal_time;
		//		intention_num=121;
		//	}
		//	break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12 12 \n");
		ROS_INFO("入库时角度大于2 ！！！需要旋转调整！！！jiaodu jiaodu before ...................:%lf\n", dsp_status.fAngle);
		dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);
		ROS_INFO("入库时角度大于2 ！！！需要旋转调整！！！jiaodu jiaodu after ...................:%lf\n", dsp_status.fAngle);
		if ((dsp_status.fAngle > 2.0) || (dsp_status.fAngle < -2.0)) // m_DSPComm.m_dsp_status.fAngle
		{
			/*dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);*/
			/*ROS_INFO_STREAM("入库时角度大于2 ！！！需要旋转调整！！！jiaodu jiaodu before ...................:%lf\n",dsp_status.fAngle);
			dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);
			ROS_INFO_STREAM("入库时角度大于2 ！！！需要旋转调整！！！jiaodu jiaodu after ...................:%lf\n",dsp_status.fAngle);*/
			if (dsp_status.fAngle > 0.0)
			{
				ROS_INFO_STREAM("turn left ... turn left ... turn left ... turn left ... turn left ... \n");
				//ToAngleByMPU(0);
				ToFindObjectByturnleft();
			}
			else
			{
				ROS_INFO_STREAM("turn right ... turn right ... turn right ... turn right ... turn right ... \n");
				ToFindObjectByturnright();
			}
		}
		else
		{
			ROS_INFO("角度对准了，为0了！！！jiaodu jiaodu .....................:%lf\n", dsp_status.fAngle); //就这里，总是为0
			Stop();
			intention_num = 121;
		}
		break;

	case 121:
		ROS_INFO_STREAM("case 121 121 121 121 121 121 121\n");
		if (abs(dsp_status.YDist) >= 30)
		{
			ROS_INFO("入库时Y轴没对准！！！需要左右调整！！！zuobiao zuobiao ...................: %ld, %ld\n", dsp_status.XDist, dsp_status.YDist);
			if (dsp_status.YDist > 0)
			{
				ROS_INFO_STREAM("GO left ... GO left ... GO left ... GO left ... GO left ... GO left ... \n");
				robotstraightleft();
			}
			else
			{
				ROS_INFO_STREAM("GO right ... GO right ... GO right ... GO right ... GO right ... GO right ... \n");
				robotstraightright();
			}
		}
		else
		{
			ROS_INFO_STREAM("入库时左右对准了！！！不要动了！！！stop stop .....................................................");
			Stop();
			intention_num = 122;
		}
		break;

	/*case 1201:
		if (fabs(dsp_status.fAngle)>0.5)
		{
			ToAngleByMPU(0);
		}
		else
		{
			Stop();
			temp_normal_time=normal_time;
			intention_num=123;
		}
		break;*/
	case 122:
		ROS_INFO_STREAM("case 122 122 122 122 122 122 \n");
		ROS_INFO("入库时角度大于0.5！！！需要旋转调整！！！jiaodu jiaodu before ...................:%lf\n", dsp_status.fAngle);
		dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);
		ROS_INFO("入库时角度大于0.5 ！！！需要旋转调整！！！jiaodu jiaodu after ...................:%lf\n", dsp_status.fAngle);
		if ((dsp_status.fAngle > 0.5) || (dsp_status.fAngle < -0.5))
		{

			/*dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);*/
			/*ROS_INFO_STREAM("入库时角度大于0.5！！！需要旋转调整！！！jiaodu jiaodu before ...................:%lf\n",dsp_status.fAngle);
			dsp_status.fAngle = DecisionMaking::mth_ChangeAngle(dsp_status.fAngle);
			ROS_INFO_STREAM("入库时角度大于0.5 ！！！需要旋转调整！！！jiaodu jiaodu after ...................:%lf\n",dsp_status.fAngle);*/
			if (dsp_status.fAngle > 0.0)
			{
				ROS_INFO_STREAM("turn left ... turn left ... turn left ... turn left ... turn left ... \n");
				ToFindObjectByturnleft();
			}
			else
			{
				ROS_INFO_STREAM("turn right ... turn right ... turn right ... turn right ... turn right ... \n");
				ToFindObjectByturnright();
			}
		}
		else
		{
			ROS_INFO_STREAM("角度再一次对准了，为0了！！！jiaodu jiaodu ................................................\n");
			Stop();
			intention_num = 123;
		}
		break;

	case 123:
		ROS_INFO_STREAM("case 123 123 123 123 123 123 123 123 123 123 123\n");
		if (dsp_status.XDist > (-10))
		{
			ROS_INFO_STREAM("back back back back back back back back back back back back back back back back\n");
			robotbacklow();
		}
		else
		{
			ROS_INFO_STREAM("stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop stop \n");
			Stop();
			intention_num = 15;
		}
		break;

	case 15:
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15 15  15\n");
		Stop(); //停车
		break;

	default:
		Stop();
		normal_time = 0;
		break;
	}
}

void DecisionMaking::Normal_Pttest() //陀螺仪测试：旋转多少度
{
	static int intention_num = 0;	 //回合过程的意图顺序号
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptM;

	ptOO.x = 1000.0;
	ptOO.y = 0.0; //回合结束前，用于回位定点

	ptM.x = 0000.0;
	ptM.y = 4000.0; //M点

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 100) //延时2s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;
		
	case 1:
		// Control.V1 = 500;
		// Control.V2 = 500;
		// Control.V3 = 500;
		// Control.V4 = 500;
		ROS_INFO_STREAM(" Er is 111111111111:");
		if (ToAngleByMPU(90))
		{
			if (stop())
			{
				ROS_INFO_STREAM("Normal_Pttest() stop...............................................................\n");
				intention_num = 2;
			}
		}
		break;

	case 2:
		Stop();
		break;
	}
}

void DecisionMaking::Normal_AtoB_test() //定点测试：O -> A -> B
{
	static int intention_num = 0;	 //回合过程的意图顺序号
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptB;

	ptOO.x = 1000.0;
	ptOO.y = 0.0; //回合结束前，用于回位定点

	ptA.x =5600;
	ptA.y = 0; //A点

	ptB.x = 0;
	ptB.y = 5600; //B点

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 45) //延时2s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 1111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA)) //到达A点
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 4;
		}
		break;
		
	case 3:
		ROS_INFO_STREAM("case 333333333333333333333333333\n");
		{
			temp_normal_time = normal_time;
			if (ToPostureByMilemeter_Pid6(ptB)) //到达B点
			{
				Stop();
				intention_num = 4;
			}
		}
		break;

	case 4:
		ROS_INFO_STREAM("case 44444444444444444444444444\n");
		if ((normal_time - temp_normal_time) > 20)
		Stop();

	default:
		Stop();
		normal_time = 0;
		break;
	}
}


void DecisionMaking::Normal_test()
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	static int vision_scan_time = 0; //视觉扫描次数统计，达到设定值时，车子要回归原位
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptB, ptM, ptM2, ptN, ptxy;

	ptOO.x = 4800;
	ptOO.y = 0; //回合结束前，用于回位定点

	ptA.x = 3000;
	ptA.y = -3000; //车子带着篮球进入传球边界线内的点，车子在此处往目标点M发射球

	ptM.x = 0;
	ptM.y = -900;


	double Ed; //定义当前位置到目标点的距离
	float jd;
	short Vx, Vy, W;
	Posture ept;	
	ept.x = (ptA.x - dsp_status.XDist);
	ept.y = (ptA.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept);

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 50) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2: //确认有球后，有球，有球有球！！！！！
		if (ToAngleByMPU6050(ptA)) //准备发射
		{
			ROS_INFO_STREAM("已经对准了传球定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停到了投球定位点，再旋转调整！！！！\n");
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 91;
			}
		}
		break;

	case 91:
		if (ToPostureByMilemeter_Pid1(ptA)) //到达A点
		{
			temp_normal_time = normal_time;
			intention_num = 3;
		}
		break;

	case 3:
		ROS_INFO_STREAM("case 3333333333333333333333333\n");
		Stop();
		break;

	}
}

void DecisionMaking::Normal_pass0() //激光摄像头测试
{
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;
	static bool armupflag = 0;
	static bool stopflag = 0;
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000000000000000000000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 45) //延时2S
		{
			/*if(ToBallByVision_new())
			{*/
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 2;
			//}
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 2222222222222222222222\n");
		//temp_normal_time=normal_time;     //为延时作准备
		the_object.whatObject = (objectType)2;//排球
		object_pub(the_object);

		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			robotstraightright(); //左移找球测试
			ROS_INFO_STREAM(" i am turninhg left \n");
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					intention_num = 2;
					ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				Stop();
				temp_normal_time = normal_time; //为延时作准备
				intention_num = 4101;
			}
		}
		break;

	case 4101:
		if (normal_time - temp_normal_time < 160)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 501;
		}
		break;

	case 501:
		if(stop())
		intention_num = 502;
		break;

	case 502:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 5;
		break;

	case 5:
		ROS_INFO_STREAM("case 555555555555555555555555555555\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 6; //抬机械臂命令已发送
		}
		break;
		
	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 66666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 10)
		{
			temp_normal_time = normal_time;
			intention_num = 7;
		}
		break;

	case 7:
		if ((normal_time - temp_normal_time) > 3)
		Stop();
		break;
	}
}

void DecisionMaking::Normal_pass1() //**********传球-回合1**********
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	static int vision_scan_time = 0; //视觉扫描次数统计，达到设定值时，车子要回归原位
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptOO, ptA, ptA0, ptB, ptM, ptM2, ptxy;

	ptOO.x = 1500;//1000
	ptOO.y = 0; //0  //回合结束前，用于回位定点

	ptA.x = 2450;//3500
	ptA.y = 0; //0    //车子带着篮球进入传球边界线内的点，车子在此处往目标点M发射球

	if (1 == place_num)
	{
		ptxy.x = 0;//0
		ptxy.y = 0;//100

		ptM.x = 550;//800
		ptM.y = 5200; //4250  //标定柱

		ptM2.x = 550;//0
		ptM2.y = 5200;
	}
	if (2 == place_num)
	{
		ptxy.x = 0;//-100
		ptxy.y = 0;//-100

		ptM.x = -150;//0
		ptM.y = -6100;//-4300 //A口进入时投球区中心位置

		ptM2.x = -150;//0
		ptM2.y = -5200;//-4300 //A口进入时投球区中心位置
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 50) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA)) //走到点A
		{
			if (stop())
			{
				Stop();
			}
			
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222222222\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			ROS_INFO_STREAM("are aiming to the posture M！！！！\n");
			if (ToAngleByMPU(110)) //瞄准目标点M
			{
				temp_normal_time = normal_time;
				intention_num = 301;
				
			}
		}
		break;

	case 301:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 302;
		}
		break;
	case 302:
		Stop();
		intention_num = 3;
		break;

	case 3: //发射第一个篮球
		ROS_INFO_STREAM("case 3333333333333333333333333333\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 30;
		}
		break;

	case 30:
		ROS_INFO_STREAM("case 30 30 30 30 30 30 30 30 \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0; //单片机位置0
			if (ToAngleByMPU(0))
			{
				ROS_INFO_STREAM("are turning to angle 0 to fin d the ball 2\n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 4;
			}
		}
		break;

	case 4: //**********视觉找篮球开始**********
		ROS_INFO_STREAM("case 44444444444444444\n");
			the_object.whatObject = (objectType)2;//蓝灰色篮球
			object_pub(the_object);
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if ((normal_time - temp_normal_time) < 140)//160
				{
					robotstraightleft(); //开环横向低速右移
					ROS_INFO_STREAM(" i'm turning right \n");
				}
				else
				{
					if (abs(dsp_status.fAngle) > 3)
					{
						if (ToAngleByMPU(0));
					}
					else
					{
						robotstraightright(); //开环横向低速右移
						ROS_INFO_STREAM(" i'm turning right \n");
					}
					
				}
				if ((dsp_status.YDist) > 3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("not find ball let'go home！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 11;
				}
			}
			if (2 == place_num)
			{
				if ((normal_time - temp_normal_time) < 130)//160
				{
					robotstraightright(); //开环横向低速右移
					ROS_INFO_STREAM(" i'm turning right \n");
				}
				else
				{
					if (abs(dsp_status.fAngle) > 3)
					{
						if (ToAngleByMPU(0));
					}
					else
					{
						robotstraightleft(); //开环横向低速右移
						ROS_INFO_STREAM(" i'm turning right \n");
					}
				}
				if ((dsp_status.YDist) < -3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("not find ball let'go home！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 11;
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" cccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4;
					ROS_INFO("lost the ball ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (aimballflag == 1))
		{
			ROS_INFO_STREAM(" ffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4;
					ROS_INFO_STREAM(" lost the ball again ！！！\n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					// temp_normal_time = normal_time;
					// intention_num = 401;昨天改的
					Stop();
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 4101;
					ROS_INFO_STREAM("catch the ball ！！！\n");
				}
			}
		}
		break; //**********视觉找篮球结束**********

	case 4101:
		if (normal_time - temp_normal_time < 106)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 501;
		}
		break;

	case 501:
		if(stop())
		intention_num = 502;
		break;

	case 502:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 5;
		break;

	case 5:
		ROS_INFO_STREAM("case 555555555555555555555555555555\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 6; //抬机械臂命令已发送
		}
		break;

	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 6666666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 80)
		{
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		break;

	case 9: //确认有球后，有球，有球有球！！！！！
		ROS_INFO_STREAM("case 99999999999999999999999999999999999\n");
		ROS_INFO_STREAM("正在对准传球定位点！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptA)) //准备发射
		{
			ROS_INFO_STREAM("已经对准了传球定位点！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("对准之后已经停到了投球定位点，再旋转调整！！！！\n");
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 91;
			}
		}
		break;

	case 91:
		ROS_INFO_STREAM("case 91 91 91 91 91 91 91 91 91 91 91 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid2(ptA)) //到达A点
		{
			temp_normal_time = normal_time;
			intention_num = 92;
		}
		break;

	case 92:
		ROS_INFO_STREAM("case 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 92 \n");
		ROS_INFO_STREAM("are aimiing the column！！！！\n");
		if (ToAngleByMPU(110))//ptM2
		{
				temp_normal_time = normal_time;
				intention_num = 920;
		
		}
		break;

	case 920:
		if(stop())
		temp_normal_time = normal_time;
		intention_num = 921;
		break;

	case 921:
		Stop();
		intention_num = 10;
		break;

	case 10:
		ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("发射 发射 发射 ！！！！\n");
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 11;
		}
		break;

	case 11: //**********车子归位开始**********
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 11 \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0; //单片机位置0
			if (ToAngleByMPU6050(ptOO))//180
			{
			ROS_INFO("the posture is right %lf\n", dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 110;
			}
		}
		break;

	case 110:
		ROS_INFO_STREAM("case 110 110 110 110 110\n");
		//Control.SendData_state=0;
		if ((normal_time - temp_normal_time) > 10)
		{
			if (ToPostureByMilemeter_Pid2(ptOO))
			{
				ROS_INFO_STREAM("stop stop stop stop stop  stop stop stop stop stop \n");
				Stop();
				intention_num = 1210;
			}
		}
		break;

	case 1210:
		if(stop())
		intention_num = 1211;
		break;

	case 1211:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 121;
		break;

	case 121:
		ROS_INFO_STREAM("case 121 121 121 121\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 8; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 122; //抬机械臂命令已发送
		}
		break;

	case 122: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 122 122 122 122\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 50)
		{
			temp_normal_time = normal_time;
			intention_num = 12;
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12\n");
		if (ToAngleByMPU6050(ptxy))//180
		{
			ROS_INFO("the posture is right %lf\n", dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 124;
		}
		break;

	case 124:
		ROS_INFO_STREAM("case 124 124 124 124 124 124 124 124 124 124 124 124 124 \n");
		//Control.SendData_state=0;
		if ((normal_time - temp_normal_time) > 10)
		{
			if (ToPostureByMilemeter_Pid2(ptxy))
			{
				ROS_INFO_STREAM("stop stop stop stop stop  stop stop stop stop stop \n");
				Stop();
				intention_num = STOP;
			}
			
		}
		break;

	case STOP:
		ROS_INFO_STREAM("case 125 125 125 125 125 125 125 125 125 125 \n");
		Stop();
		break;

	default:
		ROS_INFO_STREAM("case default default default default default default default \n");
		normal_time = 0;
		break; //**********车子归位结束**********
	}
}


void DecisionMaking::Normal_pass2() //**********传球-回合2**********
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号

	static int armupflag = 0; //抬臂标志
	static int stopflag = 0;  //停车标志

	static bool findballflag = 0;			 //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;			 //对准目标标志位，每对准一次目标后都要记得清零
	static bool Iscircleavoidancecorner = 0; //中圈避障是否从边缘靠近了
	static bool Isthreescoreabs45ready = 0;	 //三分线角度调整完成判断

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	//这些点如果需要控件改变值，需要定义在成员变量，并在初始化函数里通过控件得到初始化值
	Posture ptOO, ptA0, ptC, ptM, ptM2, ptxy; //ptZ时中圈进口有障碍球时临时确定的辅助点
															 //终点前，用于回位定点

	if (1 == place_num)
	{
		ptOO.x = 1500;
		ptOO.y = 0;

		ptA0.x = 2500; //（用向右偏的方法是2500）	//这个地方开始判断中圈进口是否有障碍球
		ptA0.y = 0;//-800

		ptxy.x = 0;
		ptxy.y = 0;

		ptC.x = 3800; //三分线中点识别球定点处
		ptC.y = 4300;

		ptM.x = 500;
		ptM.y = 5300; //传球区中心位置（第二次）

		ptM2.x = 500;//800
		ptM2.y = 5300;
	}
	if (2 == place_num)
	{
		ptOO.x = 1500;//1200
		ptOO.y = 0;//-200

		ptA0.x = 2500; //（用向右偏的方法是2500）	//这个地方开始判断中圈进口是否有障碍球//3300
		ptA0.y = 0;//0

		ptxy.x = 0;//150
		ptxy.y = 0;//-150

		ptC.x = 3800; //三分线球点//3650
		ptC.y = -4300;//-4600

		ptM.x = -250;//800
		ptM.y = -5300; //传球区中心位置（第二次）

		ptM2.x = 0;//800
		ptM2.y = -5300;
	}
	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 50) //延时17.1s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 11111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0)) //从起点O到点A0
		{
			ROS_INFO("have arrived at the posture A0 ！！！X is : %ld，Y is : %ld, angle is : %lf\n", dsp_status.XDist, dsp_status.YDist, dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 2222222222222222222222222\n");
		if ((normal_time - temp_normal_time) > 5)
		{
				temp_normal_time = normal_time;
				intention_num = 3;
		}
		break;

		/********************************** 中线识别球 ***********************************/
	case 3: //开启识别
	if ((normal_time - temp_normal_time) > 50)
	{
		ROS_INFO_STREAM("case 3333333333333333333\n");
		the_object.whatObject = (objectType)2;//红黄色排球
		object_pub(the_object);
	
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				if ((normal_time - temp_normal_time) < 150)//60
				{
					robotstraightleft(); //开环横向低速左移
					ROS_INFO_STREAM(" i am turning left now  \n");
				}
				else
				{
					if (abs(dsp_status.fAngle) > 3)
					{
						if (ToAngleByMPU(0));
					}
					else
					{
						robotstraightright(); //开环横向低速右移
						ROS_INFO_STREAM(" i'm turning right \n");
					}
				}
				if ((dsp_status.YDist) > 3500) //找了好久，一个都没找到，就回去了
				{
					ROS_INFO_STREAM("not find the ball,let's go home！！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 11;
				}
			}
			if (2 == place_num)
			{
				if ((normal_time - temp_normal_time) < 150)
				{
					robotstraightright(); 
					ROS_INFO_STREAM(" i'm turn left now \n");
				}
				else
				{
					if (abs(dsp_status.fAngle) > 3)
					{
						if (ToAngleByMPU(0));
					}
					else
					{
						robotstraightleft(); //开环横向低速右移
						ROS_INFO_STREAM(" i'm turning right \n");
					}
				}
				if ((dsp_status.YDist) < -3500) 
				{
					ROS_INFO_STREAM("not find the ball,let's go home！！\n");
					Stop();
					temp_normal_time = normal_time;
					intention_num = 11;
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" ccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision are starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ 
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4;
					ROS_INFO("lost the ball ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;  distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (aimballflag == 1))
		{
			ROS_INFO_STREAM(" fffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision are starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ 
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4;
					ROS_INFO_STREAM(" lost the ball again ！！！\n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					Stop();
					temp_normal_time = normal_time; 
					intention_num = 4101;
					ROS_INFO_STREAM("to catch the ball！！！\n");
				}
			}
		}
	}
		break; 
	
	case 4101:
		if (normal_time - temp_normal_time < 110)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 4102;
		}
		break;
	case 4102:
		if(stop())
		intention_num = 4103;
		break;

	case 4103:
		Stop();
		intention_num = 5;
		break;

	case 5:
		ROS_INFO_STREAM("case 555555555555555555555555555555\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 1; 
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 6; 
		}
		break;

	case 6: 
		ROS_INFO_STREAM("case 6666666666666666666666666\n");
		if ((normal_time - temp_normal_time) > 80)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 8;
		}
		break;

	case 8: 
		ROS_INFO_STREAM("case 8888888888888888888\n");
		ROS_INFO_STREAM("are aiming the posture A0！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptA0)) 
		{
			ROS_INFO_STREAM("have aimed the posture A0！！！！\n");
			if (stop())
			{
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 81;
			}
		}
		break;

	case 81:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid2(ptA0)) 
		{
			temp_normal_time = normal_time;
			intention_num = 82;
		}
		break;

	case 82:
		ROS_INFO_STREAM("case 82 82 82 82 82 82 \n");
		ROS_INFO_STREAM("are aiming the column！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU(110))
		{
			Control.SendData_state = 7;
			temp_normal_time = normal_time;
			intention_num = 820;
		
		}
		break;

	case 820:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 821;
		}
		break;
	case 821:
		Stop();
		intention_num = 84;
		break;

	case 84: //发射第一个篮球
		ROS_INFO_STREAM("case 84 84 84 84 84 84 84 84\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 87;
		}
		break;


	case 87:
		ROS_INFO_STREAM("case 87 87 87 87 87 87 n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0;
			if (ToAngleByMPU6050(ptC)) 
			{
				ROS_INFO_STREAM("have aimed the posture N！！！！\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 871;
				}
			}
		}
		break;

	case 871:
		ROS_INFO_STREAM("case 81 81 81 81 81 81 81 \n");
		if (ToPostureByMilemeter_Pid6(ptC)) 
		{
			temp_normal_time = normal_time;
			intention_num = 88;
		}
		break;

	case 88:
		ROS_INFO_STREAM("case 88 88 88 88 \n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(85))
				{
					temp_normal_time == normal_time;
					intention_num = 90;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-85))
				{
					temp_normal_time == normal_time;
					intention_num = 90;
				}
			}
		}
		break;
		
		/********************************** 三分线识别球 ***********************************/
	case 90: //拾球
		ROS_INFO_STREAM("case 90 90 90 90 90\n");
		the_object.whatObject = (objectType)2;//蓝灰色篮球
		object_pub(the_object);
		
			if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs(dsp_status.XDist - 700) > 500)
					{
						ROS_INFO_STREAM("66度角，向右横向移动................\n");
						robotstraightright();
						temp_normal_time = normal_time;
					}
					else
					{
						ROS_INFO_STREAM("have arrive at boundary\n");
						Stop();
						temp_normal_time = normal_time;
						intention_num = 961;//到边界没找到那就回去吧
			
					}
				}
				if (2 == place_num)
				{
					if (abs(dsp_status.XDist - 700) > 500)
					{
						ROS_INFO_STREAM("turn left with -66%\n");
						robotstraightleft();
						temp_normal_time = normal_time;
					}
					else
					{
						ROS_INFO_STREAM("have arrive at boundary\n");
						Stop();
						temp_normal_time = normal_time;
						intention_num = 961;//到边界没找到那就回去吧
			
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			
	if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" ccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision are starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 961;
					ROS_INFO("lost the ball lost the ball ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			ROS_INFO_STREAM(" fffffffffffffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision are starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 961;
					ROS_INFO_STREAM(" lost gaain lost again ！！！\n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					Stop();
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 9001;
					ROS_INFO_STREAM("catch the ball ！！！\n");
				}
			}
		}
		break; //**********视觉找篮球结束**********

	case 9001:
		if (normal_time - temp_normal_time < 110)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 9002;
		}
		break;

	case 9002:
		if(stop())
		intention_num = 9003;
		break;

	case 9003:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 9004;
		break;

	case 9004:
		ROS_INFO_STREAM("case 9004 9004 9004 9004\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 1; 
		temp_normal_time = normal_time;
		intention_num = 9005; 
		}
		break;

	case 9005: 
		ROS_INFO_STREAM("case 9005 9005\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 961;
		}
		break;

	case 961: 
		ROS_INFO_STREAM("case 961 961 961 961\n");
		ROS_INFO_STREAM("are aiming the posture A0！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptA0))
		{
			ROS_INFO_STREAM("have aimed the postureA0！！！！\n");
			if (stop())
			{
				Control.SendData_state = 7;
				temp_normal_time = normal_time;
				intention_num = 962;
			}
		}
		break;

	case 962:
		ROS_INFO_STREAM("case 962 962 962 962\n");
		if (ToPostureByMilemeter_Pid6(ptA0)) 
		{
			temp_normal_time = normal_time;
			intention_num = 9611;
		}
		break;


	case 9611:
		ROS_INFO_STREAM("case 9611 9611 9611\n");
		ROS_INFO_STREAM("are aiming the column！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU(110))//ptM1
		{
			Control.SendData_state = 7;
			temp_normal_time = normal_time;
			intention_num = 9612;
		
		}
		break;

	case 9612:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 9613;
		}
		break;
	case 9613:
		Stop();
		intention_num = 9614;
		break;

	case 9614: 
		ROS_INFO_STREAM("case 9614 9614 9614\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 110;
		}
		break;

	case 110: //旋转对准pt00点
		ROS_INFO_STREAM("case 110 110 110 110 110  \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0; //单片机位置0
			if (ToAngleByMPU6050(ptOO))//180
			{
			ROS_INFO("the posture is right %lf\n", dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 1201;
			}
		}
		break;

	case 1201:
		ROS_INFO_STREAM("case 1201 1201 1201 1201\n");
		//Control.SendData_state=0;
		if ((normal_time - temp_normal_time) > 10)
		{
			if (ToPostureByMilemeter_Pid2(ptOO))
			{
				ROS_INFO_STREAM("stop stop stop stop stop  stop stop stop stop stop \n");
				Stop();
				intention_num = 1210;
			}
		}
		break;

	case 1210:
		if(stop())
		intention_num = 1211;
		break;

	case 1211:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 121;
		break;

	case 121:
		ROS_INFO_STREAM("case 121 121 121 121\n");
		if ((normal_time - temp_normal_time) > 12)
		{
			ROS_INFO_STREAM("case flag flag\n");
			Control.SendData_state = 8; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
			ROS_INFO_STREAM("case flag1 flag1\n");
			temp_normal_time = normal_time;
			intention_num = 122; //抬机械臂命令已发送
		}
		break;

	case 122: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 122 122 122 122\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 50)
		{
			temp_normal_time = normal_time;
			intention_num = 220;
		}
		break;

	case 220:
		ROS_INFO_STREAM("case 220 220 220 220 220 220 220 \n");
		if (ToAngleByMPU6050(ptxy))//180
		{
			ROS_INFO("the posture is right %lf\n", dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 221;
		}
		break;

	case 221:
		if ((normal_time - temp_normal_time) > 10)
		{
			if (ToPostureByMilemeter_Pid2(ptxy))
			{
				ROS_INFO_STREAM("stop stop stop stop stop  stop stop stop stop stop \n");
				Stop();
				intention_num = 9999;
			}
			
		}
		
		break;

	case 9999:
		ROS_INFO_STREAM("case is home is home is home\n");
		Stop();
		break;

	default:
		break;
	}
}


void DecisionMaking::Normal_pass3() //**********传球-回合3**********
{
	//一些决策相关的标志
	static int intention_num = 0;		  //回合过程的意图顺序号
	static bool findballflag = 0;		  //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;		  //对准目标标志位，每对准一次目标后都要记得清零
	static bool Isthreescoreunnormal = 0; //捡三分球是否出现异常
	static int normal_time = 0;			  //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0;	  //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;						  //DSP 45ms发送一次数据

	Posture ptOO, ptxy,ptAA, ptA0, ptM, ptB, ptZ, ptT;
	static Posture tempptN = {0, 0};

	ptA0.x = 2000;
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptxy.x = 0;
		ptxy.y = 0;

		ptOO.x = 1300; //终点前，用于回位定点
		ptOO.y = 0;

		ptAA.x = 3700; //中点，扫描起点
		ptAA.y = 4400;

		ptM.x = 0;
		ptM.y = 5400; //投球目标点

		ptB.x = 2200;
		ptB.y = 3200; //归位辅助点

		ptZ.x = 4500;
		ptZ.y = 2500; //归位辅助点

		ptT.x = 3000;
		ptT.y = 6800;
	}
	if (2 == place_num)
	{
		ptOO.x = 1300; //终点前，用于回位定点
		ptOO.y = 0;

		ptxy.x = 100;
		ptxy.y = 0;

		ptAA.x = 3700; //中点，扫描起点//-3900
		ptAA.y = -4400;//-4700

		ptM.x = 0;
		ptM.y = -5300; //投球目标点

		ptB.x = 2200;
		ptB.y = -3200; //归位辅助点

		ptZ.x = 4350;
		ptZ.y = -2600; //归位辅助点

		ptT.x = 3000;
		ptT.y = -6800;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000\n");
		if ((normal_time - temp_normal_time) > 50) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1: //从起点0到，转向、前往第一个三分捡球区点AA
		ROS_INFO_STREAM("case 1111111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 100;
		}
		break;

	case 100:
		ROS_INFO_STREAM("case 100 100 100 100 100\n");
		if (ToAngleByMPU6050(ptAA))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222222\n");
		if (ToPostureByMilemeter_Pid6(ptAA))
		{
			Stop();
			ROS_INFO_STREAM("have arrive at AA  AA！！！\n");
			temp_normal_time = normal_time;
			intention_num = 3;
		}
		break;

	
	case 3: //使车子成66度倾斜，为横向移动找三分球做准备
		ROS_INFO_STREAM("case 333333333333333333333333333\n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(80))
				{
					temp_normal_time == normal_time;
					intention_num = 4;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-80))
				{
					temp_normal_time == normal_time;
					intention_num = 4;
				}
			}
		}
		break;

		/********************************* 第一次找球 ************************************/
	case 4:
			ROS_INFO_STREAM("case 4444444444444444444\n");
			the_object.whatObject = (objectType)2;//蓝灰色篮球
			object_pub(the_object);
			if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs((dsp_status.XDist) - 700) < 50)
					{
						Stop();
						ROS_INFO_STREAM("not find the ball let's go to inner to find！！！\n");
						temp_normal_time = normal_time;
						intention_num = 100;
					}
					else
					{
						ROS_INFO_STREAM("turn left with 80....\n");
						robotstraightright(); //继续右走
					}
				}
				if (2 == place_num)
				{
					if (abs((dsp_status.XDist) - 700) < 50)
					{
						Stop();
						ROS_INFO_STREAM("not find the ball let's go to inner to find！！！\n");
						temp_normal_time = normal_time;
						intention_num = 100;
					}
					else
					{
						ROS_INFO_STREAM("turn left with 80...\n");
						robotstraightleft(); //继续左走
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" ccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						ROS_INFO("lost ball lost ball lost ball ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" fffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						ROS_INFO_STREAM(" lost ball lost ball again ！！！\n");
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 4101;
						ROS_INFO_STREAM("catch the ball!!!!!! catch the ball ！！！\n");
					}
				}
			}
		
		break;
	
	case 4101:
		if (normal_time - temp_normal_time < 110)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 4102;
		}
		break;

	case 4102:
		if(stop())
		intention_num = 4103;
		break;

	case 4103:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 4104;
		break;

	case 4104:
		ROS_INFO_STREAM("case 4104 4104 4104 4104\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 4105; //抬机械臂命令已发送
		}
		break;

	case 4105: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 4105 4105 4105 4105\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 4106;
		}
		break;

	case 4106: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 4106 4106 4106 4106\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			tempptN.x = dsp_status.XDist; //把当前坐标赋给点N
			tempptN.y = dsp_status.YDist;
			temp_normal_time = normal_time;
			intention_num = 8201;
		}
		break;

	case 8201:
		ROS_INFO_STREAM("case 8201 8201 8201 8201 820\n");
		ROS_INFO_STREAM("are aiming to the posture Z ！！！！\n");
		if (ToAngleByMPU6050(ptB))
		{
			ROS_INFO_STREAM("have aimed the posture Z ！！！！\n");
			if (stop())
			{
				Stop();
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 8202;
			}
		}
		break;

	case 8202:
		ROS_INFO_STREAM("case 8202  8202\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid6(ptB)) 
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 820;
		}
		break;

	case 820:
		ROS_INFO_STREAM("case 820 820 820 820 820\n");
		ROS_INFO_STREAM("are aiming to the posture Z ！！！！\n");
		if (ToAngleByMPU6050(ptZ))
		{
			ROS_INFO_STREAM("have aimed the posture Z ！！！！\n");
			if (stop())
			{
				Stop();
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 83;
			}
		}
		break;

	case 83:
		ROS_INFO_STREAM("case 83 83 83 83 83\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid2(ptZ)) 
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		break;

	case 101: 
		ROS_INFO_STREAM("case 101 101 101 101 101 101 101 \n");
		ROS_INFO_STREAM("正在对准传球目标点！！！！\n");
		Control.SendData_state = 6; //关闭里程计
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("have aimed at the posture M！！！！\n");
			if (stop())
			{
				Control.SendData_state = 7; //旋转停稳之后，再打开里程计
				temp_normal_time = normal_time;
				intention_num = 102;
			}
		}
		break;

	case 102:
		if(stop())
		temp_normal_time = normal_time;
		intention_num = 1020;
		break;

	case 1020:
		Stop();
		intention_num = 1021;
		break;

	case 1021:
		ROS_INFO_STREAM("case 822 822 822 822\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 103;
		}
		break;

	case 103: //再转回来，对准内圈定位点
		ROS_INFO_STREAM("case 103 103 103 103 103 103 103\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0;
			if (ToAngleByMPU6050(ptB))
			{
				ROS_INFO_STREAM("have aimed to the inner ring！！！！\n");
				if (stop())
				{
					ROS_INFO_STREAM("appare to go strait！！！！\n");
					temp_normal_time = normal_time;
					intention_num = 1030;
				}
			}
		}
		break;

	case 1030:
		ROS_INFO_STREAM("case 1030 1030 1030 1030\n");
		if (ToPostureByMilemeter_Pid2(ptB)) 
		{
			Stop();
			ROS_INFO_STREAM("have arrived at NNNNNN\n");
			intention_num = 10301;
		}
		break;

	case 10301:
		ROS_INFO_STREAM("case 10301 10301\n");
		ROS_INFO_STREAM("are aiming to the posture Z ！！！！\n");
		if (ToAngleByMPU6050(tempptN))
		{
			ROS_INFO_STREAM("have aimed the posture Z ！！！！\n");
			if (stop())
			{
				Stop();
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 10302;
			}
		}
		break;

	case 10302:
		ROS_INFO_STREAM("case 10302 10302\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid6(tempptN)) 
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 1031;
		}
		break;

	case 1031: //再转回来，对准内圈定位点
		ROS_INFO_STREAM("case 1031 1031 1031 1031 1031 1031 1031\n");
		Control.SendData_state = 6; //关闭里程计
		if (ToAngleByMPU6050(ptT))
		{
			ROS_INFO_STREAM("have aimed to the inner ring！！！！\n");
			if (stop())
			{
				ROS_INFO_STREAM("appare to go strait！！！！\n");
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 1032;
			}
		}
		break;

	case 1032:
		ROS_INFO_STREAM("case 104 104 104 104 104 104 104 104\n");
		if (ToPostureByMilemeter_Pid5(ptT)) 
		{
			Stop();
			ROS_INFO_STREAM("have arrived at DDDDDDDDDDDDDD\n");
			intention_num = 1040;
		}
		break;

	case 1040:
		ROS_INFO_STREAM("case 1040 1040 1040 1040 1040 \n");
		if ((normal_time - temp_normal_time) > 5)
		{
			if (1 == place_num)
			{
				if (ToAngleByMPU(170))
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 105;
				}
			}
			if (2 == place_num)
			{
				if (ToAngleByMPU(-170))
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 105;
				}
			}
		}
		break;

		/***********************************  旋转识球  ***********************************/
	case 105:
		ROS_INFO_STREAM("case 105 105 105 105 105\n");
			the_object.whatObject = (objectType)2;//蓝灰色篮球
			object_pub(the_object);
			
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			if (1 == place_num)
			{
				if ((normal_time - temp_normal_time) < 3000)
				{
					ToFindObjectByturnleft(); //左旋找球测试（需要测试旋转找球时请将该函数释放）
					ROS_INFO_STREAM(" i'm turning left to find the ball\n");
				}
				else
				{
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 114;
				}
			}
			if (2 == place_num)
			{
				if ((normal_time - temp_normal_time) < 3000)
				{
					ToFindObjectByturnright(); //左旋找球测试（需要测试旋转找球时请将该函数释放）
					ROS_INFO_STREAM(" i'm turning left to find the ball\n");
				}
				else
				{
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 114;
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision are starting : %lf \n", objectAngleDistance_Y.first);
				ROS_INFO("intention_num : %lf \n", intention_num);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{
					findballflag = 0;
					aimballflag = 0;
					ROS_INFO("lost the ball : %lf \n", objectAngleDistance_Y.first);
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 105;
					}
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision are starting : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("lost the ball again \n");
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 105;
					}
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 1060;
					ROS_INFO_STREAM("catch the ball \n");
				}
			}
		}
		break;

	case 1060:
		if (normal_time - temp_normal_time < 110)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 1062;
		}
		break;

	case 1062:
		if(stop())
		intention_num = 1063;
		break;

	case 1063:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 1064;
		break;

	case 1064:
		ROS_INFO_STREAM("case 1064 1064 1064 1064\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 1065; //抬机械臂命令已发送
		}
		break;

	case 1065: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 1065 1065 1065 1065\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 114;
		}
		break;

	case 114:
		ROS_INFO_STREAM(" 114 114 114 114 114 114 114 114 114 114\n");
		Control.SendData_state = 6; //旋转时关闭里程计
		ROS_INFO_STREAM("are aiming to the posture Z ！！！！\n");
		if (ToAngleByMPU6050(ptT))
		{
			ROS_INFO_STREAM("have aimed the posture N ！！！！\n");
			if (stop())
			{
				Stop();
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 115;
			}
		}
		break;

	case 115:
		ROS_INFO_STREAM("case 115 115 115 115 115 115 115 115 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid3(ptT)) //到达N点
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 1150;
		}
		break;

	case 1150:
		ROS_INFO_STREAM(" 1150 1150 1150 1150\n");
		Control.SendData_state = 6; //旋转时关闭里程计
		ROS_INFO_STREAM("are aiming to the posture N！！！！\n");
		if (ToAngleByMPU6050(tempptN))
		{
			ROS_INFO_STREAM("have aimed the posture N ！！！！\n");
			if (stop())
			{
				Stop();
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 11501;
			}
		}
		break;

	case 11501:
		ROS_INFO_STREAM("case 11501 11501 11501 11501\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid3(tempptN)) //到达N点
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 11502;
		}
		break;

	case 11502:
		ROS_INFO_STREAM("case 8201 8201 8201 8201 820\n");
		ROS_INFO_STREAM("are aiming to the posture Z ！！！！\n");
		if (ToAngleByMPU6050(ptB))
		{
			ROS_INFO_STREAM("have aimed the posture Z ！！！！\n");
			if (stop())
			{
				Stop();
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 11503;
			}
		}
		break;

	case 11503:
		ROS_INFO_STREAM("case 8202  8202\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid6(ptB)) 
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 1151;
		}
		break;

	case 1151:
		ROS_INFO_STREAM(" 1151 1151 1151 1151 1151\n");
		Control.SendData_state = 6; //旋转时关闭里程计
		ROS_INFO_STREAM("are aiming to the posture Z ！！！！\n");
		if (ToAngleByMPU6050(ptZ))
		{
			ROS_INFO_STREAM("have aimed the posture Z ！！！！\n");
			if (stop())
			{
				Stop();
				Control.SendData_state = 7; //打开里程计
				temp_normal_time = normal_time;
				intention_num = 1152;
			}
		}
		break;

	case 1152:
		ROS_INFO_STREAM("case 1152 1152 1152 1152 1152 \n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid2(ptZ)) //到达N点
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 1153;
		}
		break;

	case 1153: //瞄准，准备发射篮球
		ROS_INFO_STREAM("case 1153 1153 1153 1153 \n");
		ROS_INFO_STREAM("are aiming the column！！！！\n");
		Control.SendData_state = 6;
		if (ToAngleByMPU6050(ptM))//ptM1
		{
			Control.SendData_state = 7;
			temp_normal_time = normal_time;
			intention_num = 1154;
		}
		break;

	case 1154:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 1155;
		}
		break;
	case 1155:
		Stop();
		intention_num = 1156;
		break;

	case 1156: //发射第er个篮球
		ROS_INFO_STREAM("case 1156 1156 1156 1156\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 118;
		}
		break;

	case 118: //**********车子归位开始**********
		ROS_INFO_STREAM(" 118 118 118 118 118\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0; //旋转时关闭里程计
			ROS_INFO_STREAM("are aiming to the posture Z ！！！！\n");
			if (ToAngleByMPU6050(ptOO))
			{
				ROS_INFO_STREAM("have aimed the posture Z ！！！！\n");
				if (stop())
				{
					Stop();
					temp_normal_time = normal_time;
					intention_num = 1180;
				}
			}
		}
		break;

	case 1180:
		ROS_INFO_STREAM("case 1180 1180 1180 1180 1180\n");
		if (ToPostureByMilemeter_Pid2(ptOO)) 
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 1181;
		}
		break;

	case 1181:
		if(stop())
		intention_num = 1182;
		break;

	case 1182:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 1183;
		break;

	case 1183:
		ROS_INFO_STREAM("case 1183 1183 1183 1183\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 8; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 1184; //抬机械臂命令已发送
		}
		break;

	case 1184: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 1184 1184 1184 1184\n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0; //单片机位置0
			temp_normal_time = normal_time;
			intention_num = 119;
		}
		break;

	case 119:
		ROS_INFO_STREAM("case 119 119 119 119 119\n");
		if (ToAngleByMPU6050(ptxy))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 120;
		}
		break;

	case 120:
		ROS_INFO_STREAM("case 120 120 120 120\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			if (ToPostureByMilemeter_Pid1(ptxy))
			{
				ROS_INFO_STREAM("stop stop stop stop stop  stop stop stop stop stop \n");
				Stop();
				intention_num = STOP;
			}
			
		}
		break;

	case STOP:
		ROS_INFO_STREAM("case 125 125 125 125 125 125\n");
		Stop();
		break;

	default:
		ROS_INFO_STREAM("case default default default default default default default \n");
		normal_time = 0;
		break; //**********车子归位结束**********
	}
}		


void DecisionMaking::Normal_bat1_new()
{
	//一些决策相关的标志
	static int intention_num = 0;	 //回合过程的意图顺序号
	static bool findballflag = 0;	 //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;	 //对准目标标志位，每对准一次目标后都要记得清零
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	static bool Iscircleavoidancecorner = 0; //中圈避障是否向边缘靠近了

	Posture ptA0, ptA, ptD, ptD2, ptD3; //ptZ是中圈有障碍球时临时确定的辅助点
	
	ptA0.x = 1500;
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptA.x = 2900; //第二个球扫描起始定位点
		ptA.y = 3600;//3600

		ptD.x = 3000; //3700
		ptD.y = 8800;

		ptD2.x = 3800;
		ptD2.y = 8800;//-8100
	}
	if (2 == place_num)
	{
		ptA.x = 2900; //第二个球扫描起始定位点
		ptA.y = -3600;//3600

		ptD.x = 3000; //3700
		ptD.y = -8800;//-8100

		ptD2.x = 3800;
		ptD2.y = -8800;//-8100
	}
	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;

	case 120:
		if ((normal_time - temp_normal_time) > 20) //延时17.1s
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		break;

	case 101:
		ROS_INFO_STREAM("case 101 101 101 101\n");
		if (ToAngleByMPU6050(ptD))
		{
			if (stop())
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 102;
			}
		}
		break;

	case 102:
		ROS_INFO_STREAM("case 102 102 102 102\n");
		if (ToPostureByMilemeter_Pid6(ptD))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 201;
		}
		break;

	case 201:
		ROS_INFO_STREAM("case 201 201 201 201 201 201\n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(45))
				{
					temp_normal_time == normal_time;
					intention_num = 2;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-45))
				{
					temp_normal_time == normal_time;
					intention_num = 2;
				}
			}
		}
		break;

		/********************************* 第一次识别标定柱 *********************************/
	case 2:
		ROS_INFO_STREAM("case 2222222222222222222\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2500)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 201;
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 201;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 3001;
				}
			}
		}
		break;

	case 3001:
		if(1 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnright();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 301;
			}
			
		}
		if(2 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnleft();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 301;
			}
			
		}
		break;

	case 301:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 302;
		}
		break;

	case 302:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 303;
		break;

	case 303: 
		ROS_INFO_STREAM("case 303 303 303\n");
		if ((normal_time - temp_normal_time) > 16)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 501;
		}
		break;

	case 501: 
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 5;
		}
		break;

	case 5: //准备到达M点开始扫描中线置球
		ROS_INFO_STREAM("case 555555555555\n");
		if (ToAngleByMPU6050(ptA))
		{
			ROS_INFO_STREAM("have aimed to the posture A！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 6;
			}
		}
		break;

	case 6:
		ROS_INFO_STREAM("case 6666666666666\n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid6(ptA))
		{
			if(stop())
			Stop();
			ROS_INFO_STREAM("have arrived at the posture A\n");
			intention_num = 7;
		}
		break;

	case 7: //使车子成0度，为横向移动找中线球做准备
		ROS_INFO_STREAM("case 77777777777\n");
			if (ToAngleByMPU(2))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 8;
				}
			}
		break;

		/***********************************  中线识球开始  ***********************************/
	case 8:
		ROS_INFO_STREAM("case 20 20 20 20 20 20 20 20 20 \n");
		the_object.whatObject = (objectType)3;//排球
		object_pub(the_object);
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
				{
					if ((dsp_status.YDist + 700) < 50)
					{
						Stop();
						temp_normal_time = normal_time;
						intention_num = 5;
					}
					else
					{
						ROS_INFO_STREAM("turn left to find ball...\n");
						robotstraightleft(); //继续走
					}
				}
				if (2 == place_num)
				{
					if ((dsp_status.YDist - 700) > 50)
					{
						Stop();
						temp_normal_time = normal_time;
						intention_num = 5;
					}
					else
					{
						ROS_INFO_STREAM("turn right to find ball..\n");
						robotstraightright(); //继续走
					}
				}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision are starting : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					ROS_INFO("lost the ball: %lf \n", objectAngleDistance_Y.first);
					// if ((normal_time - temp_normal_time) > 2)
					// {
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 8;
					// }
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision are starting: %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("lost the ball again \n");
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 8;
					}
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 9001;
					ROS_INFO_STREAM("catch the ball \n");
				}
			}
		}
		break;

	case 9001:
		if (normal_time - temp_normal_time < 83)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 9002;
		}
		break;
	
	case 9002:
		if(stop())
		temp_normal_time = normal_time;
		intention_num = 9003;
		break;

	case 9003:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 9;
		break;
	
	case 9:
		ROS_INFO_STREAM("case 99999999999999999\n");
		if ((normal_time - temp_normal_time) > 13)
		Stop();
		Control.SendData_state = 1; //1     //球已持住，发送抬机械臂命令，下一步立即置位0
		temp_normal_time = normal_time;
		intention_num = 10; //抬机械臂命令已发送
		break;

	case 10: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 10 10 10 10 10 10 10 10 \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0; //单片机位置0
			temp_normal_time = normal_time;
			intention_num = 160;
		}
		break;

	case 160:
		if (normal_time - temp_normal_time < 150)
		{
			robotback();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 16;
		}
		break;

	case 16:
		ROS_INFO_STREAM("case 16 16 16 16 16 16 \n");
		if (ToAngleByMPU6050(ptD))
		{
			if (stop())
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 161;
			}
		}
		break;

	case 161:
		ROS_INFO_STREAM("case 161 161 161 161\n");
		if (ToPostureByMilemeter_Pid6(ptD))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 172;
		}
		break;

	case 172:
		ROS_INFO_STREAM("case 172 172 172 172 172 172 \n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(45))
				{
					temp_normal_time == normal_time;
					intention_num = 18;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-45))
				{
					temp_normal_time == normal_time;
					intention_num = 18;
				}
			}
		}
		break;
		/************************** 第二次找标定柱 **************************/
	case 18:
		ROS_INFO_STREAM("case 18 18 18 18 18 18 18 18 18 \n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
        if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2000)
			{
				ROS_INFO("placenum = ",place_num);
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 172;
			}
			
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 172;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 1999;
				}
			}
		}
		break;

	case 1999:
		if(1 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnright();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 1901;
			}
			
		}
		if(2 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnleft();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 1901;
			}	
		}
		break;

	case 1901:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 1902;
		}
		break;
		
	case 1902:
		Stop();
		intention_num = 1903;
		break;

	case 1903: 
		ROS_INFO_STREAM("case 303 303 303\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 20;
		}
		break;

	case 20:
		ROS_INFO_STREAM("case 20 20 20 20 20\n");
		if (normal_time - temp_normal_time > 3)
		{
			Control.SendData_state = 0; //单片机位置0
			if (stop())
			{
				Stop();
			}
		}
		break;

	default:
		normal_time = 0;
		break;
	}
}


void DecisionMaking::Normal_bat2_new()
{									 //一些决策相关的标志
	static int intention_num = 0;	 //回合过程的意图顺序号
	static bool findballflag = 0;	 //找到目标标志位,每找完一次目标后都要记得清零
	static bool aimballflag = 0;	 //对准目标标志位，每对准一次目标后都要记得清零
	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;					 //DSP 45ms发送一次数据

	Posture ptA0, ptA, ptD, ptM, ptD3;
	static Posture ptN = {0, 0}; // B1-N

	ptA0.x = 1500; //出发前，一个定点
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptA.x = 3800; //中点，扫描起点
		ptA.y = 4500;

		ptD.x = 3000;
		ptD.y = 8800; //投球点,正中

		ptM.x = 2700;
		ptM.y = 3500; //中圈置球开始扫描点

		ptD3.x = 4700;
		ptD3.y = 9000;
	}

	if (2 == place_num)
	{
		ptA.x = 3800; //中点，扫描起点
		ptA.y = -4500;

		ptD.x = 3000;
		ptD.y = -8800; //投球点B2-D//-8100

		ptM.x = 2700;
		ptM.y = -3500; //中圈置球开始扫描点(done)

		ptD3.x = 3200;
		ptD3.y = -8700;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 0000000000000000000\n");
		if ((normal_time - temp_normal_time) > 20) //延时大约9s
		{
			temp_normal_time = normal_time;
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222\n");
		if (ToAngleByMPU6050(ptA))
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 201;
			}
		}
		break;

	case 201:
		ROS_INFO_STREAM("case 201 201 201 201 201 201 201\n");
		if (ToPostureByMilemeter_Pid2(ptA))
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 3;
			}
		}
		break;

	case 3: //使车子成66度倾斜，为横向移动找三分球做准备
		ROS_INFO_STREAM("case 333333333333333333333333333\n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(80))
				{
					temp_normal_time == normal_time;
					intention_num = 5;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-80))
				{
					temp_normal_time == normal_time;
					intention_num = 5;
				}
			}
		}
		break;
		/************************** 三分线找球 *************************/
	case 5:
		ROS_INFO_STREAM("case  5 5 5 5 5 5 5 5 5 5 5\n");
		the_object.whatObject = (objectType)3;//蓝灰色篮球
			object_pub(the_object);
			if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
			{
				findballflag = 0;
				aimballflag = 0;
				if (1 == place_num)
				{
					if (abs((dsp_status.XDist) - 700) < 50)
					{
						Stop();
						ROS_INFO_STREAM("not find the ball let's go to inner to find！！！\n");
						temp_normal_time = normal_time;
						intention_num = 2;
					}
					else
					{
						ROS_INFO_STREAM("turn left with 80....\n");
						robotstraightright(); //继续右走
					}
				}
				if (2 == place_num)
				{
					if (abs((dsp_status.XDist) - 700) < 50)
					{
						Stop();
						ROS_INFO_STREAM("not find the ball let's go to inner to find！！！\n");
						temp_normal_time = normal_time;
						intention_num = 2;
					}
					else
					{
						ROS_INFO_STREAM("turn left with 80...\n");
						robotstraightleft(); //继续左走
					}
				}
			}
			else
			{
				findballflag = 1;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" ccccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						ROS_INFO("lost ball lost ball lost ball ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 5;
						}
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" fffffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						ROS_INFO_STREAM(" lost ball lost ball again ！！！\n");
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 5;
						}
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 601;
						ROS_INFO_STREAM("catch the ball!!!!!! catch the ball ！！！\n");
					}
				}
			}
		break;

	case 601:
		if (normal_time - temp_normal_time < 97)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 602;
		}
		break;
	case 602:
		if(stop())
		intention_num = 603;
		break;

	case 603:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 6;
		break;
	
	case 6:
		ROS_INFO_STREAM("case 66666666666666666\n");
		if ((normal_time - temp_normal_time) > 20)
		{
		Control.SendData_state = 1; //1     //球已持住，发送抬机械臂命令，下一步立即置位0
		temp_normal_time = normal_time;
		intention_num = 7; //抬机械臂命令已发送
		}
		break;

	case 7: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 7 7 7 7 7 7 7 7 7 7 7 7 \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0; 
			//单片机位置0
			ptN.x = dsp_status.XDist; //把当前坐标赋给点N
			ptN.y = dsp_status.YDist;
			ROS_INFO("the posture of N is：%ld , %ld\n", ptN.x, ptN.y);
			temp_normal_time = normal_time;
			intention_num = 11;
		}
		break;

	case 11: //准备走到投球点
		ROS_INFO_STREAM("case 11 11 11 11 11 11 11 11 \n");
		ROS_INFO_STREAM("are aiming to the posture D！！！\n");
		if (ToAngleByMPU6050(ptD))
		{
			ROS_INFO_STREAM("have aimed to the posture D！！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 12;
			}
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12 12\n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid6(ptD)) //到达D点
		{
			Stop();
			ROS_INFO_STREAM("have arrived at the posture D\n");
			intention_num = 13;
		}
		break;

	case 13:
		ROS_INFO_STREAM("case 13 13 13 13 13\n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(45))
				{
					temp_normal_time == normal_time;
					intention_num = 14;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-45))
				{
					temp_normal_time == normal_time;
					intention_num = 14;
				}
			}
		}
		break;
		/***************************** 第一次找标定柱 ******************************/
	case 14:
		ROS_INFO_STREAM("case 14 14 14 14 14 14 14 14\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			ROS_INFO("placenum= ",place_num);
			if((normal_time - temp_normal_time) < 2000)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 330;
			}
			
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 14;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 1500;
				}
			}
		}
		break;

	case 1500:
		if(1 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnright();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 1501;
			}
			
		}
		if(2 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnleft();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 1501;
			}
			
		}
		break;

	case 1501:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 1502;
		}
		break;

	case 1502:
		Stop();
		intention_num = 15;
		break;

	case 15: //发射第一个篮球
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15\n");
		if ((normal_time - temp_normal_time) > 18)
		{
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 16;
		}
		break;

	case 16: //准备返回N点
		ROS_INFO_STREAM("case 16 16 16 16 16 16 16\n");
	    if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0;
			if (ToAngleByMPU6050(ptN))
			{
				ROS_INFO_STREAM("have aimed to the posture N！！\n");
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 17;
				}
			}
		}
		break;

	case 17:
		ROS_INFO_STREAM("case 17 17 17 17 17 17 \n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid6(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("have arrived at the posture N\n");
			intention_num = 18;
		}
		break;

	case 18: //准备到达M点开始扫描中线置球
		ROS_INFO_STREAM("case 18 18 18 18 18 18 \n");
		if (ToAngleByMPU6050(ptM))
		{
			ROS_INFO_STREAM("have aimed to the posture M！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 19;
			}
		}
		break;

	case 19:
		ROS_INFO_STREAM("case 19 19 19 19 19 19 \n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid6(ptM)) //到达M点
		{
			if(stop())
			Stop();
			ROS_INFO_STREAM("have arrived at the posture M\n");
			intention_num = 190;
		}
		break;

	case 190: //使车子成0度，为横向移动找中线球做准备
		ROS_INFO_STREAM("case 190 190 190 190 190 1\n");
			if (ToAngleByMPU(0))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 20;
				}
			}
		break;

		/***********************************  中线识球开始  ***********************************/
	case 20:
		ROS_INFO_STREAM("case 20 20 20 20 20 20 20 20 20 \n");
		temp_normal_time = normal_time; //为延时作准备
		the_object.whatObject = (objectType)3;//排球
		object_pub(the_object);
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			// {
			// 	temp_normal_time = normal_time;
			// 	if(ToAngleByMPU(-70))
			// 	{
			// 		 Stop();
			// 		 if ((normal_time - temp_normal_time) < 1000)
			// 		 {
			// 			 ToFindObjectByturnright();
			// 		 }
			// 	}
			// }
				{
					if ((dsp_status.YDist + 700) < 50)
					{
						Stop();
						ROS_INFO_STREAM("have arrive at the boundary but not find the ball！！\n");
						temp_normal_time = normal_time;
						intention_num= 18;
					}
					else
					{
						ROS_INFO_STREAM("turn left to find ball...\n");
						robotstraightleft(); //继续走
					}
				}
				if (2 == place_num)
				{
					if ((dsp_status.YDist - 700) > 50)
					{
						Stop();
						ROS_INFO_STREAM("have arrive at the boundary but not find the ball！！\n");
						temp_normal_time = normal_time;
						intention_num = 18;
					}
					else
					{
						ROS_INFO_STREAM("turn right to find ball..\n");
						robotstraightright(); //继续走
					}
				}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision are starting : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					ROS_INFO("lost the ball: %lf \n", objectAngleDistance_Y.first);
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 20;
					}
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision are starting: %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("lost the ball again \n");
					if ((normal_time - temp_normal_time) > 2)
					{
						temp_normal_time = normal_time; //为延时作准备
						intention_num = 20;
					}
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 210;
					ROS_INFO_STREAM("catch the ball \n");
				}
			}
		}
		break;

	case 210:
		if (normal_time - temp_normal_time < 93)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 211;
		}
		break;
	case 211:
		if(stop())
		intention_num = 212;
		break;

	case 212:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 22;
		break;
	
	case 22:
		ROS_INFO_STREAM("case 22222222222\n");
		if ((normal_time - temp_normal_time) > 20)
		{
		Stop();
		Control.SendData_state = 1; //1     //球已持住，发送抬机械臂命令，下一步立即置位0
		temp_normal_time = normal_time;
		intention_num = 23; //抬机械臂命令已发送
		}
		break;

	case 23: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 23 23 23 23 23\n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0; 
			temp_normal_time = normal_time;
			intention_num = 252;
		}
		break;

	case 252:
	    ROS_INFO_STREAM("case 252 252 252 252 252\n");
		if (normal_time - temp_normal_time < 200)
		{
			robotback();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 26;
		}
		break;

	case 26: //准备返回B1点
		ROS_INFO_STREAM("case 26 26 26 26 26 26 26\n");
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptN))
		{
			ROS_INFO_STREAM("have aimed to the posture N！！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 27;
			}
		}
		break;

	case 27:
		ROS_INFO_STREAM("case 27 27 27 27 27 27 27\n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid6(ptN)) //到达N点
		{
			Stop();
			ROS_INFO_STREAM("have arrived at the posture N\n");
			intention_num = 28;
		}
		break;

	case 28: //准备到达投球点D
		ROS_INFO_STREAM("case 28  28  28  28  28  28  28  28\n");
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptD))
		{
			ROS_INFO_STREAM("have aimed to the posture D2！！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 29;
			}
		}
		break;

	case 29:
		ROS_INFO_STREAM("case 29 29 29 29 29 29 29 \n");
		temp_normal_time = normal_time;
		if (ToPostureByMilemeter_Pid6(ptD)) //到达D点
		{
			Stop();
			ROS_INFO_STREAM("have arrived at the posture D2\n");
			intention_num = 30;
		}
		break;

	case 30: //使车子成0度，为横向移动找中线球做准备
		ROS_INFO_STREAM("case 30 30 30 30 30 30 30\n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(45))
				{
					temp_normal_time == normal_time;
					intention_num = 31;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-45))
				{
					temp_normal_time == normal_time;
					intention_num = 31;
				}
			}
		}
		break;

		/********************************* 第二次识别标定柱 *********************************/
	case 31:
		ROS_INFO_STREAM("case 31 31 31 31 31 31 31 31 3\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			ROS_INFO("placenum= ",place_num);
			if((normal_time - temp_normal_time) < 2000)
			{
				if (1 == place_num)
				{
					ROS_INFO("turn right");
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ROS_INFO("turn left");
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 330;
			}
			
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 330;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 3200;
				}
			}
		}
		break;

	case 3200:
		if(1 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnright();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 320;
			}
			
		}
		if(2 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnleft();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 320;
			}
			
		}
		break;

	case 320:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 321;
		}
		break;
	case 321:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 32;
		break;

	case 32: //准备发射排球
		ROS_INFO_STREAM("case 32 32 32 32 32 32 32 32 32\n");
		if ((normal_time - temp_normal_time) > 18)
		{
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching！！\n");
			temp_normal_time = normal_time;
			intention_num = 33;
		}
		break;

	case 330:
		ROS_INFO_STREAM("case 200 200 200 200\n");
		Control.SendData_state = 0;
	    if ((normal_time - temp_normal_time) > 20)
		if (ToAngleByMPU6050(ptD3))
		{
			if (stop())
			{
				ROS_INFO_STREAM("have aimed to the posture D2！！ \n");
				temp_normal_time = normal_time;
				intention_num = 320;
			}
		}
		break;	

	case 33: //停车，比赛结束！！！！！！！！！！！
		ROS_INFO_STREAM("case 33 33 33 33 33 33 \n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0;
			Stop(); //停车
		}
		break;

	default:
		normal_time = 0;
		break;
	}
}

void DecisionMaking::Normal_bat3_new()
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;
	static bool Isthreescoreunnormal = 0; //捡三分球是否出现异常

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	Posture ptA0, ptA,  ptD, ptD3;
	static Posture ptN = {0, 0};

	ptA0.x = 1500;
	ptA0.y = 0;

	if (1 == place_num)
	{
		ptA.x = 3800; //中点，扫描起点
		ptA.y = 4000;

		ptD.x = 3000; //投篮正中定位点（激光距离篮筐2400）（钢板400计，激光超出30计）
		ptD.y = 8800;

		ptD3.x = 3700;
		ptD3.y = 9200;

	}
	if (2 == place_num)
	{
		ptA.x = 3800; //中点，扫描起点
		ptA.y = -4000;//-4800

		ptD.x = 3000; 
		ptD.y = -8800;//-8100

		ptD3.x = 3700;
		ptD3.y = -9200;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000\n");
		temp_normal_time = normal_time; //为延时作准备
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 20) 
		{
			temp_normal_time = normal_time; //为延时作准备
			intention_num = 1;
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 1111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA0))
		{
			Stop();
			ROS_INFO_STREAM("have arrived at the posture A0！！！\n");
			temp_normal_time = normal_time;
			intention_num = 2;
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222\n");
		if (ToAngleByMPU6050(ptA))
		{
			if (stop())
			{
				ROS_INFO_STREAM("have aimed the posture A！！！\n");
				temp_normal_time = normal_time;
				intention_num = 201;
			}
		}
		break;

	case 201:
		ROS_INFO_STREAM("case 201 201 201 201 201\n");
		if (ToPostureByMilemeter_Pid6(ptA))
		{
			if (stop())
			{
				ROS_INFO_STREAM("have arrived at the posture A！！！\n");
				temp_normal_time = normal_time;
				intention_num = 3;
			}
		}
		break;

	case 3: 
		ROS_INFO_STREAM("case 333333333333333333333333333\n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(85))
				{
					temp_normal_time == normal_time;
					intention_num = 4;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-85))
				{
					temp_normal_time == normal_time;
					intention_num = 4;
				}
			}
		}
		break;

		/***************************** 第一次识别球 *******************************/
	case 4:
		ROS_INFO_STREAM("case 44444444444444444444444\n");
		if ((normal_time - temp_normal_time) > 5)
		{
			the_object.whatObject = (objectType)3;//排球
			object_pub(the_object);
			if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
			{
				findballflag = 0;
				aimballflag = 0;
			if (1 == place_num)
			{
				if (abs((dsp_status.XDist) - 600) < 50)
				{
					Stop();
					ROS_INFO_STREAM("arrive at the boundary but not fing ball！\n");
					temp_normal_time = normal_time;
					intention_num = 2; 
				}
				else
				{
					ROS_INFO_STREAM("turn left to find the ball.\n");
					robotstraightright(); //左移找球
				}
			}
			if (2 == place_num)
			{
				if (abs((dsp_status.XDist) - 600) < 50)
				{
					Stop();
					ROS_INFO_STREAM("arrive at the boundary but not fing ball！\n");
					temp_normal_time = normal_time;
					intention_num = 2; 
				}
				else
				{
					ROS_INFO_STREAM("turn right to find the ball..\n");
					robotstraightleft(); //右移找球
				}
			}
			}
			else
			{
				findballflag = 1;
				Isthreescoreunnormal = 0;
				ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
			}
			if ((findballflag == 1) && (aimballflag == 0))
			{
				ROS_INFO_STREAM(" ccccccccccccccccccccccccccc\n");
				if (ToAimBallByVision())
				{
					ROS_INFO("ToAimBallByVision are starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //对准球的过程中突然丢了
						findballflag = 0;
						aimballflag = 0;
						ROS_INFO("lost the ball！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 1;
						ROS_INFO("!objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					}
				}
			}
			if ((findballflag == 1) && (aimballflag == 1))
			{
				ROS_INFO_STREAM(" fffffffffffffffffffffffffff\n");
				if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
				{
					ROS_INFO("ToBallByVision are starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
					if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
					{ //前往球的跟前过程中，又没看到球，重新去找
						aimballflag = 0;
						findballflag = 0;
						ROS_INFO_STREAM(" lost the ball again ！！！\n");
						if ((normal_time - temp_normal_time) > 2)
						{
							temp_normal_time = normal_time; //为延时作准备
							intention_num = 4;
						}
					}
					else
					{
						aimballflag = 0;
						findballflag = 0;
						temp_normal_time = normal_time;
						intention_num = 51;
						ROS_INFO_STREAM("catch the ball！！！\n");
					}
				}
			}
		}
		break;

	case 51:
		if (normal_time - temp_normal_time < 85)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 501;
		}
		break;

	case 501:
		if(stop())
		intention_num = 502;
		break;

	case 502:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 5;
		break;

	case 5:
		ROS_INFO_STREAM("case 555555555555555555555\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
			temp_normal_time = normal_time;
			intention_num = 6; //抬机械臂命令已发送
		}
		break;

	case 6: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 6666666666666666666666\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 10)
		{
			temp_normal_time = normal_time;
			intention_num = 9;
		}
		break;

	case 9:
		ROS_INFO_STREAM("case 999999999999999\n");
		if (ToPostureByAvoidance(ptD)) //避障
		{
			temp_normal_time = normal_time;
			intention_num = 901;
		}
		break;

	case 901:
		ROS_INFO_STREAM("case 901 901 901 901 901 901\n");
		if ((normal_time - temp_normal_time) < 300)
		{
			robotforwardlow();
		}
		else
		{
			temp_normal_time = normal_time;
			intention_num = 902;
		}
		break;

	case 902: //避障后再次对准目标点
		ROS_INFO_STREAM("case 902 902 902 902 902 902\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptD))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 903;
				}
			}
		}
		break;

	case 903:
		ROS_INFO_STREAM("case 903 903 903 903 903 903 903\n");
		if (ToPostureByMilemeter_Pid2(ptD))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 10;
		}
		break;

	case 10: //转向目的点，去找标定柱
		ROS_INFO_STREAM("case 10 10 10 10 10 10\n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(45))
				{
					temp_normal_time == normal_time;
					intention_num = 13;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-45))
				{
					temp_normal_time == normal_time;
					intention_num = 13;
				}
			}
		}
		break;

		/********************************* 第一次识别标定柱 *********************************/
	case 13:
		ROS_INFO_STREAM("case 13 13 13 13 13 13 13 13 13 13 13 \n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2500)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 10;
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 13;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 1401;
				}
			}
		}
		break;

	case 1401:
		if(1 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnright();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 140;
			}
			
		}
		if(2 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnleft();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 140;
			}
			
		}
		break;


	case 140:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 141;
		}
		break;

	case 141:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 14;
		break;

	case 14: //发射第一个篮球
		ROS_INFO_STREAM("case 14 14 14 14 14 14\n");
		if ((normal_time - temp_normal_time) > 22)
		{
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 1500;
		}
		break;

	case 1500:
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 1501;
		}
		break;

	case 1501:
		ROS_INFO_STREAM("case 1501 1501 1501\n");
		if (ToAngleByMPU6050(ptD3))
		{
			if (stop())
			{
				ROS_INFO_STREAM("have aimed the posture A！！！\n");
				temp_normal_time = normal_time;
				intention_num = 1502;
			}
		}
		break;

	case 1502:
		ROS_INFO_STREAM("case 1502 1502 1502\n");
		if (ToPostureByMilemeter_Pid3(ptD3))
		{
			if (stop())
			{
				ROS_INFO_STREAM("have arrived at the posture A！！！\n");
				temp_normal_time = normal_time;
				intention_num = 15;
			}
		}
		break;

	case 15:
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15\n");
		if ((normal_time - temp_normal_time) > 20)
		if (1 == place_num)
		{
			if (ToAngleByMPU(-90))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 16;
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(90))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 16;
			}
		}
		break;

		/************************************ 第二次找球 ***********************************/
	case 16:
		ROS_INFO_STREAM("case 16 16 16 16 16 16 16 16 16\n");
		temp_normal_time = normal_time; //为延时作准备
		the_object.whatObject = (objectType)3;//排球
		object_pub(the_object);
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			aimballflag = 0;
			findballflag = 0;
			if (1 == place_num)
			{
				if (abs((dsp_status.XDist) - (-800)) < 50)
				{
					robotstraightright();
					ROS_INFO_STREAM("arrive at the boundary but not fing ball！\n");
					temp_normal_time = normal_time;
					intention_num = 1501;
				}
				else
				{
					ROS_INFO_STREAM("turn left to find the ball.\n");
					robotstraightleft(); //左移找球
				}
			}
			if (2 == place_num)
			{
				if (abs((dsp_status.XDist) - 800) < 50)
				{
					robotstraightleft();
					ROS_INFO_STREAM("arrive at the boundary but not fing ball！\n");
					temp_normal_time = normal_time;
					intention_num = 1501; 
				}
				else
				{
					ROS_INFO_STREAM("turn right to find the ball..\n");
					robotstraightright(); //右移找球
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 16;
					ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					intention_num = 16;
					ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					intention_num = 1701;
					ROS_INFO_STREAM("捡球 捡球 捡球 \n");
				}
			}
		}
		break;

case 1701:
		if (normal_time - temp_normal_time < 85)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 1702;
		}
		break;

	case 1702:
		if(stop())
		intention_num = 1703;
		break;

	case 1703:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 17;
		break;

	case 17:
		if ((normal_time - temp_normal_time) > 20)
		{
		ROS_INFO_STREAM("case 17 17 17 17 17 17\n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		temp_normal_time = normal_time;
		intention_num = 18; //抬机械臂命令已发送
		}
		break;

	case 18: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 18 18 18 18 18 18 18\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 20)
		{
			temp_normal_time = normal_time;
			intention_num = 240;
		}
		break;

	case 240:
		if (normal_time - temp_normal_time < 150)
		{
			robotback();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 24;
		}
		break;

	case 24:
		ROS_INFO_STREAM("case 24 24 24 24 24 24 24 \n");
		ROS_INFO_STREAM("are aiming to the posture D！！！\n");
		if (ToAngleByMPU6050(ptD))
		{
			ROS_INFO_STREAM("have aimed to the posture D！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 25;
			}
		}
		break;

	case 25:
		ROS_INFO_STREAM("case 25 25 25 25 25 25 25 25\n");
		if (ToPostureByMilemeter_Pid3(ptD))
		{
			Stop();
			ROS_INFO_STREAM("have arrived at the posture D1！！n");
			temp_normal_time = normal_time;
			intention_num = 26;
		}
		break;

	case 26:
		ROS_INFO_STREAM("case 26 26 26 26 26 26 26 26 26 \n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(45))
				{
					temp_normal_time == normal_time;
					intention_num = 27;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-45))
				{
					temp_normal_time == normal_time;
					intention_num = 27;
				}
			}
		}
		break;

		/***************************** 第二次找标定柱 *******************************/
	case 27:
		ROS_INFO_STREAM("case 27 27 27 27 27 27 27 27 27\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2500)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 27;
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 27;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90) < 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("pole The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 2801;
				}
			}
		}
		break;

	case 2801:
		if(1 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnright();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 280;
			}
			
		}
		if(2 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnleft();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 280;
			}
		}
		break;


	case 280:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 281;
		}
		break;

	case 281:
		Stop();
		intention_num = 28;
		break;

	case 28: 
		ROS_INFO_STREAM("case 28 28 28 28 28 28\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 2910;
		}
		break;

	case 2910:
		ROS_INFO_STREAM("case 1501 1501 1501\n");
		if ((normal_time - temp_normal_time) > 3)
		{
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptD3))
		{
			if (stop())
			{
				ROS_INFO_STREAM("have aimed the posture A！！！\n");
				temp_normal_time = normal_time;
				intention_num = 2911;
			}
		}
		}
		break;

	case 2911:
		ROS_INFO_STREAM("case 1502 1502 1502\n");
		if (ToPostureByMilemeter_Pid3(ptD3))
		{
			if (stop())
			{
				ROS_INFO_STREAM("have arrived at the posture A！！！\n");
				temp_normal_time = normal_time;
				intention_num = 2912;
			}
		}
		break;

	case 2912:
		ROS_INFO_STREAM("case 15 15 15 15 15 15 15 15\n");
		if ((normal_time - temp_normal_time) > 20)
		if (1 == place_num)
		{
			if (ToAngleByMPU(-90))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 2913;
			}
		}
		if (2 == place_num)
		{
			if (ToAngleByMPU(90))
			{
				Stop();
				temp_normal_time = normal_time;
				intention_num = 2913;
			}
		}
		break;

		/************************************ 第二次找球 ***********************************/
	case 2913:
		ROS_INFO_STREAM("case 16 16 16 16 16 16 16 16 16\n");
		temp_normal_time = normal_time; //为延时作准备
		the_object.whatObject = (objectType)3;//排球
		object_pub(the_object);
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			aimballflag = 0;
			findballflag = 0;
			if (1 == place_num)
			{
				if (abs((dsp_status.XDist) - (-800)) < 50)
				{
					robotstraightright();
					ROS_INFO_STREAM("arrive at the boundary but not fing ball！\n");
					temp_normal_time = normal_time;
					intention_num = 2910;
				}
				else
				{
					ROS_INFO_STREAM("turn left to find the ball.\n");
					robotstraightleft(); //左移找球
				}
			}
			if (2 == place_num)
			{
				if (abs((dsp_status.XDist) - 800) < 50)
				{
					robotstraightleft();
					ROS_INFO_STREAM("arrive at the boundary but not fing ball！\n");
					temp_normal_time = normal_time;
					intention_num = 2910; 
				}
				else
				{
					ROS_INFO_STREAM("turn right to find the ball..\n");
					robotstraightright(); //右移找球
				}
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM(" findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 2913;
					ROS_INFO("丢了 丢了 丢了 : %lf \n", objectAngleDistance_Y.first);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("!!!!!!!!中圈球:objectAngle aimedaimedaimed is %lf;中圈球:distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1))
		{
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision 跳进来了 跳进来了 跳进来了 : %lf \n", objectAngleDistance_Y.first);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					intention_num = 2913;
					ROS_INFO_STREAM("又丢了 又丢了 又丢了 \n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					intention_num = 2914;
					ROS_INFO_STREAM("捡球 捡球 捡球 \n");
				}
			}
		}
		break;

case 2914:
		if (normal_time - temp_normal_time < 85)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 2915;
		}
		break;

	case 2915:
		if(stop())
		intention_num = 2916;
		break;

	case 2916:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 2917;
		break;

	case 2917:
		if ((normal_time - temp_normal_time) > 20)
		{
		ROS_INFO_STREAM("case 17 17 17 17 17 17\n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		temp_normal_time = normal_time;
		intention_num = 2918; //抬机械臂命令已发送
		}
		break;

	case 2918: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 18 18 18 18 18 18 18\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 20)
		{
			temp_normal_time = normal_time;
			intention_num = 2919;
		}
		break;

	case 2919:
		if (normal_time - temp_normal_time < 150)
		{
			robotback();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2920;
		}
		break;

	case 2920:
		ROS_INFO_STREAM("case 24 24 24 24 24 24 24 \n");
		ROS_INFO_STREAM("are aiming to the posture D！！！\n");
		if (ToAngleByMPU6050(ptD))
		{
			ROS_INFO_STREAM("have aimed to the posture D！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 2923;
			}
		}
		break;

	case 2923:
		ROS_INFO_STREAM("case 25 25 25 25 25 25 25 25\n");
		if (ToPostureByMilemeter_Pid3(ptD))
		{
			Stop();
			ROS_INFO_STREAM("have arrived at the posture D1！！n");
			temp_normal_time = normal_time;
			intention_num = 2924;
		}
		break;

	case 2924:
		ROS_INFO_STREAM("case 26 26 26 26 26 26 26 26 26 \n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(45))
				{
					temp_normal_time == normal_time;
					intention_num = 2925;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-45))
				{
					temp_normal_time == normal_time;
					intention_num = 2925;
				}
			}
		}
		break;

		/***************************** 第二次找标定柱 *******************************/
	case 2925:
		ROS_INFO_STREAM("case 27 27 27 27 27 27 27 27 27\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2500)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 2925;
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((00.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 2925;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90) < 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					stop();
					aimballflag = 0;
					findballflag = 0;
					ROS_INFO_STREAM("pole The distance has been to the min error;The distance has been to the min error!!!!!!!!\n");
					temp_normal_time = normal_time;
					intention_num = 2933;
				}
			}
		}
		break;

	case 2933:
		if(1 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnright();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 2926;
			}
			
		}
		if(2 == place_num)
		{
			if ((normal_time - temp_normal_time) < 15)
			{
				ToFindObjectByturnleft();
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 2926;
			}
		}
		break;

	case 2926:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 2927;
		}
		break;

	case 2927:
		Stop();
		intention_num = 2928;
		break;

	case 2928: 
		ROS_INFO_STREAM("case 28 28 28 28 28 28\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 29;
		}
		break;

	case 300:
		ROS_INFO_STREAM("case 300 300 300 300\n");
		Control.SendData_state = 0;
	    if ((normal_time - temp_normal_time) > 20)
		if (ToAngleByMPU6050(ptD3))
		{
			if (stop())
			{
				ROS_INFO_STREAM("have aimed to the posture D2！！ \n");
				temp_normal_time = normal_time;
				intention_num = 280;
			}
		}
		break;	

	case 29:
		ROS_INFO_STREAM("case 29 29 29 29 29 29 29 29\n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0;
			Stop();
		}
		break;

	default:
		break;
	}
}

void DecisionMaking::challenge_competition()
{
	//一些决策相关的标志
	static int intention_num = 0; //回合过程的意图顺序号
	static bool findballflag = 0; //用作找目标时ToFindObjectByturn()函数调用标志、转圈标志等
	static bool aimballflag = 0;
	static bool catchballflag = 0;
	static bool Isthreescoreunnormal = 0; //捡三分球是否出现异常

	static int normal_time = 0;		 //回合进行时间，单位是收到DSP正确数据次数，即策略扫描次数
	static int temp_normal_time = 0; //辅助延时，记下需要延时时刻的time,延时完成要清零
	normal_time++;

	Posture ptA0, ptA,ptA1, ptB, ptB1, ptB2,ptC, ptD, ptE, ptF, ptM, ptM2, ptOO, ptxy;
	static Posture ptN = {0, 0};

	ptA1.x = 5200;
	ptA1.y = 0;

	ptOO.x = 5700;
	ptOO.y = 0;

	ptxy.x = 0;
	ptxy.y = 0;

	if (1 == place_num)
	{
		ptA0.x = 7700;
		ptA0.y = 700;
		
		ptA.x = 9500; 
		ptA.y = -1600;

		ptB.x = 11200; 
		ptB.y = 300;

		ptB1.x = 11900; 
		ptB1.y = -400;

		ptB2.x = 10800; 
		ptB2.y = -1500;

		ptC.x = 10500; 
		ptC.y = 800;

		ptD.x = 9900; 
		ptD.y = -2500;

		ptE.x = 6100; 
		ptE.y = 1000;

		ptF.x = 4200; 
		ptF.y = -1000;

		ptM2.x = 7800; 
		ptM2.y = 2100;

		ptM.x = 7800; 
		ptM.y = 2000;
	}
	if (2 == place_num)
	{
		ptA0.x = 7600;
		ptA0.y = -850;
		
		ptA.x = 9500; 
		ptA.y = 1600;

		ptB.x = 11200; 
		ptB.y = -500;

		ptB1.x = 11900; 
		ptB1.y = 350;

		ptB2.x = 10800; 
		ptB2.y = 1800;

		ptC.x = 11000; 
		ptC.y = 1700;

		ptD.x = 9900; 
		ptD.y = 2500;

		ptE.x = 5300; 
		ptE.y = 2500;

		ptF.x = 4200; 
		ptF.y = 1800;

		ptM.x = 8000; 
		ptM.y =-1100;

		ptM2.x = 7800; 
		ptM2.y =-1000;
	}

	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("case 00000000000000000000000000\n");
		temp_normal_time = normal_time; 
		intention_num = 120;
		break;
	case 120:
		if ((normal_time - temp_normal_time) > 20) 
		{
			temp_normal_time = normal_time; 
			intention_num = 1001;//36
		}
		break;

	case 1001:
		ROS_INFO_STREAM("case 1111111111111111111111\n");
		if (ToPostureByMilemeter_Pid1(ptA1))
		{
			Stop();
			ROS_INFO_STREAM("have arrived at the posture A0！！！\n");
			temp_normal_time = normal_time;
			intention_num = 100;
		}
		break;

    case 100:
	if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptA0))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 1;
				}
			}
		}
		break;

	case 1:
		ROS_INFO_STREAM("case 1111111111111111111111\n");
		if (ToPostureByMilemeter_Pid2(ptA0))
		{
			Stop();
			ROS_INFO_STREAM("have arrived at the posture A0！！！\n");
			temp_normal_time = normal_time;
			intention_num = 101;
		}
		break;

	case 101:
	ROS_INFO_STREAM("case 101 101 101 101 101 \n");
	ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(-90))
				{
					temp_normal_time == normal_time;
					intention_num = 2;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(90))
				{
					temp_normal_time == normal_time;
					intention_num = 2;
				}
			}
		}
		break;

	case 2:
		ROS_INFO_STREAM("case 222222222222222222222222\n");
		if (ToPostureByAvoidance(ptA)) //避障
		{
			temp_normal_time = normal_time;
			intention_num = 201;
		}
		break;

	case 201:
		ROS_INFO_STREAM("case 201 201 201 201 201 201\n");
		if ((normal_time - temp_normal_time) < 120)
		{
			robotforward();
		}
		else
		{
			temp_normal_time = normal_time;
			intention_num = 202;
		}
		break;

	case 202: //避障后再次对准目标点
		ROS_INFO_STREAM("case 202 202 202 202 202 202\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptA))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 203;
				}
			}
		}
		break;

	case 203:
		ROS_INFO_STREAM("case 203 203 203 203 203 203 203\n");
		if (ToPostureByMilemeter_Pid2(ptA))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 301;
		}
		break;

case 301:
	ROS_INFO_STREAM("case 301 301 301 301 301 \n");
	ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(90))
				{
					temp_normal_time == normal_time;
					intention_num = 3;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-90))
				{
					temp_normal_time == normal_time;
					intention_num = 3;
				}
			}
		}
		break;

	case 3:
		ROS_INFO_STREAM("case 33333333333333333333\n");
		if (ToPostureByAvoidance(ptB)) //避障
		{
			temp_normal_time = normal_time;
			intention_num = 302;
		}
		break;

	case 302:
		ROS_INFO_STREAM("case 302 302 302 302 302 302\n");
		if ((normal_time - temp_normal_time) < 145)
		{
			robotforward();
		}
		else
		{
			temp_normal_time = normal_time;
			intention_num = 303;
		}
		break;

	case 303: //避障后再次对准目标点
		ROS_INFO_STREAM("case 303 303 303 303 303 303\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptB))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 304;
				}
			}
		}
		break;

	case 304:
		ROS_INFO_STREAM("case 304 304 304 304\n");
		if (ToPostureByMilemeter_Pid2(ptB))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 6001;
		}
		break;

	case 6001: 
		ROS_INFO_STREAM("case 6001 6001 6010\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptB2))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 6002;
				}
			}
		}
		break;

	case 6002:
		ROS_INFO_STREAM("case 6002 6002 6002\n");
		if (ToPostureByMilemeter_Pid4(ptB2))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 6003;
		}
		break;

	
	case 6003: 
		ROS_INFO_STREAM("case 6001 6001 6010\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			if (ToAngleByMPU6050(ptB1))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 6004;
				}
			}
		}
		break;

	case 6004:
		ROS_INFO_STREAM("case 6002 6002 6002\n");
		if (ToPostureByMilemeter_Pid4(ptB1))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 6;
		}
		break;

	case 6:
		ROS_INFO_STREAM("case 66666666666666\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			ROS_INFO_STREAM("are aiming to the posture M！！！！\n");
			if (ToAngleByMPU(-157)) //瞄准目标点M
			{
				temp_normal_time = normal_time;
				intention_num = 601;
			}
		}
		break;

	case 601:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 602;
		}
		break;

	case 602:
		Stop();
		intention_num = 603;
		break;

	case 603: //发射第一个篮球
		ROS_INFO_STREAM("case 603 603 603 603\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 6040;
		}
		break;

	case 6040:
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0; //单片机位置0
			if (ToAngleByMPU6050(ptB))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 6041;
				}
			}
		}
		break;

	case 6041:
		if (ToPostureByMilemeter_Pid2(ptB))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 604;
		}
		break;

	case 604:
		ROS_INFO_STREAM("case 604 604 604 604\n");
		if ((normal_time - temp_normal_time) > 3)
		{
			if (ToAngleByMPU(0))
			{
				ROS_INFO_STREAM("are turning to angle 0 to fin d the ball 2\n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 7;
			}
		}
		break;

	case 7: //第一次拾球
		ROS_INFO_STREAM("case 77777777777777777777777777777\n");
			the_object.whatObject = (objectType)2;//蓝灰色篮球
			object_pub(the_object);
			
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				ROS_INFO_STREAM(" i'm turn right \n");
				if ((dsp_status.YDist) > -2500) //找了好久，一个都没找到，就回去了
				{
					robotstraightleft(); 
				}
				else
				{
					temp_normal_time = normal_time;
					intention_num = 6040;
				}
			}
			if (2 == place_num)
			{
				ROS_INFO_STREAM(" i'm turning left\n");
				if ((dsp_status.YDist) < 2500) 
				{
					robotstraightright(); 
				}
				else
				{
					temp_normal_time = normal_time;
					intention_num = 6040;
				}
				
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" cccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 7;
					ROS_INFO("lost the ball ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (aimballflag == 1))
		{
			ROS_INFO_STREAM(" ffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 7;
					ROS_INFO_STREAM(" lost the ball again ！！！\n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					Stop();
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 71;
					ROS_INFO_STREAM("catch the ball ！！！\n");
				}
			}
		}
		break; //**********视觉找篮球结束**********

	case 71:
		if (normal_time - temp_normal_time < 67)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 72;
		}
		break;

	case 72:
		if(stop())
		intention_num = 73;
		break;

	case 73:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 74;
		break;

	case 74:
		ROS_INFO_STREAM("case 74 74 74 74 74\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 8; //抬机械臂命令已发送
		}
		break;

	case 8: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 8 8 8 8 8 8 8 8\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 80)
		{
			temp_normal_time = normal_time;
			intention_num = 90;
		}
		break;

	case 90:
	    ROS_INFO_STREAM("case 90 90 90 90 90 90\n");
		if (normal_time - temp_normal_time < 60)
		{
			robotback();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 11;
		}
		break;

	case 11:
		ROS_INFO_STREAM("case 11 11 11 11 11 11\n");
		if (ToAngleByMPU6050(ptB1)) 
		{
			ROS_INFO_STREAM("have aimed to the posture F！！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 12;
			}
		}
		break;

	case 12:
		ROS_INFO_STREAM("case 12 12 12 12 12 12 12\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid4(ptB1)) 
		{
			temp_normal_time = normal_time;
			intention_num = 13;
		}
		break;

	case 13: 
		ROS_INFO_STREAM("case 13 13 13 13 13\n");
		if (ToAngleByMPU(-158)) 
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 130;
			}
		}
		break;

	case 130:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 131;
		}
		break;

	case 131:
		Stop();
		intention_num = 14;
		break;

	case 14: 
		ROS_INFO_STREAM("case 14 14 14 14 14 14\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 1701;
		}
		break;

	case 1701:
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0; //单片机位置0
			if (ToAngleByMPU6050(ptB))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 1702;
				}
			}
		}
		break;

	case 1702:
		if (ToPostureByMilemeter_Pid3(ptB))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 1703;
		}
		break;

	case 1703:
		ROS_INFO_STREAM("case 604 604 604 604\n");
		if ((normal_time - temp_normal_time) > 3)
		{
			if (ToAngleByMPU(0))
			{
				ROS_INFO_STREAM("are turning to angle 0 to fin d the ball 2\n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 3001;
			}
		}
		break;

	case 3001: //第二次拾球
		ROS_INFO_STREAM("case 3001 3001 3001\n");
			the_object.whatObject = (objectType)2;//蓝灰色篮球
			object_pub(the_object);
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				ROS_INFO_STREAM(" i'm turn right \n");
				if ((dsp_status.YDist) > -2500) //找了好久，一个都没找到，就回去了
				{
					robotstraightleft(); 
				}
				else
				{
					temp_normal_time = normal_time;
					intention_num = 1701;
				}
			}
			if (2 == place_num)
			{
				ROS_INFO_STREAM(" i'm turning left\n");
				if ((dsp_status.YDist) < 2500) 
				{
					robotstraightright(); 
				}
				else
				{
					temp_normal_time = normal_time;
					intention_num = 1701;
				}
				
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" cccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 3001;
					ROS_INFO("lost the ball ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (aimballflag == 1))
		{
			ROS_INFO_STREAM(" ffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 3001;
					ROS_INFO_STREAM(" lost the ball again ！！！\n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					Stop();
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 3023;
					ROS_INFO_STREAM("catch the ball ！！！\n");
				}
			}
		}
		break; //**********视觉找篮球结束**********

	case 3023:
		if (normal_time - temp_normal_time < 68)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 3024;
		}
		break;

	case 3024:
		if(stop())
		intention_num = 3025;
		break;

	case 3025:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 3026;
		break;

	case 3026:
		ROS_INFO_STREAM("case 74 74 74 74 74\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 3005; //抬机械臂命令已发送
		}
		break;

	case 3005: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 3005 30005 3005\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 80)
		{
			temp_normal_time = normal_time;
			intention_num = 3006;
		}
		break;

	case 3006:
	    ROS_INFO_STREAM("case 3006 3006 3006\n");
		if (normal_time - temp_normal_time < 100)
		{
			robotback();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 3007;
		}
		break;

	case 3007:
		ROS_INFO_STREAM("case 3007 3007 3007\n");
		if (ToAngleByMPU6050(ptB1)) 
		{
			ROS_INFO_STREAM("have aimed to the posture F！！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 3008;
			}
		}
		break;

	case 3008:
		ROS_INFO_STREAM("case 3008 3008 3008\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid6(ptB1)) 
		{
			temp_normal_time = normal_time;
			intention_num = 3009;
		}
		break;

	case 3009: 
		ROS_INFO_STREAM("case 3009 3009 3009\n");
		if (ToAngleByMPU(-158)) 
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 3010;
			}
		}
		break;

	case 3010:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 3011;
		}
		break;

	case 3011:
		Stop();
		intention_num = 3012;
		break;

	case 3012: 
		ROS_INFO_STREAM("case 3012 3012 3012\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 3013;
		}
		break;

	case 3013:
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0; //单片机位置0
			if (ToAngleByMPU6050(ptB))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 3014;
				}
			}
		}
		break;

	case 3014:
		if (ToPostureByMilemeter_Pid3(ptB))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 3015;
		}
		break;

	case 3015:
		if ((normal_time - temp_normal_time) > 3)
		{
			if (ToAngleByMPU(0))
			{
				ROS_INFO_STREAM("are turning to angle 0 to fin d the ball 2\n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 4001;
			}
		}
		break;

	case 4001: //第三次拾球
		ROS_INFO_STREAM("case 4001 4001 4001\n");
			the_object.whatObject = (objectType)2;//蓝灰色篮球
			object_pub(the_object);	
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				ROS_INFO_STREAM(" i'm turn right \n");
				if ((dsp_status.YDist) > -2500) //找了好久，一个都没找到，就回去了
				{
					robotstraightleft(); 
				}
				else
				{
					temp_normal_time = normal_time;
					intention_num = 3013;
				}
			}
			if (2 == place_num)
			{
				ROS_INFO_STREAM(" i'm turning left\n");
				if ((dsp_status.YDist) < 2500) 
				{
					robotstraightright(); 
				}
				else
				{
					temp_normal_time = normal_time;
					intention_num = 3013;
				}
				
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" cccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4001;
					ROS_INFO("lost the ball ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (aimballflag == 1))
		{
			ROS_INFO_STREAM(" ffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 4001;
					ROS_INFO_STREAM(" lost the ball again ！！！\n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					Stop();
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 2003;
					ROS_INFO_STREAM("catch the ball ！！！\n");
				}
			}
		}
		break; //**********视觉找篮球结束**********

	case 2003:
		if (normal_time - temp_normal_time < 66)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 2004;
		}
		break;

	case 2004:
		if(stop())
		intention_num = 2025;
		break;

	case 2025:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 2026;
		break;

	case 2026:
		ROS_INFO_STREAM("case 74 74 74 74 74\n");
		Stop();
		if ((normal_time - temp_normal_time) > 10)
		{
		ROS_INFO_STREAM("case flag flag\n");
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		ROS_INFO_STREAM("case flag1 flag1\n");
		temp_normal_time = normal_time;
		intention_num = 2005; //抬机械臂命令已发送
		}
		break;

	case 2005: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 2005 20005 2005\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 80)
		{
			temp_normal_time = normal_time;
			intention_num = 2006;
		}
		break;

	case 2006:
	    ROS_INFO_STREAM("case 2006 2006 2006\n");
		if (normal_time - temp_normal_time < 100)
		{
			robotback();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2007;
		}
		break;

	case 2007:
		ROS_INFO_STREAM("case 2007 2007 2007\n");
		if (ToAngleByMPU6050(ptB1)) 
		{
			ROS_INFO_STREAM("have aimed to the posture F！！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 2008;
			}
		}
		break;

	case 2008:
		ROS_INFO_STREAM("case 2008 2008 2008\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid6(ptB1)) 
		{
			temp_normal_time = normal_time;
			intention_num = 2009;
		}
		break;

	case 2009: 
		ROS_INFO_STREAM("case 2009 2009 2009\n");
		if (ToAngleByMPU(-157)) 
		{
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 2010;
			}
		}
		break;

	case 2010:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 2011;
		}
		break;

	case 2011:
		Stop();
		intention_num = 2012;
		break;

	case 2012: 
		ROS_INFO_STREAM("case 2012 2012 2012\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 2013;
		}
		break;

	case 2013:
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0; //单片机位置0
			if (ToAngleByMPU6050(ptB))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 2014;
				}
			}
		}
		break;

	case 2014:
		if (ToPostureByMilemeter_Pid3(ptB))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 2015;
		}
		break;

	case 2015:
		ROS_INFO_STREAM("case 604 604 604 604\n");
		if ((normal_time - temp_normal_time) > 5)
		{
			if (ToAngleByMPU(0))
			{
				ROS_INFO_STREAM("are turning to angle 0 to fin d the ball 2\n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 20;
			}
		}
		break;


	case 20://第四次拾球
		ROS_INFO_STREAM("case 20 20 20 20 20 20\n");
		the_object.whatObject = (objectType)2;//蓝灰色篮球
		object_pub(the_object);
			
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0)) 
		{
			findballflag = 0;
			aimballflag = 0;
			if (1 == place_num)
			{
				ROS_INFO_STREAM(" i'm turn right \n");
				if ((dsp_status.YDist) > -2500) //找了好久，一个都没找到，就回去了
				{
					robotstraightleft(); 
				}
				else
				{
					temp_normal_time = normal_time;
					intention_num = 2013;
				}
			}
			if (2 == place_num)
			{
				ROS_INFO_STREAM(" i'm turning left\n");
				if ((dsp_status.YDist) < 2500) 
				{
					robotstraightright(); 
				}
				else
				{
					temp_normal_time = normal_time;
					intention_num = 2013;
				}
				
			}
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			ROS_INFO_STREAM(" cccccccccccccccccccccccccccccc\n");
			if (ToAimBallByVision())
			{
				ROS_INFO("ToAimBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准球的过程中突然丢了
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 20;
					ROS_INFO("lost the ball ！！！： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((aimballflag == 1) && (aimballflag == 1))
		{
			ROS_INFO_STREAM(" ffffffffffffffffffffffffff\n");
			if (ToBallByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				ROS_INFO("ToBallByVision is starting ： %lf  , %ld\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //前往球的跟前过程中，又没看到球，重新去找
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 20;
					ROS_INFO_STREAM(" lost the ball again ！！！\n");
				}
				else
				{
					aimballflag = 0;
					findballflag = 0;
					Stop();
					temp_normal_time = normal_time; //为延时作准备
					intention_num = 21;
					ROS_INFO_STREAM("catch the ball ！！！\n");
				}
			}
		}
		break; //**********视觉找篮球结束**********

	case 21:
		if (normal_time - temp_normal_time < 66)
		{
			robotforwardtoolow();
		}
		else
		{
			Stop();
			intention_num = 2200;
		}
		break;

	case 2200:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 2201;
		}
		break;

	case 2201:
		Stop();
		intention_num = 22;
		break;

	case 22:
		if ((normal_time - temp_normal_time) > 10)
		ROS_INFO_STREAM("case 22 22 22 22 22 22 22\n");
		Stop();
		Control.SendData_state = 1; //球已持住，发送抬机械臂命令（不需要放下命令，只需延时），下一步立即复位0
		temp_normal_time = normal_time;
		intention_num =23; //抬机械臂命令已发送
		break;

	case 23: //等待机械臂抬起,放臂完成
		ROS_INFO_STREAM("case 23 23 23 23 23 23\n");
		Control.SendData_state = 0; //单片机位置0
		if ((normal_time - temp_normal_time) > 80)
		{
			temp_normal_time = normal_time;
			intention_num = 240;
		}
		break;

	case 240:
	    ROS_INFO_STREAM("case 240 240 240 240\n");
		if (normal_time - temp_normal_time < 100)
		{
			robotback();
		}
		else
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 24;
		}
		break;

		
	case 24:
		ROS_INFO_STREAM("case 24 24 24 24 24 24 24 24\n");
		if (ToAngleByMPU6050(ptB1)) 
		{
			ROS_INFO_STREAM("have aimed to the posture D！！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 25;
			}
		}
		break;

	case 25:
		ROS_INFO_STREAM("case 25 25 25 25 25 25\n");
		Control.SendData_state = 0;
		if (ToPostureByMilemeter_Pid6(ptB1)) 
		{
			temp_normal_time = normal_time;
			intention_num = 28;
		}
		break;

	case 28: 
		ROS_INFO_STREAM("case 28 28 28 28 28 28\n");
		if (ToAngleByMPU(-157)) 
		{
			ROS_INFO_STREAM("have aimed to the posture F！！！\n");
			if (stop())
			{
				temp_normal_time = normal_time;
				intention_num = 280;
			}
		}
		break;

	case 280:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 281;
		}
		break;

	case 281:
		Stop();
		intention_num = 29;
		break;

	case 29: 
		ROS_INFO_STREAM("case 29 29 29 29 29 29\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; //发送发射指令，下一步立即置位0
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 2901;
		}
		break;

	case 2901:
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0;
			if (ToAngleByMPU6050(ptC))
			{
				ROS_INFO("the posture is right %lf\n", dsp_status.fAngle);
				Stop();
				temp_normal_time = normal_time;
				intention_num = 2902;
			}
		}
		break;

	case 2902:
		if ((normal_time - temp_normal_time) > 10)
		{
			if (ToPostureByMilemeter_Pid2(ptC))
			{
				ROS_INFO_STREAM("stop stop stop stop stop  stop stop stop stop stop \n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 30;
			}
		}
		break;

	case 30:
		ROS_INFO_STREAM("case 30 30 30 30 30 30 30\n");
		Control.SendData_state = 0;
		if (ToAngleByMPU6050(ptE))
		{
			ROS_INFO("the posture is right %lf\n", dsp_status.fAngle);
			Stop();
			temp_normal_time = normal_time;
			intention_num = 0301;
		}
		break;

	case 0301:
		ROS_INFO_STREAM("case 0301 0301 0301\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			if (ToPostureByMilemeter_Pid2(ptE))
			{
				ROS_INFO_STREAM("stop stop stop stop stop  stop stop stop stop stop \n");
				Stop();
				temp_normal_time = normal_time;
				intention_num = 351;
			}
		}
		break;

	case 351: 
		ROS_INFO_STREAM("case 351 351 351 351\n");
		if ((normal_time - temp_normal_time) > 10)
		{
			if (ToAngleByMPU6050(ptF))
			{
				if (stop())
				{
					temp_normal_time = normal_time;
					intention_num = 352;
				}
			}
		}
		break;

	case 352:
		ROS_INFO_STREAM("case 352 352 352\n");
		if (ToPostureByMilemeter_Pid2(ptF))
		{
			Stop();
			temp_normal_time = normal_time;
			intention_num = 36;
		}
		break;

    case 36:
		ROS_INFO_STREAM("case 36 36 36 36 36\n");
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(170))
				{
					temp_normal_time == normal_time;
					intention_num = 37;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-170))
				{
					temp_normal_time == normal_time;
					intention_num = 37;
				}
			}
		}
		break;

		/********************************* 第一次识别标定柱 *********************************/
	case 37:
		ROS_INFO_STREAM("case 37 37 37 37 37\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2000)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 36;
			}
			
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 37;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 371;
				}
			}
		}
		break;

	case 371:
		if(stop())
		{
			temp_normal_time = normal_time;
			intention_num = 372;
		}
		break;

	case 372:
		Stop();
		intention_num = 3730;
		break;

	case 3730:
		if(stop())
		intention_num = 3731;
		break;

	case 3731:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 373;
		break;

	case 373: 
		ROS_INFO_STREAM("case 373 373 373\n");
		if ((normal_time - temp_normal_time) > 20)
		{
			Stop();
			Control.SendData_state = 2; 
			ROS_INFO_STREAM("launching launching launching ！！！！\n");
			temp_normal_time = normal_time;
			intention_num = 420;
		}
		break;

	case 420: 
		ROS_INFO_STREAM("case 420 420 420\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 4201;
		}
		break;

	case 4201:
		if ((normal_time - temp_normal_time) > 150)
		{
			temp_normal_time = normal_time;
			intention_num = 4204;
		}
		break;
	
	case 4204:
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(170))
				{
					temp_normal_time == normal_time;
					intention_num = 43;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-170))
				{
					temp_normal_time == normal_time;
					intention_num = 43;
				}
			}
		}
		break;
	/********************************* 第二次识别标定柱 *********************************/
	case 43:
		ROS_INFO_STREAM("case 43 43 43 43\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2000)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 4204;
			}
			
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 43;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 434;
				}
			}
		}
		break;

	case 432:
		if(stop())
		intention_num = 433;
		break;

	case 433:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 434;
		break;

	case 434: //发射第一个篮球
		ROS_INFO_STREAM("case 434 434 434\n");
		Control.SendData_state = 2;
		temp_normal_time = normal_time;
		intention_num = 435;
		break;

	case 435: 
		ROS_INFO_STREAM("case 435 435 435 435\n");
		if ((normal_time - temp_normal_time) > 3)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 441;
		}
		break;

	case 441:
		if ((normal_time - temp_normal_time) > 150)
		{
			temp_normal_time = normal_time;
			intention_num = 442;
		}
		break;
	
	case 442:
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(170))
				{
					temp_normal_time == normal_time;
					intention_num = 443;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-170))
				{
					temp_normal_time == normal_time;
					intention_num = 443;
				}
			}
		}
		break;
	/********************************* 第三次识别标定柱 *********************************/
	case 443:
		ROS_INFO_STREAM("case 443 443 443 443\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2000)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 442;
			}
			
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 443;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 447;
				}
			}
		}
		break;

	case 444:
		if(stop())
		intention_num = 445;
		break;

	case 446:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 447;
		break;

	case 447: //发射第一个篮球
		ROS_INFO_STREAM("case 434 434 434\n");
		Control.SendData_state = 2;
		temp_normal_time = normal_time;
		intention_num = 448;
		break;

	case 448: //发射第一个篮球
		ROS_INFO_STREAM("case 435 435 435 435\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 451;
		}
		break;

	case 451:
		if ((normal_time - temp_normal_time) > 150)
		{
			temp_normal_time = normal_time;
			intention_num = 452;
		}
		break;
	
	case 452:
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(170))
				{
					temp_normal_time == normal_time;
					intention_num = 453;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-170))
				{
					temp_normal_time == normal_time;
					intention_num = 453;
				}
			}
		}
		break;
	/********************************* 第四次识别标定柱 *********************************/
	case 453:
		ROS_INFO_STREAM("case 453 453 453 453\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2000)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 452;
			}
			
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 453;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 456;
				}
			}
		}
		break;

	case 454:
		if(stop())
		intention_num = 455;
		break;

	case 455:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 456;
		break;

	case 456: 
		ROS_INFO_STREAM("case 456 456 456\n");
		Control.SendData_state = 2;
		temp_normal_time = normal_time;
		intention_num = 457;
		break;

	case 457: 
		ROS_INFO_STREAM("case 435 435 435 435\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 461;
		}
		break;

	case 461:
		if ((normal_time - temp_normal_time) > 150)
		{
			temp_normal_time = normal_time;
			intention_num = 462;
		}
		break;
	
	case 462:
		ROS_INFO_STREAM("are spining  are spining！\n");
		if((normal_time - temp_normal_time) > 20)
		{
			if(1 == place_num)
			{
				if(ToAngleByMPU(170))
				{
					temp_normal_time == normal_time;
					intention_num = 463;
				}
			}
			if(2 == place_num)
			{
				if(ToAngleByMPU(-170))
				{
					temp_normal_time == normal_time;
					intention_num = 463;
				}
			}
		}
		break;
	/********************************* 第五次识别标定柱 *********************************/
	case 463:
		ROS_INFO_STREAM("case 43 43 43 43\n ");
		the_object.whatObject = (objectType)1;//标定柱
		object_pub(the_object);
		temp_normal_time == normal_time;
		if ((objectAngleDistance_Y.first == 0.0) && (objectAngleDistance_Y.second == 0.0))
		{
			findballflag = 0;
			aimballflag = 0;
			if((normal_time - temp_normal_time) < 2000)
			{
				if (1 == place_num)
				{
					ToFindObjectByturnright();
				}
				if (2 == place_num)
				{
					ToFindObjectByturnleft();
				}
			}
			else
			{
				temp_normal_time = normal_time;
				intention_num = 462;
			}
			
		}
		else
		{
			findballflag = 1;
			ROS_INFO_STREAM("findobjectflag findobjectflag findobjectflag\n");
		}
		if ((findballflag == 1) && (aimballflag == 0))
		{
			if (ToAimBallByVision())
			{
				if ((0.0 == objectAngleDistance_Y.first) && (0.0 == objectAngleDistance_Y.second))
				{ //对准的过程中突然丢了
					findballflag = 0;
					aimballflag = 0;
					intention_num = 463;
				}
				else
				{
					aimballflag = 1;
					ROS_INFO("the pole objectAngle aimedaimedaimed is %lf;distance aimedaimedaimed is %ld!!!!!!!!\n", objectAngleDistance_Y.first, objectAngleDistance_Y.second);
				}
			}
		}
		if ((findballflag == 1) && (aimballflag == 1)) //前进到距离2.5m
		{
			if (ToPoleByVision(objectAngleDistance_Y.second, objectAngleDistance_Y.first))
			{
				if ((fabs(objectAngleDistance_Y.first - 90.0) <= 6) && ((objectAngleDistance_Y.second == -1) || (abs(objectAngleDistance_Y.second - 2400) < 50)) && (aimballflag == 1))
				{
					Stop();
					aimballflag = 0;
					findballflag = 0;
					temp_normal_time = normal_time;
					intention_num = 466;
				}
			}
		}
		break;

	case 464:
		if(stop())
		intention_num = 465;
		break;

	case 465:
		Stop();
		temp_normal_time = normal_time;
		intention_num = 466;
		break;

	case 466: 
		ROS_INFO_STREAM("case 434 434 434\n");
		Control.SendData_state = 2;
		temp_normal_time = normal_time;
		intention_num = 467;
		break;

	case 467: 
		ROS_INFO_STREAM("case 435 435 435 435\n");
		if ((normal_time - temp_normal_time) > 2)
		{
			Control.SendData_state = 0;
			temp_normal_time = normal_time;
			intention_num = 49;
		}
		break;

	case 49:
		ROS_INFO_STREAM("case 49 49 49 49 49 \n");
		Stop();
		break;

	default:
		ROS_INFO_STREAM("case default default default default default default default \n");
		normal_time = 0;
		break; 
	}
}


void DecisionMaking::PidAngle_init(float kp, float ki, float kd) //PidAngle控制算法的变量初始化
{
	pid_angle.set_value = 0;
	pid_angle.actual_value = 0;
	pid_angle.err = 0;
	pid_angle.err_last = 0;
	pid_angle.input_value = 0;
	pid_angle.integral = 0;

	pid_angle.Kp = kp;
	pid_angle.Ki = ki;
	pid_angle.Kd = kd;
}

void DecisionMaking::PidDistX_init(float kp, float ki, float kd) //PidDist控制算法的变量初始化
{
	pid_distx.set_value = 0;
	pid_distx.actual_value = 0;
	pid_distx.err = 0;
	pid_distx.err_last = 0;
	pid_distx.input_value = 0;
	pid_distx.integral = 0;

	pid_distx.Kp = kp;
	pid_distx.Ki = ki;
	pid_distx.Kd = kd;
}

void DecisionMaking::PidDistY_init(float kp, float ki, float kd) //PidDist控制算法的变量初始化
{
	pid_disty.set_value = 0;
	pid_disty.actual_value = 0;
	pid_disty.err = 0;
	pid_disty.err_last = 0;
	pid_disty.input_value = 0;
	pid_disty.integral = 0;

	pid_disty.Kp = kp;
	pid_disty.Ki = ki;
	pid_disty.Kd = kd;
}

float DecisionMaking::Pidlow_angle(float angle) //慢速的角度调整，快速移动时用
{
	//这里kp之前默认为2
	pid_angle.err = angle;

	float k = 1;
	if (fabs(pid_angle.err) < 15)
		//k=0.3;
		k = 1.4;
	else
		//k=1;          //定义一个比例系数k，当偏差小于15时，使得比例系数扩大1.5倍（其实就是分段P控制了一下）
		k = 1;

	pid_angle.integral += pid_angle.err;
	pid_angle.input_value = k * pid_angle.Kp * pid_angle.err + pid_angle.Ki * pid_angle.integral + pid_angle.Kd * (pid_angle.err - pid_angle.err_last);

	//！！！设定车体旋转角速度的最高值，单位度/s
	//注意：不能限较高的最低速（特别是这个最低速度还不小时），因为当偏差很小时，由于限制了最低速度，会超调到反向偏差（每次采样时间和速度决定了每次调节的偏差值），如此不断震荡。
	if (fabs(pid_angle.input_value) > 30)
	{
		if (pid_angle.err > 0)
			pid_angle.input_value = 30;
		else
			pid_angle.input_value = -30;
	}

	pid_angle.err_last = pid_angle.err;

	return pid_angle.input_value;
}

float DecisionMaking::Pid_angle_new(float angle)
{
	ROS_INFO("pid调节前的角度为：%lf \n", angle);
	pid_angle.err = angle;
	float k = 0;
	if ((angle >= 0) && (angle <= 180))
	{
		if (angle <= 5)
			k = 0.8;
		else if (angle <= 10)
			k = 0.82;		  //0.85
		else if (angle <= 25) //15
			k = 0.9;		  //0.6
		else if (angle <= 40)
			k = 1.5; //0.65
		else if ((angle > 40) && (angle <= 80))
			k = 1.2; //0.7//k=-(1-((1-0.4)/(140-40))*(angle-40));
		else if ((angle > 80) && (angle <= 110))
			k = 1.0; //0.6
		else
			k = 0.8; //0.46
	}
	else
	{
		if (angle >= -5)
			k = 0.7;
		else if (angle >= -10)
			k =0.9;
		else if (angle >= -25)
			k = 1.7; //k=0.75;
		else if (angle >= -40)
			k = 1.5; //k=0.8;
		else if ((angle < -40) && (angle >= -80))
			k = 1.2; //k=0.9;//k=1-((1-0.4)/(140-40))*(angle-40);
		else if ((angle < -80) && (angle >= -110))
			k = 1.0; //k=0.88;
		else
			k = 0.8;
	}
	pid_angle.integral += pid_angle.err;
	pid_angle.input_value = k * (pid_angle.Kp * pid_angle.err + pid_angle.Ki * pid_angle.integral + pid_angle.Kd * (pid_angle.err - pid_angle.err_last));
	ROS_INFO_STREAM("pid_angle.input_value" << pid_angle.input_value);

	pid_angle.err_last = pid_angle.err;
	//PidAngle.input_value = k * PidAngle.Kp * PidAngle.err;
	return pid_angle.input_value;
}

float DecisionMaking::Pid_distX(float dist) //相对X轴距离的PID控制实现
{
	pid_distx.err = dist;
	float k;
	if (pid_distx.err <= 1000)
	{
		k = 1; //1
	}
	else if ((pid_distx.err > 1000) && (pid_distx.err <= 4000))
	{
		k = 0.4 - ((0.3 - 0.05) / (4000 - 1000)) * (pid_distx.err - 1000);
	}
	else if ((pid_distx.err > 4000) && (pid_distx.err <= 5000))
	{
		k = 0.18 - ((0.18 - 0.08) / (5000 - 4000)) * (pid_distx.err - 4000);
	}
	else
	{
		k = 0.08;
	}
	pid_distx.integral += pid_distx.err;
	pid_distx.input_value = k * (pid_distx.Kp * pid_distx.err + pid_distx.Ki * pid_distx.integral + pid_distx.Kd * (pid_distx.err - pid_distx.err_last));
	if (pid_distx.input_value < 200)
	{
		pid_distx.input_value = 200;
	}
	if (pid_distx.input_value > 800)
	{
		pid_distx.input_value = 800;
	}
	pid_distx.err_last = pid_distx.err;
	return pid_distx.input_value;
}

float DecisionMaking::Pid_distY(float dist) //相对Y轴距离的PID控制实现
{
	pid_disty.err = dist;
	float k;
	if (pid_disty.err <= 1000)
	{
		k = 1; //0.4
	}
	else if ((pid_disty.err > 1000) && (pid_disty.err <= 4000))
	{
		k = 0.4 - ((0.3 - 0.05) / (4000 - 1000)) * (pid_disty.err - 1000);
	}
	else if ((pid_disty.err > 4000) && (pid_disty.err <= 5000))
	{
		k = 0.18 - ((0.18 - 0.08) / (5000 - 4000)) * (pid_disty.err - 4000);
	}
	else
	{
		k = 0.08;
	}
	pid_disty.integral += pid_disty.err;
	pid_disty.input_value = k * (pid_disty.Kp * pid_disty.err + pid_disty.Ki * pid_disty.integral + pid_disty.Kd * (pid_disty.err - pid_disty.err_last));
	if (pid_disty.input_value < 200)
	{
		pid_disty.input_value = 200;
	}
	if (pid_disty.input_value > 800)
	{
		pid_disty.input_value = 800;
	}
	pid_disty.err_last = pid_disty.err;
	return pid_disty.input_value;
}

float DecisionMaking::Pid_realize(float set, float actual) //常规PID实现
{
	pid_angle.set_value = set;
	pid_angle.err = pid_angle.set_value - actual;
	pid_angle.integral += pid_angle.err;
	pid_angle.input_value = pid_angle.Kp * pid_angle.err + pid_angle.Ki * pid_angle.integral + pid_angle.Kd * (pid_angle.err - pid_angle.err_last);
	pid_angle.err_last = pid_angle.err;
	return pid_angle.input_value;
}

void DecisionMaking::GetNspeed(short Vx, short Vy, double W, float jd) //轮速转换函数，换算得到轮子的给定转速,Vx,Vy为车体在以出发点为坐标原点的X/Y方向的实际速度，单位mm/s，W为车体自身旋转角速度，单位角度/s
{
	jd = -jd;																																  //逻辑取反，理解起来有点绕（比如车子要往30度方向走，按照只给Vx应该先让它顺时针转30度才会是往30度正前方走，但现在车头还是朝向出发时的正前方，相对于期望的转30度后的方位角就是-30度了！）
	Control.V1 = (short)(-sin((45.00 - jd) * pi / 180) * Vx - cos((45.00 - jd) * pi / 180) * Vy - 304 * W * pi / 180.0) / (127 * pi / 612.5); //(76.2*pi*2.0)/12.25*60
	Control.V2 = (short)(cos((45.00 - jd) * pi / 180) * Vx - sin((45.00 - jd) * pi / 180) * Vy - 304 * W * pi / 180.0) / (127 * pi / 612.5);
	Control.V3 = (short)(sin((45.00 - jd) * pi / 180) * Vx + cos((45.00 - jd) * pi / 180) * Vy - 304 * W * pi / 180.0) / (127 * pi / 612.5);
	Control.V4 = (short)(-cos((45.00 - jd) * pi / 180) * Vx + sin((45.00 - jd) * pi / 180) * Vy - 304 * W * pi / 180.0) / (127 * pi / 612.5);
	ROS_INFO("Control.V1  is %d\n", Control.V1);
	ROS_INFO("Control.V2  is %d\n", Control.V2);
	ROS_INFO("Control.V3  is %d\n", Control.V3);
	ROS_INFO("Control.V4  is %d\n", Control.V4);
}

double DecisionMaking::mth_PointsDist(Posture a, Posture b)
{
	return sqrt((float)((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)));
}

double DecisionMaking::mth_PointsDist(Posture a)
{
	return sqrt((float)(a.x * a.x + a.y * a.y));
}

double DecisionMaking::mth_ChangeAngle(double theta)
{
	while (theta > 180)
		theta = theta - 360;
	while (theta <= -180)
		theta = theta + 360;
	return theta;
}

bool DecisionMaking::ToAngleByPosture(Posture pt)
{
	Posture ept;
	double theta1, theta2, Er;
	float Vx, Vy, W, jd;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);	//转化为角度制
	theta1 = (double)dsp_status.fAngle; //当前机器人偏离X轴角度,角度制
	theta2 = atan2(ept.y, ept.x) * 180 / pi;
	//double atan2(double y,double x) 返回的是原点至点(x,y)的方位角，即与 x 轴的夹角。
	//也可以理解为复数 x+yi 的辐角。返回值的单位为弧度，取值范围为(-pi,pi)
	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1); //**2、机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);

	ROS_INFO("dsp_status.fAngle theta1 is %f\n", theta1);
	ROS_INFO("atan2(ept.y,ept.x) theat2 is %f\n", theta2);
	ROS_INFO("ToAngleByPosture Er is:%f\n", Er);

	if ((Er > 0) && (Er <= 180))
	{
		if (Er > 15)
			ToFindObjectByturnright_high();
		else
			ToFindObjectByturnrightlow();
	}
	else
	{
		if (Er < -15)
			ToFindObjectByturnleft_high();
		else
			ToFindObjectByturnleftlow();
	}
	Control.flag_start_stop = 1; //允许电机转动
	if (fabs(Er) < 2.5)
	{
		stop();
		ROS_INFO_STREAM("ToAngleByMPU6050() stop11111111111111111111111111111111111111\n");
		return 1;
	}
	else
		return 0;
}

bool DecisionMaking::ToAngleByPosture2(Posture pt) //2019 new
{
	Posture ept;
	double theta1, theta2, Er;
	float Vx, Vy, W, jd;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);	//转化为角度制
	theta1 = (double)dsp_status.fAngle; //当前机器人偏离X轴角度,角度制
	theta2 = atan2(ept.y, ept.x) * 180 / pi;
	//double atan2(double y,double x) 返回的是原点至点(x,y)的方位角，即与 x 轴的夹角。
	//也可以理解为复数 x+yi 的辐角。返回值的单位为弧度，取值范围为(-pi,pi)
	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1); //**2、机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);

	ROS_INFO("dsp_status.fAngle theta1 is %f\n", theta1);
	ROS_INFO("atan2(ept.y,ept.x) theat2 is %f\n", theta2);
	ROS_INFO("ToAngleByPosture Er is:%f\n", Er);

	if ((Er >= -180) && (Er <= 180))
		//{
		//if(Er>15)
		//	ToFindObjectByturnright_high();
		//	else
		//		ToFindObjectByturnrightlow();
		//}else
		//{
		//	if(Er<-15)
		//	ToFindObjectByturnleft_high();
		//	else
		//	ToFindObjectByturnleftlow();
		//}
		Vx = 450;
	Vy = 0;
	jd = (Er);
	W = 0;
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((abs((dsp_status.XDist) - (pt.x)) < 70) && (abs((dsp_status.YDist) - (pt.y)) < 70))
	{
		Stop();
		ROS_INFO_STREAM("已经到目标点！！！\n");
		return 1;
	}
	/*if(fabs(Er)<2.5)
	{
		stop();
	ROS_INFO_STREAM(" stop11111111111111111111111111111111111111\n");
		return 1;
	}*/
	else
		return 0;
}

bool DecisionMaking::ToAngleByMPU6050(Posture pt) //通过陀螺仪调整自身到一个期望姿态角度以对准某点
{
	Posture ept;
	double theta1, theta2, Er;
	float Vx, Vy, W, jd;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);	//转化为角度制
	theta1 = (double)dsp_status.fAngle; //当前机器人偏离X轴角度,角度制
	theta2 = atan2(ept.y, ept.x) * 180 / pi;
	//double atan2(double y,double x) 返回的是原点至点(x,y)的方位角，即与 x 轴的夹角。
	//也可以理解为复数 x+yi 的辐角。返回值的单位为弧度，取值范围为(-pi,pi)
	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1); //**2、机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);

	ROS_INFO("dsp_status.fAngle theta1 is %f\n", theta1);
	ROS_INFO("atan2(ept.y,ept.x) theat2 is %f\n", theta2);
	ROS_INFO("ToAngleByMPU6050 Er is:%f\n", Er);

	///******************PID控制start******************/
	W = 3*Pid_angle_new((float)Er); //pid比例系数要根据实际情况调节
	if (abs(W) > 100)//60
	{
		if (W > 100)
			W = 100;//60
		else if (W < 0)
			W = -100;//-60
		else
			W = 0;
	}
	ROS_INFO("ToAngleByMPU6050 W is:%f\n", W);

	Vx = 0;
	Vy = 0;
	jd = 0;
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
								 ///*****************PID控制end*******************/

	//给函数添加完成返回值，视为完成的角度根据精度调整
	if (fabs(Er) < 2)
	{
		stop();
		ROS_INFO_STREAM("ToAngleByMPU6050() stop11111111111111111111111111111111111111\n");
		return 1;
	}
	else
		return 0;
}

bool DecisionMaking::ToAngleByMPU(double Angle) //通过陀螺仪调整自身到一个期望姿态角度
{
	double theta1, theta2, Er;
	float Vx, Vy, W, jd;

	theta1 = (double)dsp_status.fAngle; //当前机器人偏离X轴角度,角度制
	theta2 = Angle;

	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1); //**2、机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);
	ROS_INFO(" Er is :  %f\n", Er);

	// W=-30;
	W = 3 * Pid_angle_new((float)Er); //pid比例系数要根据实际情况调节
	if (W > 70)
		W = 70;
	else if (W < -70)
		W = -70;

	ROS_INFO("ToAngleByMPU W is :  %f\n", W);
	Vx = 0;
	Vy = 0;
	jd = 0;
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动

	if (fabs(Er) < 2)
	{
		ROS_INFO_STREAM("010101010100100101010010\n");
		stop();
		ROS_INFO_STREAM("stop11111111111111111111111111111111111111\n");
		return 1;
	}
	else
		return 0;
}

bool DecisionMaking::ToAngleForHomeByMPU6050() //用于走折线快速回位原点
{
	if (abs(dsp_status.fAngle) >= 3.0)
	{
		if (dsp_status.fAngle > 0)
		{
			ToFindObjectByturnleft();
		}
		else
		{
			ToFindObjectByturnright();
		}
		return 0;
	}
	else if (abs(dsp_status.YDist) >= 30)
	{
		if (dsp_status.YDist > 0)
		{
			robotstraightleft();
		}
		if (dsp_status.YDist < 0)
		{
			robotstraightright();
		}
		return 0;
	}
	else if (abs(dsp_status.XDist) >= 30)
	{
		robotback();
		return 0;
	}
	else
	{
		Stop();
		return 1;
	}
}

bool DecisionMaking::ToAimBallByVision() //这是停下的时候通过摄像头返回的角度偏差对准球
{
	double Er;
	short Vx, Vy, W;
	float jd;

	Er = 90.00 - objectAngleDistance_Y.first;
	ROS_INFO("ToAimBallByVision 里的角度 Er is : %lf \n", Er);
	if (fabs(Er) >= 2)
	{
		W = 1.2 * Pidlow_angle(Er); //此比例系数要根据实际情况调节，也可以通过形参传递过来
		if (abs(W) < aim_Wmin)
			if (W > 0)
				W = aim_Wmin;
			else
				W = -aim_Wmin;
	}
	else
		W = 0;
	ROS_INFO("ToAimBallByVision 里的角速度 W is: %d \n", W);
	Vx = 0;
	Vy = 0;
	jd = 0;

	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToAimBallByVision 里的旋转:%d\n", W);
	ROS_INFO("ToAimBallByVision 里的角度 Er is : %lf \n", Er);
	if (fabs(Er) < 3)
	{
		stop();
		return 1;
		ROS_INFO("ToAimBallByVision 对准了 对准了 对准了 :%d\n", W);
	}
	else if (90.00 == Er) 
	{
		ROS_INFO_STREAM("突然来个90度，需要处理呀！！！");
		return 1;
	}
	else
		return 0;
}

void DecisionMaking::ToFindObjectByturnleft()
{
	Control.V1 = 200; //新换的大轮给100的轮速可以识别，给150试一下
	Control.V2 = 200;
	Control.V3 = 200;
	Control.V4 = 200;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnleft V1 V2 V3 V4 is %d ,%d ,%d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnleft_high()
{
	Control.V1 = 700; //新换的大轮给100的轮速可以识别，给150试一下
	Control.V2 = 700;
	Control.V3 = 700;
	Control.V4 = 700;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnleft V1 V2 V3 V4 is %d ,%d ,%d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnleftlow()
{
	Control.V1 = 1; //新换的大轮给100的轮速可以识别，给150试一下
	Control.V2 = 1;
	Control.V3 = 1;
	Control.V4 = 1;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnleft V1 V2 V3 V4 is %d ,%d ,%d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnright()
{
	Control.V1 = -250;
	Control.V2 = -250;
	Control.V3 = -250;
	Control.V4 = -250;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnright V1 V2 V3 V4 is %d , %d , %d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnright_high()
{
	Control.V1 = -700; //新换的大轮给100的轮速可以识别，给150试一下
	Control.V2 = -700;
	Control.V3 = -700;
	Control.V4 = -700;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnleft V1 V2 V3 V4 is %d ,%d ,%d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::ToFindObjectByturnrightlow()
{
	Control.V1 = -1;
	Control.V2 = -1;
	Control.V3 = -1;
	Control.V4 = -1;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("ToFindObjectByturnright V1 V2 V3 V4 is %d , %d , %d, %d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotback() //开环后退
{
	Control.V1 = 600;
	Control.V2 = -600;
	Control.V3 = -600;
	Control.V4 = 600;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotback V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotbacklow() //开环后退
{
	Control.V1 = 250;
	Control.V2 = -250;
	Control.V3 = -250;
	Control.V4 = 250;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotbacklow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotbacktoolow() //开环后退
{
	Control.V1 = 100;
	Control.V2 = -100;
	Control.V3 = -100;
	Control.V4 = 100;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotbacklow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotforward() //开环前进
{
	Control.V1 = -1000;
	Control.V2 = 1000;
	Control.V3 = 1000;
	Control.V4 = -1000;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotforward V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotforwardlow() //开环前进
{
	Control.V1 = -400;
	Control.V2 = 400;
	Control.V3 = 400;
	Control.V4 = -400;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotforwardlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotforwardtoolow() //开环前进
{
	Control.V1 = -250;
	Control.V2 = 250;
	Control.V3 = 250;
	Control.V4 = -250;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotforwardlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotstraightright() //开环横向右移
{
	Control.V1 = -600; //-350; //450(2017)
	Control.V2 = -600; //-350;
	Control.V3 = 600;  //350;
	Control.V4 = 600;  //350;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotstraightright V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotstraightleft() //开环横向左移
{
	Control.V1 = 600;  //300; //450(2017)
	Control.V2 = 600;  //300;
	Control.V3 = -600; //-300;
	Control.V4 = -600; //-300;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotstraughtleft V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotstraightrightlow() //开环横向右移(速度略低)
{
	Control.V1 = -350;
	Control.V2 = -350;
	Control.V3 = 350;
	Control.V4 = 350;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotstraightrightlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotstraightleftlow() //开环横向左移(速度略低)
{
	Control.V1 = 350;//220
	Control.V2 = 350;
	Control.V3 = -350;
	Control.V4 = -350;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotstraughtleftlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robot45back() //开环45度后退
{
	Control.V1 = 450;
	Control.V2 = 0;
	Control.V3 = -450;
	Control.V4 = 0;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robot45back V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robot45backlow() //开环45度后退
{
	Control.V1 = 150;
	Control.V2 = 0;
	Control.V3 = -150;
	Control.V4 = 0;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robot45backlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotfu45back() //开环-45度后退
{
	Control.V1 = 0;
	Control.V2 = -450;
	Control.V3 = 0;
	Control.V4 = 450;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotfu45back V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

void DecisionMaking::robotfu45backlow() //开环-45度后退
{
	Control.V1 = 0;
	Control.V2 = -150;
	Control.V3 = 0;
	Control.V4 = 150;

	Control.flag_start_stop = 1; //允许电机转动
	ROS_INFO("robotfu45backlow V1 V2 V3 V4 is %d ,%d ,%d,%d\n", Control.V1, Control.V2, Control.V3, Control.V4);
}

bool DecisionMaking::ToPostureByMilemeter_Pid(Posture sourpt, Posture destpt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed, Er; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd, aimjd;
	Posture ept;

	ept.x = (destpt.x - dsp_status.XDist);
	ept.y = (destpt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed

	aimjd = atan2((double)(destpt.y - sourpt.y), (double)(destpt.x - sourpt.x)) * 180 / pi;
	aimjd = DecisionMaking::mth_ChangeAngle(aimjd);

	Er = atan2((double)ept.y, (double)ept.x) * 180 / pi; //转化为角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);
	//处理当前位姿距目标点的距离、角度偏差end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid aimjd is:%f\n", aimjd);
	ROS_INFO("ToPostureByMilemeter_Pid Er is:%f\n", Er);

	if (abs(aimjd - dsp_status.fAngle) <= 3)
		W = 0;
	else
		W = 0.6 * Pidlow_angle(Er);
	ROS_INFO("ToPostureByMilemeter_Pid W is %d\n", W);

	static bool startflag1 = false;
	static bool startflag2 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2)
	{
		if (speedcount < 3)
			if (Ed > 0)
				Vx = 200;
			else
				Vx = -200;
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2)
	{
		if (speedcount < 6)
			if (Ed > 0)
				Vx = 450;
			else
				Vx = -450;
		else
			startflag2 = true;
	}
	Vy = 0;
	jd = 0;
	if (startflag1 && startflag2)
	{
		if (abs(Ed) > 2000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
		{
			Vx = 700; //Vx=350;
			ROS_INFO("ToPostureByMilemeter_Pid when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 2000) && (abs(Ed) > 1000))
		{
			Vx = 600; //	Vx=300;
			ROS_INFO("ToPostureByMilemeter_Pid when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 1000) && (abs(Ed) > 600))
		{
			Vx = 500; //Vx=160;
			ROS_INFO("ToPostureByMilemeter_Pid when 600< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 600) && (abs(Ed) > 400))
		{
			Vx = 400; //Vx=150;
			ROS_INFO("ToPostureByMilemeter_Pid when 400<Ed<600,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 400) && (abs(Ed) > 200))
		{
			Vx = 300; //Vx=140
			ROS_INFO("ToPostureByMilemeter_Pid when 200<Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		if (abs(Ed) <= 200)
		{
			Vx = 200; //Vx=130
			ROS_INFO("ToPostureByMilemeter_Pid when Ed<200,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	//（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	if (abs(Ed) < 150 || (abs(ept.x) < 141 && abs(ept.y) < 141))
	{
		startflag1 = false;
		startflag2 = false;
		speedcount = 0;
		stop();
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid stop because abs(Ed)<150||(abs(ept.x)<141&&abs(ept.y)<141) ToPostureByMilemeter_Pid stop\n");
		return 1;
	}
	else if ((abs(ept.x) < 120 || abs(ept.y) < 120))
	{
		startflag1 = false;
		startflag2 = false;
		speedcount = 0;
		stop();
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid stopbecause abs(ept.x)<120||abs(ept.y)<120 ToPostureByMilemeter_Pid stop\n");
		return 1;
	}
	else
		return 0;
	Control.flag_start_stop = 1; //允许电机转动
								 /******************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Er, Ed; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //机器人当前位置点距目标点的距离Ed

	double theta1, theta2;
	theta1 = (double)dsp_status.fAngle;						 //当前机器人偏离X轴角度,角度制
	theta2 = atan2((double)ept.y, (double)ept.x) * 180 / pi; //转化为角度制
	Er = DecisionMaking::mth_ChangeAngle(theta2 - theta1);	 //机器人当前位姿偏离目标点的角度Er，角度制
	Er = DecisionMaking::mth_ChangeAngle(Er);
	//处理当前位姿距目标点的距离、角度偏差end
	ROS_INFO("ToPostureByMilemeter_Pid Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid Er is:%f\n", Er);

	if (abs(Er) < 2)
		W = 0;
	else
		W = 0.3 * Pidlow_angle(Er);

	static bool startflag1 = false;
	static bool startflag2 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2)
	{
		if (speedcount < 3)
			if (Ed > 0)
				Vx = 200;
			else
				Vx = -200;
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2)
	{
		if (speedcount < 6)
			if (Ed > 0)
				Vx = 400;
			else
				Vx = -400;
		else
			startflag2 = true;
	}
	Vy = 0;
	jd = 0;
	if (startflag1 && startflag2)
	{
		if (abs(Ed) > 2000)
		{
			Vx = 600;
			ROS_INFO("ToPostureByMilemeter when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}

		else if ((abs(Ed) <= 2000) && (abs(Ed) > 1000))
		{
			Vx = 500;
			ROS_INFO("ToPostureByMilemeter when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 1000) && (abs(Ed) > 400))
		{
			Vx = 400;
			ROS_INFO("ToPostureByMilemeter when 400< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
		if (abs(Ed) <= 400)
		{
			W = 0;
			Vx = 400; //300
			ROS_INFO("ToPostureByMilemeter when Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	//（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角
	if (fabs(Ed) < 100) //是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	{
		startflag1 = false;
		startflag2 = false;
		speedcount = 0;
		ROS_INFO_STREAM("ToPostureByMilemeter() stop111111111111111111111111111111111111111111111\n");
		return 1;
	}
	else
		return 0;
	Control.flag_start_stop = 1; //允许电机转动
}

bool DecisionMaking::ToPostureByMilemeter_Pid1(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed; //定义当前位置到目标点的距离
	float jd;
	short Vx, Vy, W;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed

	//处理当前位姿距目标点的距离end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid1 Ed is:%f\n", Ed);

	Vy = 0;
	W = 0;
	jd = 0;
	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static bool startflag4 = false;
	static bool startflag5 = false;
	static bool startflag6 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5 && !startflag6)
	{
		if (speedcount < 3)
			if (Ed > 0)
				Vx = 100;//300
			else
				Vx = -100;//-300
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5 && !startflag6)
	{
		if (speedcount < 8)
			if (Ed > 0)
				Vx = 200;//400
			else
				Vx = -200;//-400
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3 && !startflag4 && !startflag5 && !startflag6)
	{
		if (speedcount < 13)
			if (Ed > 0)
				Vx = 300;//400
			else
				Vx = -300;//-400
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3 && !startflag4 && !startflag5 && !startflag6)
	{
		if (speedcount < 18)//9
			if (Ed > 0)
				Vx = 400;//500
			else
				Vx = -400;//-500
		else
			startflag4 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && !startflag5 && !startflag6)
	{
		if (speedcount < 23)
			if (Ed > 0)
				Vx = 500;//400
			else
				Vx = -500;//-400
		else
			startflag5 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && startflag5 && !startflag6)
	{
		if (speedcount < 28)//9
			if (Ed > 0)
				Vx = 600;//500
			else
				Vx = -600;//-500
		else
			startflag6 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && startflag5 && startflag6)
		/*速度真正的PID调节start*/
		Vx = 1.5*Pid_distX(Ed);//1.3
	/*速度真正的PID调节end*/

	GetNspeed(Vx, Vy, W, jd);
	if (abs(Ed) < 100 || ((abs(ept.x) < 80)))
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		startflag4 = false;
		startflag5 = false;
		startflag6 = false;
		speedcount = 0;
		if (stop())
			;
		{
			Stop();
		} //停止最好用这个
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid1() Stop \n");
		return 1;
	}
	else
		return 0;
	Control.flag_start_stop = 1; //允许电机转动
								 /*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid2(Posture pt) //通过里程计全向平移到绝对点
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed, Er; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed
	Er = atan2(ept.y, ept.x) * 180 / pi;
	Er = DecisionMaking::mth_ChangeAngle(Er);
	jd = Er - dsp_status.fAngle;
	jd = DecisionMaking::mth_ChangeAngle(jd);
	//处理当前位姿距目标点的距离、角度偏差end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid2 Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid2 Er is:%f\n", Er);
	ROS_INFO("ToPostureByMilemeter_Pid2 jd is:%f\n", jd);
	Vy = 0;
	W = 0;
	// if (fabs(Er) > 3)
	// {
	// 	W = 0.2 * Pidlow_angle(Er);
	// }
	// else
	// 	W = 0;

	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static bool startflag4 = false;
	static bool startflag5 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 3)
		{
			if (Ed > 0)
				Vx = 400;//VX1 - 300;
			else
				Vx = 400;//VX2 + 300;
		}
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 6)
		{
			if (Ed > 0)
				Vx = 900;//VX1 - 200;
			else
				Vx = 900;//VX2 + 200;
		}
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 9)
		{
			if (Ed > 0)
				Vx = 1200;//VX1 - 100;
			else
				Vx = 1200;//VX2 + 100;
		}
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 12)
		{
			if (Ed > 0)
				Vx = 1600;//VX1 ;
			else
				Vx = 1600;//VX2 ;
		}
		else
			startflag4 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && !startflag5)
	{
		if (speedcount < 15)
		{
			if (Ed > 0)
				Vx = 2000;//VX1 + 100;
			else
				Vx = 2000;//VX2 - 100;
		}
		else
			startflag5 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && startflag5)
	{
		if (speedcount < 18)
		{
			if (Ed > 0)
				Vx = 2300;//VX1 + 200;
			else
				Vx = 2300;//VX2 - 200;
		}
		else
		{
			if (fabs(Ed) > 2000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
			{
				Vx = 2000;//VX1 + 200; //Vx=350;
				ROS_INFO("ToPostureByMilemeter_Pid2 when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 2000) && (fabs(Ed) > 1000))
			{
				Vx = 1600;//VX1 + 100; //Vx=300;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 1000) && (fabs(Ed) > 600))
			{
				Vx = 1200;//VX1 ; //Vx=160;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 600< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 600) && (fabs(Ed) > 400))
			{
				Vx = 800;//VX1; //Vx=150;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 400<Ed<600,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 400) && (fabs(Ed) > 200))
			{
				Vx = 400;//VX1 - 100; //Vx=140
				ROS_INFO("ToPostureByMilemeter_Pid2 when 200<Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			if ((fabs(Ed) <= 200) && (fabs(Ed) >= 100))
			{
				W = 0;
				Vx = 200;//VX1 - 300; //Vx=130
				ROS_INFO("ToPostureByMilemeter_Pid2 when 90<Ed<200,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((fabs(Ed) <= 100) || (abs(ept.x) <= 80)) /*||(abs(ept.y)<=130)*/
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		startflag4 = false;
		startflag5 = false;
		speedcount = 0;
		if (stop())
			;
		{
			Stop();
		} //停止最好用这个
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid2() Stop \n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid6(Posture pt) //通过里程计全向平移到绝对点
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed, Er; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed
	Er = atan2(ept.y, ept.x) * 180 / pi;
	Er = DecisionMaking::mth_ChangeAngle(Er);
	jd = Er - dsp_status.fAngle;
	jd = DecisionMaking::mth_ChangeAngle(jd);
	//处理当前位姿距目标点的距离、角度偏差end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid2 Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid2 Er is:%f\n", Er);
	ROS_INFO("ToPostureByMilemeter_Pid2 jd is:%f\n", jd);
	Vy = 0;
	W = 0;
	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static bool startflag4 = false;
	static bool startflag5 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 3)
		{
			if (Ed > 0)
				Vx = 200;//VX1 - 300;
			else
				Vx = 200;//VX2 + 300;
		}
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 6)
		{
			if (Ed > 0)
				Vx = 400;//VX1 - 200;
			else
				Vx = 400;//VX2 + 200;
		}
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 9)
		{
			if (Ed > 0)
				Vx = 900;//VX1 - 100;
			else
				Vx = 900;//VX2 + 100;
		}
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 12)
		{
			if (Ed > 0)
				Vx = 1500;//VX1 ;
			else
				Vx = 1500;//VX2 ;
		}
		else
			startflag4 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && !startflag5)
	{
		if (speedcount < 15)
		{
			if (Ed > 0)
				Vx = 1800;//VX1 + 100;
			else
				Vx = 1800;//VX2 - 100;
		}
		else
			startflag5 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && startflag5)
	{
		if (speedcount < 18)
		{
			if (Ed > 0)
				Vx = 2200;//VX1 + 200;
			else
				Vx = 2200;//VX2 - 200;
		}
		else
		{
			if (fabs(Ed) > 2000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
			{
				Vx = 2000;//VX1 + 200; //Vx=350;
				ROS_INFO("ToPostureByMilemeter_Pid2 when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 2000) && (fabs(Ed) > 1000))
			{
				Vx = 1600;//VX1 + 100; //Vx=300;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 1000) && (fabs(Ed) > 600))
			{
				Vx = 1200;//VX1 ; //Vx=160;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 600< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 600) && (fabs(Ed) > 400))
			{
				Vx = 900;//VX1; //Vx=150;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 400<Ed<600,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 400) && (fabs(Ed) > 200))
			{
				Vx = 500;//VX1 - 100; //Vx=140
				ROS_INFO("ToPostureByMilemeter_Pid2 when 200<Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			if ((fabs(Ed) <= 200) && (fabs(Ed) >= 100))
			{
				W = 0;
				Vx = 300;//VX1 - 300; //Vx=130
				ROS_INFO("ToPostureByMilemeter_Pid2 when 90<Ed<200,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((fabs(Ed) <= 100) || (abs(ept.y) <= 80)) /*||(abs(ept.y)<=130)*/
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		startflag4 = false;
		startflag5 = false;
		speedcount = 0;
		if (stop())
			;
		{
			Stop();
		} //停止最好用这个
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid2() Stop \n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid3(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed, Er; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed
	Er = atan2(ept.y, ept.x) * 180 / pi;
	Er = DecisionMaking::mth_ChangeAngle(Er);
	jd = Er - dsp_status.fAngle;
	jd = DecisionMaking::mth_ChangeAngle(jd);
	//处理当前位姿距目标点的距离、角度偏差end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid2 Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid2 Er is:%f\n", Er);
	ROS_INFO("ToPostureByMilemeter_Pid2 jd is:%f\n", jd);
	Vy = 0;
	W = 0;
	// if (fabs(Er) > 3)
	// {
	// 	W = 0.2 * Pidlow_angle(Er);
	// }
	// else
	// 	W = 0;

	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static bool startflag4 = false;
	static bool startflag5 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 3)
		{
			if (Ed > 0)
				Vx = 200;//VX1 - 300;
			else
				Vx = 200;//VX2 + 300;
		}
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 6)
		{
			if (Ed > 0)
				Vx = 350;//VX1 - 200;
			else
				Vx = 350;//VX2 + 200;
		}
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 9)
		{
			if (Ed > 0)
				Vx = 500;//VX1 - 100;
			else
				Vx = 500;//VX2 + 100;
		}
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 12)
		{
			if (Ed > 0)
				Vx = 800;//VX1 ;
			else
				Vx = 800;//VX2 ;
		}
		else
			startflag4 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && !startflag5)
	{
		if (speedcount < 15)
		{
			if (Ed > 0)
				Vx = 1100;//VX1 + 100;
			else
				Vx = 1100;//VX2 - 100;
		}
		else
			startflag5 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && startflag5)
	{
		if (speedcount < 18)
		{
			if (Ed > 0)
				Vx = 1500;//VX1 + 200;
			else
				Vx = 1500;//VX2 - 200;
		}
		else
		{
			if (fabs(Ed) > 2000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
			{
				Vx = 1300;//VX1 + 200; //Vx=350;
				ROS_INFO("ToPostureByMilemeter_Pid2 when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 2000) && (fabs(Ed) > 1000))
			{
				Vx = 1000;//VX1 + 100; //Vx=300;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 1000) && (fabs(Ed) > 600))
			{
				Vx = 800;//VX1 ; //Vx=160;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 600< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 600) && (fabs(Ed) > 400))
			{
				Vx = 600;//VX1; //Vx=150;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 400<Ed<600,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 400) && (fabs(Ed) > 200))
			{
				Vx = 400;//VX1 - 100; //Vx=140
				ROS_INFO("ToPostureByMilemeter_Pid2 when 200<Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			if ((fabs(Ed) <= 200) && (fabs(Ed) >= 100))
			{
				W = 0;
				Vx = 200;//VX1 - 300; //Vx=130
				ROS_INFO("ToPostureByMilemeter_Pid2 when 90<Ed<200,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((fabs(Ed) <= 100) || (abs(ept.x) <= 80)) /*||(abs(ept.y)<=130)*/
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		startflag4 = false;
		startflag5 = false;
		speedcount = 0;
		if (stop())
			;
		{
			Stop();
		} //停止最好用这个
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid2() Stop \n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid4(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed, Er; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed
	Er = atan2(ept.y, ept.x) * 180 / pi;
	Er = DecisionMaking::mth_ChangeAngle(Er);
	jd = Er - dsp_status.fAngle;
	jd = DecisionMaking::mth_ChangeAngle(jd);
	//处理当前位姿距目标点的距离、角度偏差end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid2 Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid2 Er is:%f\n", Er);
	ROS_INFO("ToPostureByMilemeter_Pid2 jd is:%f\n", jd);
	Vy = 0;
	W = 0;
	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static bool startflag4 = false;
	static bool startflag5 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 3)
		{
			if (Ed > 0)
				Vx = 200;//VX1 - 300;
			else
				Vx = 200;//VX2 + 300;
		}
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 6)
		{
			if (Ed > 0)
				Vx = 400;//VX1 - 200;
			else
				Vx = 400;//VX2 + 200;
		}
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 9)
		{
			if (Ed > 0)
				Vx = 600;//VX1 - 100;
			else
				Vx = 600;//VX2 + 100;
		}
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 12)
		{
			if (Ed > 0)
				Vx = 900;//VX1 ;
			else
				Vx = 900;//VX2 ;
		}
		else
			startflag4 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && !startflag5)
	{
		if (speedcount < 15)
		{
			if (Ed > 0)
				Vx = 1200;//VX1 + 100;
			else
				Vx = 1200;//VX2 - 100;
		}
		else
			startflag5 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && startflag5)
	{
		if (speedcount < 18)
		{
			if (Ed > 0)
				Vx = 1500;//VX1 + 200;
			else
				Vx = 1500;//VX2 - 200;
		}
		else
		{
			if (fabs(Ed) > 2000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
			{
				Vx = 1500;//VX1 + 200; //Vx=350;
				ROS_INFO("ToPostureByMilemeter_Pid2 when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 2000) && (fabs(Ed) > 1000))
			{
				Vx = 1200;//VX1 + 100; //Vx=300;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 1000) && (fabs(Ed) > 600))
			{
				Vx = 900;//VX1 ; //Vx=160;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 600< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 600) && (fabs(Ed) > 400))
			{
				Vx = 600;//VX1; //Vx=150;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 400<Ed<600,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 400) && (fabs(Ed) > 200))
			{
				Vx = 400;//VX1 - 100; //Vx=140
				ROS_INFO("ToPostureByMilemeter_Pid2 when 200<Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			if ((fabs(Ed) <= 200) && (fabs(Ed) >= 100))
			{
				W = 0;
				Vx = 200;//VX1 - 300; //Vx=130
				ROS_INFO("ToPostureByMilemeter_Pid2 when 90<Ed<200,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((fabs(Ed) <= 100) || (abs(ept.y) <= 80)) /*||(abs(ept.y)<=130)*/
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		startflag4 = false;
		startflag5 = false;
		speedcount = 0;
		if (stop())
			;
		{
			Stop();
		} //停止最好用这个
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid2() Stop \n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByMilemeter_Pid5(Posture pt)
{
	//处理当前位姿距目标点的距离Ed（mm）、角度偏差Er(角度) **** (start)
	double Ed, Er; //定义当前位置到目标点的角度偏差、距离
	short Vx, Vy, W;
	float jd;
	Posture ept;

	ept.x = (pt.x - dsp_status.XDist);
	ept.y = (pt.y - dsp_status.YDist);
	Ed = DecisionMaking::mth_PointsDist(ept); //**1、机器人当前位置点距目标点的距离Ed
	Er = atan2(ept.y, ept.x) * 180 / pi;
	Er = DecisionMaking::mth_ChangeAngle(Er);
	jd = Er - dsp_status.fAngle;
	jd = DecisionMaking::mth_ChangeAngle(jd);
	//处理当前位姿距目标点的距离、角度偏差end

	//***测试9
	ROS_INFO("ToPostureByMilemeter_Pid2 Ed is:%f\n", Ed);
	ROS_INFO("ToPostureByMilemeter_Pid2 Er is:%f\n", Er);
	ROS_INFO("ToPostureByMilemeter_Pid2 jd is:%f\n", jd);
	Vy = 0;
	W = 0;
	// if (fabs(Er) > 3)
	// {
	// 	W = 0.2 * Pidlow_angle(Er);
	// }
	// else
	// 	W = 0;

	static bool startflag1 = false;
	static bool startflag2 = false;
	static bool startflag3 = false;
	static bool startflag4 = false;
	static bool startflag5 = false;
	static int speedcount = 0;
	speedcount++;
	if (!startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 3)
		{
			if (Ed > 0)
				Vx = 200;//VX1 - 300;
			else
				Vx = 200;//VX2 + 300;
		}
		else
			startflag1 = true;
	}
	if (startflag1 && !startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 6)
		{
			if (Ed > 0)
				Vx = 350;//VX1 - 200;
			else
				Vx = 350;//VX2 + 200;
		}
		else
			startflag2 = true;
	}
	if (startflag1 && startflag2 && !startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 9)
		{
			if (Ed > 0)
				Vx = 500;//VX1 - 100;
			else
				Vx = 500;//VX2 + 100;
		}
		else
			startflag3 = true;
	}
	if (startflag1 && startflag2 && startflag3 && !startflag4 && !startflag5)
	{
		if (speedcount < 12)
		{
			if (Ed > 0)
				Vx = 650;//VX1 ;
			else
				Vx = 650;//VX2 ;
		}
		else
			startflag4 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && !startflag5)
	{
		if (speedcount < 15)
		{
			if (Ed > 0)
				Vx = 800;//VX1 + 100;
			else
				Vx = 800;//VX2 - 100;
		}
		else
			startflag5 = true;
	}
	if (startflag1 && startflag2 && startflag3 && startflag4 && startflag5)
	{
		if (speedcount < 18)
		{
			if (Ed > 0)
				Vx = 900;//VX1 + 200;
			else
				Vx = 900;//VX2 - 200;
		}
		else
		{
			if (fabs(Ed) > 2000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
			{
				Vx = 900;//VX1 + 200; //Vx=350;
				ROS_INFO("ToPostureByMilemeter_Pid2 when Ed>2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 2000) && (fabs(Ed) > 1000))
			{
				Vx = 900;//VX1 + 100; //Vx=300;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 1000<Ed<2000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 1000) && (fabs(Ed) > 600))
			{
				Vx = 700;//VX1 ; //Vx=160;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 600< Ed<1000,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 600) && (fabs(Ed) > 400))
			{
				Vx = 500;//VX1; //Vx=150;
				ROS_INFO("ToPostureByMilemeter_Pid2 when 400<Ed<600,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			else if ((fabs(Ed) <= 400) && (fabs(Ed) > 200))
			{
				Vx = 350;//VX1 - 100; //Vx=140
				ROS_INFO("ToPostureByMilemeter_Pid2 when 200<Ed<400,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
			if ((fabs(Ed) <= 200) && (fabs(Ed) >= 100))
			{
				W = 0;
				Vx = 200;//VX1 - 300; //Vx=130
				ROS_INFO("ToPostureByMilemeter_Pid2 when 90<Ed<200,Vx is %d,Vy is %d W is:%d\n", Vx, Vy, W);
			}
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((fabs(Ed) <= 100)) /*||(abs(ept.y)<=130)*/
	{
		startflag1 = false;
		startflag2 = false;
		startflag3 = false;
		startflag4 = false;
		startflag5 = false;
		speedcount = 0;
		if (stop())
			;
		{
			Stop();
		} //停止最好用这个
		ROS_INFO_STREAM("ToPostureByMilemeter_Pid2() Stop \n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToPostureByAvoidance(Posture pt) //通过里程计前后左右平移到绝对点
{
	static bool avoidance_flag = false;
	//static int avoidance_count=0;
	double Edleft, Edright; //定义当前位置到目标点的角度偏差、距离
	double Vx, Vy, W;
	float jd;

	vector<pair<double, long>> objectAngleDist;
	double ki = 0.1; //pid 比例系数//0.03

	if(data.size())
	{
		ROS_INFO_STREAM("GET LASER DATA\n");
	}
	else
	{
		ROS_INFO_STREAM("DO NOT GET LASER DATA\n");
	}


	int returnValue = find_object(data, objectAngleDist);

	if (1 == returnValue)
	{
		avoidance_flag = true;
		ROS_INFO_STREAM("find ball right find ball right find ball right find ball right find ball right find ball right\n");
		ROS_INFO("ToPostureByAvoidance rightball_angle is:%f\n", objectAngleDist[0].first);
		ROS_INFO("ToPostureByAvoidance rightball_dist is:%d\n", objectAngleDist[0].second);
		if (objectAngleDist[0].second < 2000)
		{
			Edright = (objectAngleDist[0].second) * sin(abs(objectAngleDist[0].first) / 57.3);
			ROS_INFO("Edright is %f\n", Edright);
			if (Edright < 500)
				W = -ki * Pid_distY(Edright - 500);
			else
				W = 0;
			ROS_INFO("W by Edright is %f\n", W);
			//Vx=400;Vy=0;jd=0;
			if (Edright > 1500) //离右边球比较远，就不去避了
				avoidance_flag = false;
		}
		else
		{
			avoidance_flag = false;
		}
		Vx = 1000;//400
		Vy = 0;
		jd = 0;
	}
	if (-1 == returnValue)
	{
		avoidance_flag = true;
		ROS_INFO_STREAM("find ball left find ball left find ball left find ball left find ball left find ball left \n");
		ROS_INFO("ToPostureByAvoidance leftball_angle is:%f\n", objectAngleDist[0].first);
		ROS_INFO("ToPostureByAvoidance leftball_dist is:%d\n", objectAngleDist[0].second);
		if (objectAngleDist[0].second < 2000)
		{
			Edleft = (objectAngleDist[0].second) * sin(abs(objectAngleDist[0].first) / 57.3);
			ROS_INFO("Edleft is %f\n", Edleft);
			if (Edleft < 500)
				W = ki * Pid_distY(Edleft - 500);
			else
				W = 0;
			ROS_INFO("W by Edleft is %f\n", W);
			//Vx=400;Vy=0;jd=0;
			if (Edleft > 1500) //离左边球比较远，就不去避了
				avoidance_flag = false;
		}
		else
		{
			avoidance_flag = false;
		}
		Vx = 1000;//400
		Vy = 0;
		jd = 0;
	}
	if (2 == returnValue)
	{
		avoidance_flag = true;
		ROS_INFO_STREAM("find ball left and right find ball left and right find ball left and right find ball left and right \n");
		ROS_INFO("ToPostureByAvoidance rightball_angle is:%f\n", objectAngleDist[0].first);
		ROS_INFO("ToPostureByAvoidance rightball_dist is:%d\n", objectAngleDist[0].second);
		ROS_INFO("ToPostureByAvoidance leftball_angle is:%f\n", objectAngleDist[1].first);
		ROS_INFO("ToPostureByAvoidance leftball_dist is:%d\n", objectAngleDist[1].second);
		Edright = (objectAngleDist[0].second) * sin(abs(objectAngleDist[0].first) / 57.3);
		Edleft = (objectAngleDist[1].second) * sin(abs(objectAngleDist[1].first) / 57.3);

		ROS_INFO("Edright is %f\n", Edright);
		ROS_INFO("Edleft is %f\n", Edleft);
		if ((objectAngleDist[0].second > 2000) && (objectAngleDist[1].second > 2000))
			avoidance_flag = false;
		else if ((objectAngleDist[0].second > 2000) && (objectAngleDist[1].second < 2000))
		{
			if ((Edleft < 500) /*&&(Edright>=500)*/)
			{
				W = ki * Pid_distY(Edleft - 500);
				ROS_INFO("W by Edleft is %f\n", W);
			}
			else
			{
				W = 0;
			}
		}
		else if ((objectAngleDist[0].second < 2000) && (objectAngleDist[1].second > 2000))
		{
			if ((Edright < 500) /*&&(Edleft>=500)*/)
			{
				W = -ki * Pid_distY(Edright - 500);
				ROS_INFO("W by Edright is %f\n", W);
			}
			else
			{
				W = 0;
			}
		}
		else
		{
			if ((Edright >= 1500) && (Edleft >= 1500))
				avoidance_flag = false;
			if ((Edright >= 500) && (Edleft >= 500))
			{
				W = 0;
			}
			if ((Edright < 500) && (Edleft >= 500))
			{
				W = -ki * Pid_distY(Edright - 500);
				ROS_INFO("W by Edright is %f\n", W);
			}
			if ((Edleft < 500) && (Edright >= 500))
			{
				W = ki * Pid_distY(Edleft - 500);
				ROS_INFO("W by Edleft is %f\n", W);
			}
			if ((Edleft < 500) && (Edright < 500))
			{
				//优先避开最近的球,等到最近的球成功避开后，前方就只有另一个球了，接下来避开另一个球
				if (objectAngleDist[0].second <= objectAngleDist[1].second)
					W = -ki * Pid_distY(Edright - 500);
				else
					W = ki * Pid_distY(Edleft - 500);
			}
			//Vx=400;Vy=0;jd=0;
		}
		Vx = 1000;//400
		Vy = 0;
		jd = 0;
	}
	if (0 == returnValue)
	{
		avoidance_flag = false;
	}
	GetNspeed(Vx, Vy, W, jd);
	if (avoidance_flag == false)
		return 1;
	else
		return 0;
}

bool DecisionMaking::ToBallByVision(float Ed, float Er)
//这个函数的比例系数很重要，系数大会有超调，系数小会出现克服不了惯性动不了（特别是停下来调整角度过）
{
	short Vx, Vy, W;
	float jd;
	Er = 90.00 - Er;

	ROS_INFO("ToBallByVision Ed is:%f\n", Ed);
	ROS_INFO("ToBallByVision Er is:%f\n", Er);

	if (fabs(Er) > 2)
		W = 0.2 * Pidlow_angle((float)Er);
	else
		W = 0;
	ROS_INFO("ToBallByVision() W is:%d\n", W);
	Vy = 0;
	jd = 0.0;

	static bool startflag7 = false;
	static bool startflag8 = false;
	static bool startflag9 = false;
	static bool startflag10 = false;
	static int Nspeedcount = 0;
	Nspeedcount++;
	if (!startflag7 && !startflag8 && !startflag9 && !startflag10)
	{
		if (Nspeedcount < 3)
			if (Ed > 0)
				Vx = 200; //150
			else
				Vx = -200; //-150
		else
			startflag7 = true;
	}
	if (startflag7 && !startflag8 && !startflag9 && !startflag10)
	{
		if (Nspeedcount < 6)
			if (Ed > 0)
				Vx = 450; //220
			else
				Vx = -450; //-220
		else
			startflag8 = true;
	}
	if (startflag7 && startflag8 && !startflag9 && !startflag10)
	{
		if (Nspeedcount < 9)
			if (Ed > 0)
				Vx = 700; //220
			else
				Vx = -700; //-220
		else
			startflag9 = true;
	}
	if (startflag7 && startflag8 && startflag9 && !startflag10)
	{
		if (Nspeedcount < 12)
			if (Ed > 0)
				Vx = 1000; //220
			else
				Vx = -1000; //-220
		else
			startflag10 = true;
	}
	if (startflag7 && startflag8 && startflag9 && startflag10)
	{
		if (fabs(Ed) > 1800) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
		{
			Vx = 1000; //360
			ROS_INFO("when Ed>1800,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		else if ((fabs(Ed) <= 1800) && (fabs(Ed) > 1200)) //进入第一次识球范围，由于视觉扫描周期较长，且不稳定，速度降低些
		{
			Vx = 800; //330
			ROS_INFO("when 1200<Ed<1800,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		else if ((fabs(Ed) <= 1200) && (fabs(Ed) > 700))
		{
			Vx = 650; //300
			ROS_INFO("when 700<Ed<1200,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		else if ((fabs(Ed) <= 700) && (fabs(Ed) > 300)) //靠近机械臂持球距离330
		{
			Vx = 550; //270
			ROS_INFO("when 300<Ed<700,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		else if ((fabs(Ed) <= 300) && (fabs(Ed) > 150)) //此时球已经进入机械臂（实际上此时，视觉已经不调整角度，直接读取激光正面距离直线走过去）
		{
			Vx = 450; //240
			ROS_INFO("when 150<Ed<300,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		if ((fabs(Ed) <= 150) && (fabs(Ed) > 100)) //此时距离太小，速度太慢，将系数调大
		{
			Vx = 350; //210
			ROS_INFO("when Ed<150,Vx is %d,Vy is %d\n", Vx, Vy);
		}
		//if((fabs(Ed)<=100)&&(fabs(Ed)>50)) //此时距离太小，速度太慢，将系数调大
		//{
		//	Vx=120;//120
		//	ROS_INFO_STREAM("when Ed<100,Vx is %d,Vy is %d\n",Vx,Vy);
		//}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if ((0 < Ed) && (Ed <= 270)) //（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角						//是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	{
		startflag7 = false;
		startflag8 = false;
		startflag9 = false;
		startflag10 = false;
		Nspeedcount = 0;
		// Stop();
		ROS_INFO_STREAM("is 270 stop\n");
		ROS_INFO_STREAM("ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop\n");
		return 1;
	}
	else if (0 == Ed) //走近的过程中突然来了一个0距离，要做处理，这里是在DecisionMaking里完成
	{
		ROS_INFO_STREAM("ToBallByVision 走近的过程中突然来了一个0距离 ！！！\n");
		Stop();
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToBallByVision_new()
//这个函数的比例系数很重要，系数大会有超调，系数小会出现克服不了惯性动不了（特别是停下来调整角度）
{
	/*  */
	static bool avoidance_flag = false;
	//static int avoidance_count=0;
	double Edleft, Edright, Edforward; //定义当前位置到目标点的角度偏差、距离
	//double Vx,Vy,W;
	//float jd;

	std::vector<std::pair<double, long>> objectAngleDist;
	double ki = 0.03; //pid 比例系数

	if(data.size())
	{
		ROS_INFO_STREAM("GET LASER DATA\n");
	}
	else
	{
		ROS_INFO_STREAM("DO NOT GET LASER DATA\n");
	}

	int returnValue = find_object(data, objectAngleDist);

	if ((-1 == returnValue) || (1 == returnValue) || (2 == returnValue))
	{
		avoidance_flag = true;
		ROS_INFO("ToPostureByAvoidance rightball_angle is:%f\n", objectAngleDist[0].first);
		ROS_INFO("ToPostureByAvoidance rightball_dist is:%d\n", objectAngleDist[0].second);
		Edforward = (objectAngleDist[0].second) * cos(fabs(objectAngleDist[0].first) / 57.3);
		ROS_INFO("Edforward is %f\n", Edforward);
	}
	/*  */
	short Vx, Vy, W;
	float jd;
	float k;
	Edforward = Edforward - 700.00;

	ROS_INFO("ToBallByVision_new Edforward is:%f\n", Edforward);

	Vy = 0;
	W = 0;
	jd = 0.0;
	static bool startflag7 = false;
	static bool startflag8 = false;
	static int Nspeedcount = 0;
	Nspeedcount++;
	if (!startflag7 && !startflag8)
	{
		if (Nspeedcount < 3)
			if (Edforward > 0)
				Vx = 200;
			else
				Vx = -200;
		else
			startflag7 = true;
	}
	if (startflag7 && !startflag8)
	{
		if (Nspeedcount < 6)
			if (Edforward > 0)
				Vx = 240;
			else
				Vx = -240;
		else
			startflag8 = true;
	}
	if (startflag7 && startflag8)
	{
		if (abs(Edforward) > 3000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
		{
			if (Edforward > 0)
				Vx = 300;
			else
				Vx = -300;
			//Vx=700;
			ROS_INFO("when Edforward>3000,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((fabs(Edforward) <= 3000) && (fabs(Edforward) > 1800)) //进入第一次识球范围，由于视觉扫描周期较长，且不稳定，速度降低些
		{
			if (Edforward > 0)
				Vx = 270;
			else
				Vx = -270;
			//Vx=600;
			ROS_INFO("when 1800<Edforward<3000,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((fabs(Edforward) <= 1800) && (fabs(Edforward) > 500))
		{
			if (Edforward > 0)
				Vx = 240;
			else
				Vx = -240;
			//Vx=500;
			ROS_INFO("when 500<Edforward<1800,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((fabs(Edforward) <= 500) && (fabs(Edforward) > 20))
		{
			k = 0.4;
			if (Edforward > 0)
				Vx = k * Edforward;
			else
				Vx = k * Edforward;
			ROS_INFO("when 20<Edforward<500,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1; //允许电机转动
	if (fabs(Edforward) <= 50)	 //（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角						//是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	{
		Vx = 0;
		GetNspeed(Vx, Vy, W, jd);
		Control.flag_start_stop = 1; //允许电机转动
		startflag7 = false;
		startflag8 = false;
		Nspeedcount = 0;
		ROS_INFO("ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop ToBallByVision Stop\n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

bool DecisionMaking::ToPoleByVision(float Ed, float Er)
//这个函数的比例系数很重要，系数大会有超调，系数小会出现克服不了惯性动不了（特别是停下来调整角度）
{
	short Vx, Vy, W;
	float jd;
	float k;
	Ed = Ed - 2400.00;
	Er = 90.00 - Er;

	ROS_INFO("ToPoleByVision Ed is:%f\n", Ed);
	ROS_INFO("ToPoleByVision Er is:%f\n", Er);

	Vy = 0;
	jd = 0.0;
	if (fabs(Er) > 2)
		W = 0.2 * Pidlow_angle((float)Er);
	else
		W = 0;
	ROS_INFO("ToBallByVision() W is:%d\n", W);

	static bool startflag7 = false;
	static bool startflag8 = false;
	static int Nspeedcount = 0;
	Nspeedcount++;
	if (!startflag7 && !startflag8)
	{
		// W = 0;
		if (Nspeedcount < 3)
			if (Ed > 0)
				Vx = 250;
			else
				Vx = -250;
		else
			startflag7 = true;
	}
	if (startflag7 && !startflag8)
	{
		// W = 0;
		if (Nspeedcount < 6)
			if (Ed > 0)
				Vx = 400;
			else
				Vx = -400;
		else
			startflag8 = true;
	}
	if (startflag7 && startflag8)
	{
		if (abs(Ed) > 3000) //注意对Ed分段P控制时，交接处的V不要越变太大。否则较大扭矩损伤电机也影响运动品质。					 //根据决策需要，修改分段距离
		{
			// if (fabs(Er) > 3)
			// 	W = 0.2 * Pid_angle_new((float)Er);//0.2
			// else
			// {
			// 	W = 0;
			// }
			if (Ed > 0)
				Vx = 360;
			else
				Vx = -360;
			//Vx=700;
			ROS_INFO("when Ed>3000,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 3000) && (abs(Ed) > 1800)) //进入第一次识球范围，由于视觉扫描周期较长，且不稳定，速度降低些
		{
			// if (fabs(Er) > 3)
			// 	W = 0.2 * Pid_angle_new((float)Er);//0.2
			// else
			// {
			// 	W = 0;
			// }
			if (Ed > 0)
				Vx = 340;
			else
				Vx = -340;
			//Vx=600;
			ROS_INFO("when 1800<Ed<3000,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 1800) && (abs(Ed) > 500))
		{
			// if (fabs(Er) > 3)
			// 	W = 0.2 * Pid_angle_new((float)Er);//0.2
			// else
			// {
			// 	W = 0;
			// }
			if (Ed > 0)
				Vx = 320;
			else
				Vx = -320;
			//Vx=500;
			ROS_INFO("when 500<Ed<1800,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
		else if ((abs(Ed) <= 500) && (abs(Ed) > 20))
		{
			// if (fabs(Er) > 3)
			// 	W = 0.2 * Pid_angle_new((float)Er);//0.3
			// else
			// {
			// 	W = 0;
			// }
			if (Ed > 0)
			{
				Vx = 300;
			}
			else
			{
				Vx = -300;
			}
			ROS_INFO("when 20<Ed<500,Vx is %d,Vy is %d,W is %d\n", Vx, Vy, W);
		}
	}
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1;		   //允许电机转动
	if ((fabs(Ed) <= 10) || (Ed == -2401)) //（这是主要原因2）这里一定要限定距离停下来（根据情况可能还要调大），因为当距离很小的，哪怕走的很准，X,Y值都误差不大，但此时的偏角						//是等于X，Y与目标点坐标的差值来算的，是会变的很大的
	{
		W = 0;
		Vx = 0;
		GetNspeed(Vx, Vy, W, jd);
		Control.flag_start_stop = 1; //允许电机转动
		startflag7 = false;
		startflag8 = false;
		Nspeedcount = 0;
		ROS_INFO_STREAM("ToPoleByVision Stop ToPoleByVision Stop ToPoleByVision Stop ToPoleByVision Stop ToPoleByVision Stop\n");
		return 1;
	}
	else
		return 0;
	/*****************PID控制end******************/
}

void DecisionMaking::Stop() //停车，无返回值，即不负责判断是否停下
{
	Control.V1 = 0;
	Control.V2 = 0;
	Control.V3 = 0;
	Control.V4 = 0;
	Control.flag_start_stop = 1;
	ROS_INFO_STREAM("Stop()开环停车，无返回值停车 开环停车，无返回值停车\n");
}

bool DecisionMaking::stop() //停车，有返回值
{
	short Vx, Vy, W;
	float jd;
	Vx = 0;
	Vy = 0;
	jd = 0;
	W = 0;
	//Vx=0.4*pid_distX(-dsp_status.Vx);
	//Vy=0.3*pid_distY(-dsp_status.Vy);
	//W=0.4*pidlow_angle(-DSPW.DW);
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1;
	if ((abs(dsp_status.Vx) < 40)) //使得原地转圈和较大合线速度下停车都适用
	{
		Control.V1 = 0;
		Control.V2 = 0;
		Control.V3 = 0;
		Control.V4 = 0;
		ROS_INFO_STREAM("stop()有返回值停车，有返回值停车，有返回值停车，有返回值停车\n");
		return 1;
	}
	else
		return 0;
}

bool DecisionMaking::stop_new() //停车，有返回值
{
	short Vx, Vy, W;
	float jd;
	Vx = 0;
	Vy = 0;
	jd = 0;
	W = 0;
	//Vx=0.4*pid_distX(-dsp_status.Vx);
	//Vy=0.3*pid_distY(-dsp_status.Vy);
	//W=0.4*pidlow_angle(-DSPW.DW);
	GetNspeed(Vx, Vy, W, jd);
	Control.flag_start_stop = 1;
	if ((abs(dsp_status.DW) < 5)) //使得原地转圈和较大合线速度下停车都适用
	{
		Control.V1 = 0;
		Control.V2 = 0;
		Control.V3 = 0;
		Control.V4 = 0;
		ROS_INFO_STREAM("stop()有返回值停车，有返回值停车，有返回值停车，有返回值停车\n");
		return 1;
	}
	else
		return 0;
}
