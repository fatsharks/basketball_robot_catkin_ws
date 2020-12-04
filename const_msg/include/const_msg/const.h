#ifndef _ROBOTCUP2007_CONST_H_FILE__INCLUDED_
#define _ROBOTCUP2007_CONST_H_FILE__INCLUDED_

//常用符号定义
#define pi 3.141592653579
#define PI 3.141592653579

//机器人物理数据  轮子宽度39mm
#define Robot_Len 560	//机器人长宽(单位：mm)	
#define Robot_r	152.4	//机器人轮直径(单位：mm)

#define ALFA_L	1	//左电机反馈系数(消除电机给定静差)
#define ALFA_R	1	//右电机反馈系数(消除电机给定静差)

//class CGeneralDlg中符号定义
#define TIMENUMBER 50		//定时器时间 (单位1ms)
#define BASICTIME_IDEVENT 1 //基本定时器ID (50ms定时器)

#define MAX_WAITTIME_DSP 20		//DSP最大无消息响应时间(单位 50ms) 1s
#define MAX_WAITTIME_GVISION 20	//1#视觉最大无消息响应时间(单位 50ms) 1s
#define MAX_WAITTIME_LVISION 20	//2#视觉最大无消息响应时间(单位 50ms) 1s 
#define MAX_WAITTIME_CMMN 20	//远程通讯最大无消息响应时间(单位 50ms) 1s

#define CMMN_TIMENUMBER 300	//	定时器时间 (单位1ms) 发送本机器人状态信息时间
#define	CMMNTIME_IDEVENT 2		//基本定时器ID (50ms定时器)

#define MAX_DEVICE_LENGTH 1024

#define BLUE 0
#define YELLOW 1

#define MYSELF 0
#define CMMN 1
#define KEYBOARD 2

#define KickOff		0 //中线球
#define Throwin 	1 //边线球
#define CornerBall 	2 //角球门球
#define NBall		3 //争球(普通)
#define GateBall	4 //门球
#define FreeBall	5 //任意球
#define PenaltyBall	6 //点球

#define OURTEAM		0
#define OPPONENT	1

#define NoClock	    0
#define IsClock	    1

//以下用于GV下球 门 门柱识别

#define MIN_VALID_BLOCK_AREA		20//10

#define FLAG_PART_ANGLE				10.0f //门柱各部分中心点的最大角度误差（全景）
#define FLAG_OTHER_ABGLE			20.0f //门柱间的最大角度误差（全景）
#define MAX_LENGTH_BALL_GVISION		220   //球的最大有效距离（全景）
#define MIN_AREA_BALL_GVISION		40	  //球的最小有效面积（全景）

//以下用于LV下球 门 门柱识别
#define MIN_AREA_GATE	500 //门最小面积
//int MINCX = 15;
//int MINY = 10;
#define MIN_AREA_GATEPOLE	100 //门柱最小面积
#define MIN_AREA_BALL   50//  22  //球最小面积


#define FC_INTENTION_SHOOT_Dist  6500   /// 射门距离 
#define FC_INTENTION_SHOOT_Angle  25    //// 射门角度


#define FindObtVision2_Area   6000    ///// 障碍物面积 
#define  Vision2ToBall_Dist   1500    ////// 抢球切换距离

#define  FC_INTENTION_RETURN_Dist   3500   ////归位距离


#endif