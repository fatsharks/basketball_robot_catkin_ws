#include <ros/ros.h>
#include <iostream>
#include "const_msg/globe.h"
#include "const_msg/dsp_to_pc.h"
#include "const_msg/pc_to_dsp.h"
#include "const_msg/object_param.h"
#include <std_msgs/Int32.h>
#include "cnn_new/vision_new.h"
#include "dsp/serial_communication.h"
#include <sensor_msgs/LaserScan.h>

#define angle_ball        0
#define StartW1			  10
#define StartW2			  15
#define angle_Wmin		  15
#define aim_Wmin		  5

 //pid控制变量结构体。公式：u(k)=Kp*(err(k)+Ki*求和err(j)+Kd*(err(k)-err(k-1)))
typedef struct PID_struct{
    float set_value;              //定义设定值
	float actual_value;           //定义实际值
	float err;                   //定义偏差值
	float err_last;              //定义上一个偏差值
	float Kp,Ki,Kd;              //定义比例、积分、微分系数
	float input_value;            //定义控制执行器的输入变量
	float integral;              //定义积分值
}PID;

class DecisionMaking
{
private:
    ros::NodeHandle nh;
    ros::Subscriber detect_sub, serial_sub, laser_sub;
    ros::Publisher  detect_pub, serial_pub;

public:
    PID pid_angle;//定义一个角度pid结构体
    PID pid_distx;//定义一个x方向的距离pid结构体
    PID pid_disty;//定义一个y方向的距离pid结构体

	std::vector<long> data;						//激光数据
    objectParameter the_object;				//捡球类型
    std::pair<double, long>objectAngleDistance_Y;//接收到找到球的信息
    DSPStatus dsp_status;					//接收到的dsp状态
    DSPControl Control;  					//DSP控制信息

    /********************当前信息start********************************/	
	unsigned char Normal_num;   //TODO: 由roslaunch文件写入参数　决策号，即不同回合决策选择标志
	unsigned char place_num;    //TODO:　同上　1是向右出发，y取正值；2是向左出发，y取负值

    DecisionMaking(unsigned char _normalNum, unsigned char _placeNum);
    ~DecisionMaking();
    /********************ROS通信*****************************/
    void detect_callback(const const_msg::object_paramConstPtr& obj_param);
    void serial_callback(const const_msg::dsp_to_pcConstPtr& dsp_state);
	void laser_callback(const sensor_msgs::LaserScanConstPtr& laser);

	void object_pub(objectParameter& obj);
	//发布找球命令
	void control_pub(DSPControl& ctrl);
	//发布底层控制命令

	/*********************PID部分*****************************/
    void PidAngle_init(float kp,float ki,float kd);         //初始化角度pid
	void PidDistX_init(float kp,float ki,float kd);         //初始化相对X轴距离pid
	void PidDistY_init(float kp,float ki,float kd);         //初始化相对Y轴距离pid
	float Pidlow_angle(float angle);//慢速角度调整，快速移动时用
	float Pid_angle(float angle);//角度PID函数，相对角度
	float Pid_angle_new(float angle);
	float Pid_distX(float dist);//距离PID函数，相对X轴距离
	float Pid_distY(float dist);//距离PID函数，相对Y轴距离
	float Pid_realize(float set,float actual);//常规PID函数,2016年没用

    /*********************** 基本动作函数********************/	
	//2016下面的这些动作函数不建议使用全局变量，因为会与DSPStatus有冲突。
	//2016short Vx,Vy,W;//Vx,Vy是以出发点为原点的直角坐标系，车体在X/Y方向的实际速度，单位mm/s，W为车体自身旋转角速度，单位角度/s
	//2016double Er,Ed,Edx,Edy;     //定义以出发点为原点的直角坐标系的当前位置到目标点的角度偏差，距离偏差，X轴、Y轴距离偏差
	//2016double theta1;//定义机器人当前偏离X轴角度,角度制，其值一般是接收到的车体旋转角度数据DSPstatus.fAngle
	//2016double theta2;//定义机器人要对准目标点需要偏离X轴的角度，角度制, 其值一般是double atan2(double y,double x) 返回原点至点(x,y)的方位角，即与 x 轴的夹角。
	void GetNspeed(short Vx,short Vy, double W,float jd);//轮速转换函数，以“出发点”为原点的直角坐标系上的车体速度转换为以“车体中心”为原点的坐标系上的车体速度，DSP会再换算得到轮子的给定转速。"jd"表示角度，轮速正负（即运动方向）与期望角度有关
	double mth_PointsDist(Posture a, Posture b);
	double mth_PointsDist(Posture a);
	double mth_ChangeAngle(double theta);

	void ToFindObjectByturnleft();			//原地旋转，左旋
	void ToFindObjectByturnright();			//原地旋转，右旋
	void ToFindObjectByturnleft_high();
	void ToFindObjectByturnright_high();
	void ToFindObjectByturnleftlow();
	void ToFindObjectByturnrightlow();
	void robotback();						//后退（开环）
	void robotbacklow();					//后退（开环）速度略低 回位时用
	void robotbacktoolow();					//后退（开环）速度极低 投篮时用
	void robotforward();					//前进（开环）
	void robotforwardlow();					//前进（开环）速度略低 中圈避障后找球
	void robotforwardtoolow();				//前进（开环）速度极低 投篮时用
	void robotstraightright();				//横向右移
	void robotstraightleft();				//横向左移
	void robotstraightrightlow();			//横向右移 速度略低 bat3三分线向下切找球
	void robotstraightleftlow();			//横向左移 速度略低 bat3三分线向下切找球
	void robot45back();						//45度后退 bat3捡完底角球退入场内
	void robot45backlow();					//45度后退 速度很低 pass1中圈向下移找球
	void robotfu45back();					//-45度后退 bat3捡完底角球退入场内
	void robotfu45backlow();				//-45度后退 速度很低 pass1中圈向下移找球    
    
	bool ToPostureByMilemeter_Pid(Posture sourpt,Posture destpt);//pid算法，通过弧线前进到绝对点，只能在出发时或fAngle=0时调用（pid控制的只是角度，距离只是使用了简单的P控制）
	bool ToAngleByMPU6050(Posture pt);//通过陀螺仪调整自身姿态角度以对准某点
	bool ToAngleByPosture(Posture pt);//旋转对准某点（新加的）
	bool ToAngleByPosture2(Posture pt);
	bool ToAngleByMPU(double Angle);//通过陀螺仪调整自身姿态角度
	bool ToAngleForHomeByMPU6050();//通过陀螺仪的数据，调整角度，走折线，快速回位
	bool ToPostureByMilemeter(Posture pt);//开环速度前进，对准确度要求比较高时使用
	bool ToPostureByMilemeter_Pid1(Posture pt);//pid算法，车头正对前方到达绝对点，速度较快，争取时间!!!!!!调用此函数需要先对准角度!!!!!!。
	bool ToPostureByMilemeter_Pid2(Posture pt);//pid算法，按不同方向运动函数，通过函数定义里jd值的变化，可按照指定角度移动到绝对点。jd=0与ToPostureByMilemeter_Pid1(Posture pt)相同，jd=90可用于测量Y轴里程。
	bool ToPostureByMilemeter_Pid3(Posture pt);//用于车头朝向目标点行进的情况，预留冗余误差较大
	bool ToPostureByMilemeter_Pid4(Posture pt);//专门用于到达中圈内ptD点而写
	bool ToPostureByMilemeter_Pid5(Posture pt);//专门为投篮回合2回到中圈定点而写
	bool ToPostureByMilemeter_Pid6(Posture pt);//专门为投篮2跑点而写，根据pid2改编

	bool ToAimBallByVision();//视觉转向对准球或杆
	bool ToBallByVision(float Ed,float Er);//通过视觉提供的距离目标的Ed、Er到目标,对准球后直接前进拾球
	bool ToPoleByVision(float Ed,float Er);//视觉对准杆后,前进到距离2.5m的地方
	bool ToBallByVision_new();//识别球的过程中自动调整到理想的角度和距离
	
	void Stop();                             //发送0速度，车轮抱死，用于缓慢停车
	bool stop();							//PID调节给出反向速度，用于紧急停车
	bool stop_new();
	
	bool find_object_by_laser(std::vector<long> &data, std::vector<std::pair<double, long>> &objectAngleDist);//激光识别前方障碍球
	int find_object(std::vector<long> &data, std::vector<std::pair<double,long>> &objectInfo);//判断左右两边最近的障碍球
	bool ToPostureByAvoidance(Posture pt);  //避障走定点，坐标没有用于计算，只是为了显示避障的前进方向，避障钱需要给出角度对准
    
    /**************控制方案的接口函数**************/
	void Strategy();	//控制策略选择函数，这里是选择哪个回合
    /********控制方案函数，这里指每个回合的控制策略start********/
	void Normal_pass0();		//激光摄像头通信测试
	void Normal_pass1();		//传球-回合1
	void Normal_pass2();		//传球-回合2
	void Normal_pass3();		//传球-回合3
	void Normal_AtoB_test();	//定点移动测试
	void Normal_Pttest();		//陀螺仪旋转测试
	void Normal_position();		//车体直行测试
	void Normal_avoidance();	//避障测试
	void Normal_return_test();	//回位测试
	void Normal_bat1_new();		//新投篮-回合1
	void Normal_bat2_new();		//新投篮-回合2
	void Normal_bat3_new();		//新投篮-回合3
    void Normal_test();
	void challenge_competition();//挑战赛
};


