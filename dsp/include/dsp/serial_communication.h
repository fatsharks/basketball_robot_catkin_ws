#ifndef _SERIAL_COMMUNICATION_H_
#define _SERIAL_COMMUNICATION_H_

#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <const_msg/dsp_to_pc.h>
#include <const_msg/pc_to_dsp.h>

#define rbuffersize 16
#define wbuffersize 12

extern serial::Serial ser; //声明一个串口类对象
/********************串口数据接受发送结构体*********************/
/// @brief 串口数据接受结构体
/// @param flag_start_stop 允许电机转动的标志位
/// @param V1 四个轮速
/// @param SendData_state 抬杆、弹球等动作位
typedef struct tagDSPSend
{
    unsigned char flag_start_stop;
    short V1, V2, V3, V4;
    unsigned char SendData_state;
} DSPControl; //发送数据结构

/// @brief 串口数据发送结构体
/// @param XDist x y坐标轴的位移
/// @param Vx x y坐标轴的速度
/// @param DW 通过陀螺仪信息计算的实际ｚ轴角速度 在串口节点完成计算
/// @param RecData_state 光电感应开关，检测球是否已经在位
typedef struct tagDSPRev
{
    int XDist, YDist;
    float fAngle;
    short Vx, Vy;
    short DW;                    
    unsigned char RecData_state; 
} DSPStatus;                     

enum ArmState
{
    donothing = 0,
    arm_up = 1,
    push_ball_high = 2,
    push_ball = 3,
    push_ball_low = 4,
    pick_ball = 5,
    turning_off = 6,
    turning_on = 7,
    armup_gameover = 8
}; //抬杆　弹球等控制指令

/************************串口通信类******************************/
class serial_communication
{
private:
    ros::NodeHandle nh;
    ros::Subscriber control_sub;
    ros::Publisher  status_pub;

public:
    DSPControl m_DSPControl;
    DSPStatus  m_DSPStatus;
    
    bool receive_flag;  //接受串口发上来的标志位
    bool send_flag;     //向串口下发的标志位
    serial_communication();
    ~serial_communication();
    /****************************数据回调函数********************************/
    void controlCallback(const const_msg::pc_to_dspConstPtr& control_msg);
    /****************************数据发布函数********************************/
    void DSPStatus_pub(DSPStatus& pDSPStatus);
    /**************************串口初始化函数*********************************/
    bool serial_port_init(serial::Serial& serialport);          
    /****************************读入数据***********************************/
    unsigned char* DSPStatus_read(unsigned char* tempbuffer);
    /*****************************校验数据**********************************/
    unsigned short checksum(unsigned char *ptr);
    /*****************************翻译数据**********************************/
    DSPStatus& translate(unsigned char *databuff, DSPStatus &pDSPStatus); 
    /*****************************写入数据**********************************/
    bool DSPControl_write(serial::Serial& serialport, DSPControl& pDSPControl);
    /*****************************启动DSP*********************************/
    bool OnDSPStartCommunication(serial::Serial& serialport);
    /*****************************停止DSP*********************************/
    bool OnDSPStopCommunication(serial::Serial& serialport);
    /************************角度调整到(-180,+180]**************************/
    double mth_ChangeAngle(double theta);

};

#endif
