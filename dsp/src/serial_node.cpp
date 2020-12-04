#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <serial/serial.h>
#include <time.h>
#include "dsp/serial_communication.h"

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
    init(argc, argv, "serial_node");
    serial_communication ser_process; //定义一个串口通信类
    unsigned char *pbuffer;
    unsigned char tempbuffer[rbuffersize];
    Rate r(22);
    clock_t start, end;
    while (ser_process.serial_port_init(ser) && ok())
    {
        //ser.flush();
        ser.read(tempbuffer, rbuffersize);
        //ser.flush();
        //ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!XDist is: "<<(unsigned int)tempbuffer[3]<<" "<<(unsigned int)tempbuffer[4]);

        std::cout << "tempbuffer : ";
        for (size_t i = 0; i < 16; i++)
        {
            std::cout << std::hex << " " << (unsigned int)tempbuffer[i];
        }
        std::cout << std::endl;

        pbuffer = ser_process.DSPStatus_read(tempbuffer);
        ROS_INFO_STREAM("after");
        ser_process.m_DSPStatus = ser_process.translate(pbuffer, ser_process.m_DSPStatus);
        ROS_INFO_STREAM("m_DSPStatus.XDist is: " << ser_process.m_DSPStatus.XDist);
        ROS_INFO_STREAM("m_DSPStatus.YDist is: " << ser_process.m_DSPStatus.YDist);
        ROS_INFO_STREAM("dsp x-velocity is: " << ser_process.m_DSPStatus.Vx);
        ROS_INFO_STREAM("dsp y-velocity is: " << ser_process.m_DSPStatus.Vy);
        ser_process.DSPStatus_pub(ser_process.m_DSPStatus);
        spinOnce();
        //r.sleep();//一定要去掉，不然会导致延时
    }
    return 0;
}