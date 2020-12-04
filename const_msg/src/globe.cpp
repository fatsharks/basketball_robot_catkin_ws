#include <ros/ros.h>
#include <iostream>
#include "const_msg/globe.h"



int main(int argc, char**argv)
{
    ros::init(argc, argv, "globe");
    Posture a = {100, 100, 100}, b = {100, 100, 100};
    double theta = 10;
    return 0;
}