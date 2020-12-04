#include <ros/ros.h>
#include <iostream>
#include <decisionmaking/decisionmaking.h>

#define TIAOLEFT 2
#define TIAORIGHT 1
#define ZILEFT 1
#define ZIRIGHT 2

using namespace std;
using namespace ros;

int main(int argc, char**argv)
{
    init(argc, argv, "main_control");
    int normalNum = atoi(argv[1]);
    int placeNum = atoi(argv[2]);
    DecisionMaking decisionmaking((uint8_t)normalNum, (uint8_t)placeNum);
    //decisionmaking.Normal_num = 3;
    //decisionmaking.place_num  = ZILEFT;
    spin();    
    return 0;
}