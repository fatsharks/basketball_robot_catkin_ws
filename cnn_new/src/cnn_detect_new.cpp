#include <ros/ros.h>
#include <iostream>
#include <stdlib.h>
#include "cnn_new/vision_new.h"
#include "const_msg/object_param.h"

using namespace std;
using namespace ros;
using namespace cv;

int main(int argc, char **argv)
{
	init(argc, argv, "detecting");
	Vision Process;
	Rate r(30);
	while (ok())
	{
		spinOnce();
	}
	return 0;
}
