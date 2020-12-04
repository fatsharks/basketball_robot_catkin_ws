#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <map>
#include "cnn_new/vision_new.h"

#define dataSize 1081　 //激光硬件为正负135度扫描范围

objectType object_type_target;
std::vector<long> laser_data_range;	  //激光数据
objectParameter object_detect_result; //目标角度距离信息
cv::Point2f objectCenter;			  //目标的中心点坐标

std::map<std::string, objectType> multi_object = {{"unkownObject", unkownObject}, {"pillar", pillar}, {"red_basketball", basketball}, {"reddish_brown_basketball", basketball}, {"blue_basketball", basketball}, {"red_yellow_volleyball", volleyball}, {"blue_yellow_volleyball", volleyball}}; //初始化键值对

std::vector<std::pair<double, long>> objectAngleDistance_1; //激光找到的一组标定柱的角度位置
std::vector<std::pair<double, long>> objectAngleDistance_2; //激光找到的一组球的角度位置
std::pair<double, long> finalobjectAngleDistance_1(0.0, 0); //标定柱最后发送的角度距离
std::pair<double, long> finalobjectAngleDistance_2(0.0, 0); //球最后发送给的角度距离

//全局变量和局部变量　static关键字https://blog.csdn.net/weiyuefei/article/details/51563890

Vision::Vision()
{
	cnn_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Vision::cnn_callback, this);
	laser_data_sub = nh.subscribe("/scan", 1, &Vision::laser_callback, this);
	object_target_sub = nh.subscribe("/target_type", 1, &Vision::object_target_callback, this);
	detect_data_pub = nh.advertise<const_msg::object_param>("/detect_result", 1);
}

Vision::~Vision() {}

/****************************receive image data**********************/
void Vision::cnn_callback(const darknet_ros_msgs::BoundingBoxesConstPtr &boxes)
{
	std::vector<double> area; //目标的面积大小，用于darknet判断
	std::vector<darknet_ros_msgs::BoundingBox> candidate_boxes;
	area.clear();
	candidate_boxes.clear();
	for (size_t i = 0; i < boxes->bounding_boxes.size(); i++)
	{
		auto iter = multi_object.find(boxes->bounding_boxes[i].Class);
		if (iter->second == object_type_target)
			candidate_boxes.push_back(boxes->bounding_boxes[i]);
	}
	if (candidate_boxes.size() != 0)
	{
		//ROS_INFO_STREAM("candidate box size is: " << candidate_boxes.size());
		for (size_t i = 0; i < candidate_boxes.size(); i++)
		{
			double temp = (candidate_boxes[i].xmax - candidate_boxes[i].xmin) * (candidate_boxes[i].ymax - candidate_boxes[i].ymin);
			area.push_back(temp);
		}
		std::vector<double>::iterator position = max_element(area.begin(), area.end());
		int index = position - area.begin();
		objectCenter.x = 0.5 * (candidate_boxes[index].xmax + candidate_boxes[index].xmin);
		objectCenter.y = 0.5 * (candidate_boxes[index].ymax + candidate_boxes[index].ymin);
		//ROS_INFO_STREAM("class is: " << candidate_boxes[index].Class);
		//ROS_INFO_STREAM("x-center is: " << 0.5 * (candidate_boxes[index].xmax + candidate_boxes[index].xmin));
		//ROS_INFO_STREAM("y-center is: " << 0.5 * (candidate_boxes[index].ymax + candidate_boxes[index].ymin));
	}
	else
	{
		objectCenter.x = 0.0;
		objectCenter.y = 0.0;
		//ROS_ERROR_STREAM("fail to detect object " << candidate_boxes[index].Class);
	}
}

/************************receive laser data**********************/
void Vision::laser_callback(const sensor_msgs::LaserScanConstPtr &laser)
{
	laser_data_range.clear();
	for (size_t i = 0; i < laser->ranges.size(); i++)
	{
		long temp_1 = laser->ranges[i] * 1000; //单位换算，从meter_to_micrometer
		laser_data_range.push_back(temp_1);
	}
	//ROS_INFO_STREAM("data size is: " << laser_data_range.size());
	cnn_detect();
}

/***************************receive object target*******************/
void Vision::object_target_callback(const std_msgs::Int32ConstPtr &nums)
{
	object_type_target = (objectType)(nums->data);
	//ROS_INFO_STREAM("target callback is: " << object_type_target);
}

void Vision::send_final_result(std::pair<double, long> &final_object_param)
{
	const_msg::object_param param_msg;
	param_msg.obj_angle = final_object_param.first;
	param_msg.obj_distance = final_object_param.second;
	detect_data_pub.publish(param_msg);
}

bool Vision::find_calibration_object_by_laser_data(std::vector<long> &data, std::vector<std::pair<double, long>> &objectAngleDistance)
{ //参数二有两部分
	if (!data.empty())
	{
		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i] > 4000)
				data[i] = 4000; //最大的都为4000
		}
		int start = 0;
		int end = 0;
		for (size_t i = 260; i < 821; i++)
		//激光雷达扫描最佳范围为正负70度，换算过来为[260,820]
		{
			long dataSub = data[i + 1] - data[i];
			if (dataSub < -400 && data[i + 1] > 1000 && data[i + 2] < 4000)
			{
				start = i + 1;
			}
			if (dataSub > 400 && start && data[i] > 1000 && data[i - 1] < 4000)
			{
				end = i;
			}
			if (start && end)
			{
				//if(end-start>=5 && end-start<=350)
				//ROS_INFO_STREAM("end-start" << end - start);
				if (end - start > 2 && end - start <= 350)
				{
					//auto minDist = min_element(data.begin()+start,data.begin()+end);
					long centerDist = data[(int)((start + end) / 2)];
					/*if(centerDist-*minDist<=100)
					{*/
					std::pair<double, long> temPara;
					temPara.first = 0.25 * ((int)((start + end) / 2)); //角度
					temPara.second = centerDist;					   //距离
					objectAngleDistance.push_back(temPara);
					//}
				}
				else if (end - start > 350 && end - start < 600)
				{
					//auto minDist = min_element(data.begin()+start,data.begin()+end);
					long centerDist = data[(int)((start + end) / 2)];
					/*if(centerDist-*minDist<=100)
					{*/
					std::pair<double, long> temPara;
					temPara.first = 0.25 * ((int)((start + end) / 2)); //角度
					temPara.second = centerDist;					   //距离
					objectAngleDistance.push_back(temPara);
					//}
				}
				start = 0;
				end = 0;
			}
		}
	}
	for (size_t i = 0; i < objectAngleDistance.size(); i++)
	{
		objectAngleDistance[i].first = objectAngleDistance[i].first - 45;
		ROS_INFO_STREAM("object angle: " << objectAngleDistance[i].first << " "
										 << "object distance: " << objectAngleDistance[i].second);
	}
	if (objectAngleDistance.size() == 0)
	{
		ROS_INFO_STREAM("fail to find_object_by_laser_data");
		return false;
	}
	else
	{
		ROS_INFO_STREAM("success to find_object_by_laser_data");
		return true;
	}
}

bool Vision::find_nearest_object_by_laser_data(std::vector<long> &data, std::vector<std::pair<double, long>> &objectAngleDistance)
{ //第一个有
	//ROS_INFO_STREAM("laser data size: " << data.size());
	if (!data.empty())
	{
		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i] > 3000)
				data[i] = 3000;
		}
		int start = 0;
		int end = 0;
		for (size_t i = 260; i < 821 /*data.size()-1*/; i++)
		{
			long dataSub = data[i + 1] - data[i];
			//if(dataSub<-400  && data[i+2]<2000)
			//if (dataSub < -200 && data[i + 2] < 1200)
			if (dataSub < -200 && data[i + 2] < 2000)
			//if(dataSub<-300  && data[i+2]<1200)
			{
				start = i + 1;
				//for(int count=1;count<=5;count++)
				for (int count = 1; count <= 10; count++)
				{
					if ((data[i + 1 + count] - data[i - count]) < -200 && data[i + 1 + count] < 1200)
					{
						;
					}
					else
					{
						start = 0;
						break;
					}
				}
				//ROS_INFO_STREAM("start:" << start);
			}
			//if(dataSub>400 && start && data[i-1]<2000)
			//if (dataSub > 200 && start && data[i - 1] < 1200)
			if (dataSub > 100 && start && data[i - 1] < 2000)
			//if(dataSub>300 && start && data[i-1]<1200)
			{
				end = i;
				//for(int count=1;count<=5;count++)
				for (int count = 1; count <= 10; count++)
				{
					if ((data[i + 1 + count] - data[i - count]) > 200 && data[i - count] < 1200)
					{
						;
					}
					else
					{
						end = 0;
						break;
					}
				}
				//ROS_INFO_STREAM("end:" << end);
			}
			if (start && end)
			{
				//ROS_INFO_STREAM("end-start" << end - start);
				//if(end-start>=5 && end-start<=350)
				if (end - start > 1 && end - start <= 350)

				{
					long centerDist = data[(int)((start + end) / 2)]; //计算物体中心点的坐标
					/////自己加
					if (centerDist >= 150)
					{
						/////
						std::pair<double, long> tempPair;
						tempPair.first = 0.25 * ((int)(start + end) / 2);
						tempPair.second = centerDist;
						objectAngleDistance.push_back(tempPair);
					}
					/*auto minDist = min_element(data.begin()+start,data.begin()+end); 
						long centerDist = data[(int)((start+end)/2)]; 
						if(centerDist-*minDist<=100)
						{
							pair<double,long>temPara;
							temPara.first=0.25*((int)((start+end)/2));
							temPara.second=centerDist;
							objectAngleDistance.push_back(temPara);
						}*/
				}
				if (end - start > 1 && end - start <= 550)
				//if(end-start>=5 && end-start<=550)
				{
					long centerDist = data[(int)((start + end) / 2)]; //计算物体中心点的坐标
					/////自己加
					if (centerDist < 150)
					{
						/////
						std::pair<double, long> tempPair;
						tempPair.first = 0.25 * ((int)(start + end) / 2);
						tempPair.second = centerDist;
						objectAngleDistance.push_back(tempPair);
					}
				}
				start = 0;
				end = 0;
			}
		}
	}
	//	}
	for (size_t i = 0; i < objectAngleDistance.size(); i++)
	{
		objectAngleDistance[i].first = objectAngleDistance[i].first - 45;
	}
	if (objectAngleDistance.size() == 0)
	{
		ROS_INFO_STREAM("fail to find_nearest_object_by_laser_data");
		return false;
	}
	else
	{
		ROS_INFO_STREAM("success to find_nearest_object_by_laser_data");
		return true;
	}
}

bool Vision::find_nearest_object_by_laser_data_laser_ball(std::vector<long> &data, std::vector<std::pair<double, long>> &objectAngleDistance)
{
	//ROS_INFO_STREAM("laser data size:  << data.size());
	if (!data.empty())
	{
		for (size_t i = 0; i < data.size(); i++)
		{
			if (data[i] > 3000)
				data[i] = 3000;
		}
		int start = 0;
		int end = 0;
		for (size_t i = 246; i < 840 /*data.size()-1*/; i++)
		{
			long dataSub = data[i + 1] - data[i];
			if (dataSub < -400 && data[i + 2] < 3000)
			{
				start = i + 1;
				//ROS_INFO_STREAM("start:" << start);
			}
			if (dataSub > 400 && start && data[i - 1] < 3000)
			{
				end = i;
				//ROS_INFO_STREAM("end:" << end);
			}
			if (start && end)
			{
				if (end - start >= 10 && end - start <= 350)
				{
					long centerDist = data[(int)((start + end) / 2)]; //计算物体中心点的坐标
					std::pair<double, long> tempPair;
					tempPair.first = 0.25 * ((int)(start + end) / 2);
					tempPair.second = centerDist;
					objectAngleDistance.push_back(tempPair);
					/*auto minDist = min_element(data.begin()+start,data.begin()+end); 
						long centerDist = data[(int)((start+end)/2)]; 
						if(centerDist-*minDist<=100)
						{
							pair<double,long>temPara;
							temPara.first=0.25*((int)((start+end)/2));
							temPara.second=centerDist;
							objectAngleDistance.push_back(temPara);
						}*/
				}
				start = 0;
				end = 0;
			}
		}
	}
	//	}
	for (size_t i = 0; i < objectAngleDistance.size(); i++)
	{
		objectAngleDistance[i].first = objectAngleDistance[i].first - 45;
	}
	if (objectAngleDistance.size() == 0)
	{
		ROS_INFO_STREAM("fail to find_nearest_object_by_laser_data");
		return false;
	}
	else
	{
		ROS_INFO_STREAM("success to find_nearest_object_by_laser_data");
		return true;
	}
}

bool Vision::find_final_object(std::vector<std::pair<double, long>> &objectAngleDistance, cv::Point2f &objectCenter, std::pair<double, long> &finalobjectAngleDistance)
{
	if (objectAngleDistance.size() == 0 || objectCenter.x == 0)
	{
		ROS_INFO_STREAM("objectCenter-x:" << objectCenter.x);
		ROS_INFO_STREAM("fail to find_first_basketball");
		return false;
	}
	else
	{
		ROS_INFO_STREAM("success to  find_first_basketball");
		//ROS_INFO_STREAM("number of object distance-angle groups by laser: " << objectAngleDistance.size());
		for (size_t i = 0; i < objectAngleDistance.size(); i++)
		{
			ROS_INFO_STREAM("angle of object by laser: " << objectAngleDistance[i].first << "distance of object by laser: " << objectAngleDistance[i].second);
		}
		ROS_INFO_STREAM("X-axis of object center(image): " << objectCenter.x
														   << "Y-axis of object center(image): " << objectCenter.y);
		std::vector<double> absValue;
		double objectCenterAngle;

		if (objectCenter.x < 320)
		{
			objectCenterAngle = 90 + (320 - objectCenter.x) * ((double)38 / 640);
			ROS_INFO_STREAM("angle of image center(after transfer coordination): " << objectCenterAngle);
		}
		else
		{
			objectCenterAngle = 90 - (objectCenter.x - 320) * ((double)38 / 640);
			ROS_INFO_STREAM("angle of image center(after transfer coordination): " << objectCenterAngle);
		}

		/*if(objectCenter.x <640)     
		{
			objectCenterAngle=90+(640-objectCenter.x)*((double)38/1280);
			cout<<"中心点的角度："<<objectCenterAngle<<endl;
		}else{
	         objectCenterAngle=71+(objectCenter.x-640)*((double)38/1280);
			cout<<"中心点的角度："<<objectCenterAngle<<endl;
		}*/

		for (size_t i = 0; i < objectAngleDistance.size(); i++)
		{
			absValue.push_back(abs(objectAngleDistance[i].first - objectCenterAngle));
		}
		for (size_t i = 0; i < absValue.size(); i++)
		{
			ROS_INFO_STREAM("angle difference: " << absValue[i]);
		}
		auto it = min_element(absValue.begin(), absValue.end());
		if (*it < angleDist) //当差值小于10时就找到了
		{
			size_t index = it - absValue.begin();
			finalobjectAngleDistance.first = objectAngleDistance[index].first;
			finalobjectAngleDistance.second = objectAngleDistance[index].second;
			ROS_INFO_STREAM("last angle of object" << finalobjectAngleDistance.first
												   << "last distance of object" << finalobjectAngleDistance.second);
			ROS_INFO_STREAM("success to find_final_object");
			return true;
		}
		else
		{
			ROS_INFO_STREAM("fail to find_final_object");
			return false;
		}
	}
}

/********************find_final_object__plus***********************************/
bool Vision::find_final_object_plus(std::vector<std::pair<double, long>> &objectAngleDistance, cv::Point2f &objectCenter, std::pair<double, long> &finalobjectAngleDistance)
{
	if ((objectAngleDistance.size() == 0) || objectCenter.x == 0)
	{
		ROS_INFO_STREAM("fail to find_first_object");
		return false;
	}
	else
	{
		ROS_INFO_STREAM("success to  find_first_object");
		//ROS_INFO_STREAM("共有物体角度距离：" << objectAngleDistance.size() << "对");
		for (size_t i = 0; i < objectAngleDistance.size(); i++)
		{
			ROS_INFO_STREAM("angle of object: " << objectAngleDistance[i].first
												<< "distance of object: " << objectAngleDistance[i].second);
		}
		ROS_INFO_STREAM("X-axis of object center: " << objectCenter.x
													<< "Y-axis of object center: " << objectCenter.y);
		std::vector<double> absValue;
		double objectCenterAngle;
		if (objectCenter.x < 320)
		{
			//objectCenterAngle = 180-((180-38)/2+(objectCenter.x+1)*((double)38/640));
			// objectCenterAngle=(180-38)/2+(objectCenter.x)*((double)38/640);
			objectCenterAngle = 90 + (320 - objectCenter.x) * ((double)38 / 640);
			ROS_INFO_STREAM("angle of center: " << objectCenterAngle);
		}
		else
		{
			//objectCenterAngle = 180-(90+(objectCenter.x+1-320)*((double)38/640));
			//objectCenterAngle= 90+(objectCenter.x-320)*((double)38/640);
			objectCenterAngle = 71 + (640 - objectCenter.x) * ((double)38 / 640);
			ROS_INFO_STREAM("angle of center: " << objectCenterAngle);
		}
		for (size_t i = 0; i < objectAngleDistance.size(); i++)
		{
			absValue.push_back(abs(objectAngleDistance[i].first - objectCenterAngle));
		}

		for (size_t i = 0; i < absValue.size(); i++)
		{
			ROS_INFO_STREAM("angle difference: " << absValue[i]);
		}
		auto it = min_element(absValue.begin(), absValue.end());
		if (*it < angleDist)
		{
			size_t index_1 = it - absValue.begin();
			finalobjectAngleDistance.first = objectAngleDistance[index_1].first;
			std::vector<long> distance;
			for (size_t i = 0; i < objectAngleDistance.size(); i++)
			{
				distance.push_back(objectAngleDistance[i].second);
			}
			auto it = min_element(distance.begin(), distance.end());
			size_t index_2 = it - distance.begin();
			finalobjectAngleDistance.second = objectAngleDistance[index_2].second;
			ROS_INFO_STREAM("success to find_final_object__plus");
			return true;
		}
		else
		{
			ROS_INFO_STREAM("fail to find_final_object__plus");
			return false;
		}
	}
}

void Vision::cnn_detect()
{
	static bool detect_flag = false;			 //检测到目标的标志位，决定是否跳入激光接管
	static bool complete_flag = false;			 //检测完成标志位
	static objectType last_target;				 //上次循环的目标类型
	static double ball_angle, calibration_angle; //视觉最后给出的目标角度
	static int intention_num = 0;				 //检测逻辑意图顺序号
	static int normal_count;					 //回合进行次数
	static int second_count;
	static int find_signal = 0;
	static int list[5] = {0 ,0 ,0, 0, 0};


	switch (intention_num)
	{
	case 0:
		ROS_INFO_STREAM("stage 000000000000000000");
		if (!complete_flag && (objectCenter.x == 0 && objectCenter.y == 0))
		{
			detect_flag = false;
			ROS_ERROR_STREAM("fail to detect object in main process");
		}
		else
		{
			
			intention_num = 1;
		}
		break;
	case 1:
		ROS_INFO_STREAM("stage 1111111111111111111");
		if (!(int)object_type_target)
		{
			ROS_INFO_STREAM("fail to receive target type in main process");
		}
		else
		{
			ROS_INFO_STREAM("success to receive target in main process");
			ROS_INFO_STREAM("the target type is:" << object_type_target);
			if (object_type_target == 1)
			{
				intention_num = 2; //检测标定柱
			}
			else
			{
				intention_num = 3; //检测球
			}
		}
		break;
	case 2:
		//视觉和激光联合找标定柱
		ROS_INFO_STREAM("calibration stage 2222222222222222222");
		if (laser_data_range.empty())
		{
			ROS_ERROR_STREAM("fail to receive laser data in stage 2");
		}
		else
		{
			ROS_INFO_STREAM("success to receive laser data in stage 2");
			objectAngleDistance_1.clear();
			if (!find_calibration_object_by_laser_data(laser_data_range, objectAngleDistance_1))
			{
				ROS_INFO_STREAM("fail to find calibration by laser data in main process");
				detect_flag = false;
				complete_flag = false;
				finalobjectAngleDistance_1.first = 0.0;
				finalobjectAngleDistance_1.second = 0;
				send_final_result(finalobjectAngleDistance_1); //发送标定柱角度距离
			}
			else
			{
				ROS_INFO_STREAM("success to find calibration by laser data in main process");
				if (!find_final_object(objectAngleDistance_1, objectCenter, finalobjectAngleDistance_1))
				{
					ROS_INFO_STREAM("fail to find final calibration in main process");
					detect_flag = false;
					complete_flag = false;
					finalobjectAngleDistance_1.first = 0.0;
					finalobjectAngleDistance_1.second = 0;
					send_final_result(finalobjectAngleDistance_1);
				}
				else
				{
					ROS_INFO_STREAM("success to find final calibration in main process");
					detect_flag = true;
					send_final_result(finalobjectAngleDistance_1); //发送标定柱角度距离
				}
			}
			intention_num = 0;
		}
		break;
	case 3:
		//视觉和激光联合找球
		ROS_INFO_STREAM("ball stage 333333333333333333333");
		if (laser_data_range.empty())
		{
			ROS_ERROR_STREAM("fail to receive laser data in stage 3");
		}
		else
		{
			ROS_INFO_STREAM("success to receive laser data in stage 3");
			objectAngleDistance_2.clear();
			if (!find_nearest_object_by_laser_data_laser_ball(laser_data_range, objectAngleDistance_2) && objectCenter.x && objectCenter.y)
			{
				ROS_INFO_STREAM("fail to find ball by laser data in main process");
				detect_flag = false;
				complete_flag = false;
				finalobjectAngleDistance_2.first = 0.0;
				finalobjectAngleDistance_2.second = 0;
				send_final_result(finalobjectAngleDistance_2);
				intention_num = 0;
			}
			else
			{
				ROS_INFO_STREAM("success to find ball by laser data in main process");
				//ROS_INFO_STREAM("size is: " << objectAngleDistance_2.size() << " center is: " << objectCenter.x);
				if (!find_final_object(objectAngleDistance_2, objectCenter, finalobjectAngleDistance_2))
				{
					ROS_INFO_STREAM("fail to find final ball in main process");
					detect_flag = false;
					complete_flag = false;
					finalobjectAngleDistance_2.first = 0.0;
					finalobjectAngleDistance_2.second = 0;
					send_final_result(finalobjectAngleDistance_2);
					intention_num = 0;
				}
				else
				{
					ROS_INFO_STREAM("success to find final ball in main process");
					detect_flag = true;
					send_final_result(finalobjectAngleDistance_2);
				}
			}
		}
		if (finalobjectAngleDistance_2.second <= 300 && detect_flag)
		{
			ball_angle = finalobjectAngleDistance_2.first;
			normal_count = 0;
			intention_num = 5;
		}
		break;
	case 4:
		//激光单独找标定柱
		ROS_INFO_STREAM("only laser calibration stage 444444444444444444");
		if (laser_data_range.empty())
		{
			ROS_ERROR_STREAM("fail to receive laser data in stage 4");
		}
		else
		{
			ROS_INFO_STREAM("success to receive laser data in stage 4");
			objectAngleDistance_1.clear();
			if (!find_calibration_object_by_laser_data(laser_data_range, objectAngleDistance_1))
			{
				ROS_INFO_STREAM("fail to find calibration by laser data in main process");
				normal_count++;
				detect_flag = false;
				complete_flag = false;
				normal_count++;
				if (normal_count > max_count)
				{
					intention_num = 0;
				}
			}
			else
			{
				ROS_INFO_STREAM("success to find calibration by laser data in main process");
				detect_flag = true;
				for (size_t i = 0; i < objectAngleDistance_1.size(); i++)
				{
					ROS_INFO_STREAM("laser angle candidata after find object by vision: " << objectAngleDistance_1[i].first << "laser distance candidate after find object by vision: " << objectAngleDistance_1[i].second);
					if (abs(calibration_angle - objectAngleDistance_1[i].first) < 35)
					{
						finalobjectAngleDistance_1.first = objectAngleDistance_1[i].first;
						finalobjectAngleDistance_1.second = objectAngleDistance_1[i].second;
					}
					ROS_INFO_STREAM("possibel object angle to send: " << finalobjectAngleDistance_1.first << "possible object dsitance to send: " << finalobjectAngleDistance_1.second);
					if (finalobjectAngleDistance_1.second < 2460 && finalobjectAngleDistance_1.second >= 2340)
					{
						finalobjectAngleDistance_1.second = -1;
						send_final_result(finalobjectAngleDistance_1);
						ROS_INFO_STREAM("last object angle to send: " << finalobjectAngleDistance_1.first << "last object distance to send: " << finalobjectAngleDistance_1.second);
					}
					else
					{
						send_final_result(finalobjectAngleDistance_1);
						ROS_INFO_STREAM("last object angle to send: " << finalobjectAngleDistance_1.first << "last object distance to send: " << finalobjectAngleDistance_1.second);
					}
				}
			}
			//TODO: 确定停止距离，进入重新循环
		}
		break;
	case 5:
		//激光单独找球
		ROS_INFO_STREAM("only laser ball stage 55555555555555555555555555555555");
		std::vector<std::pair<double, long>> objectAngleDistance_2_plus; //视觉最后角度匹配的多组角度位置
		if (laser_data_range.empty())
		{
			ROS_ERROR_STREAM("fail to receive laser data in stage 5");
		}
		else
		{
			ROS_INFO_STREAM("success to receive laser data in stage 5");
			objectAngleDistance_2.clear();
			if (!find_nearest_object_by_laser_data_laser_ball(laser_data_range, objectAngleDistance_2))
			{
				ROS_INFO_STREAM("fail to find ball by laser data in main process");
				detect_flag = false;
				complete_flag = false;
				normal_count++;
				if (normal_count > max_count)
				{
					intention_num = 0;
				}
			}
			else
			{
				ROS_INFO_STREAM("success to find ball by laser data in main process");
				detect_flag = true;
				for (size_t i = 0; i < objectAngleDistance_2.size(); i++)
				{
					ROS_INFO_STREAM("laser angle candidate of object: " << objectAngleDistance_2[i].first << "laser distance candidate of object: " << objectAngleDistance_2[i].second);
					if (abs(ball_angle - objectAngleDistance_2[i].first) < 30)
					{
						objectAngleDistance_2_plus.push_back(objectAngleDistance_2[i]);
					}
				}
				ROS_INFO_STREAM("ball angle is: " << ball_angle);
				if (objectAngleDistance_2_plus.size() > 0)
				{
					std::pair<double, long> min_objectAngleDistance_2 = objectAngleDistance_2_plus[0]; //距离最小的组
					for (size_t i = 0; i < objectAngleDistance_2_plus.size(); i++)
					{
						if (objectAngleDistance_2_plus[i].second <= min_objectAngleDistance_2.second)
						{
							min_objectAngleDistance_2 = objectAngleDistance_2_plus[i];
						}
					}
					ROS_INFO_STREAM("possible laser angle: " << min_objectAngleDistance_2.first << "possible laser distance: " << min_objectAngleDistance_2.second);
					ROS_INFO_STREAM("ball angle by vision: " << ball_angle);
					ROS_INFO_STREAM("angle difference between vision and laser: " << abs(ball_angle - min_objectAngleDistance_2.first));
					finalobjectAngleDistance_2.first = min_objectAngleDistance_2.first;
					finalobjectAngleDistance_2.second = min_objectAngleDistance_2.second;
					send_final_result(finalobjectAngleDistance_2);
				}
				else
				{
					ROS_INFO_STREAM("angle difference bigger than 30");
					second_count++;
					detect_flag = false;
					complete_flag = false;
					//ROS_INFO_STREAM("second count is: " << second_count);
					if (second_count > max_count)
					{
						normal_count = 0;
						second_count = 0;
						intention_num = 0;
					}
				}
			}
			if (finalobjectAngleDistance_2.second <= 60 && detect_flag)
			{
				ROS_INFO_STREAM("after <=60");
				finalobjectAngleDistance_2.first = 0;
				finalobjectAngleDistance_2.second = 0;
				send_final_result(finalobjectAngleDistance_2);
				complete_flag = true;
				intention_num = 0;
				ROS_INFO_STREAM("detect complete!");
			}
		}
		break;
	}
}