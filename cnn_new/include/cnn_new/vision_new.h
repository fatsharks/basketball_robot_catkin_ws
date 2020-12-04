#ifndef _VISION_H_
#define _VISION_H_

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <opencv2/opencv.hpp>
#include <const_msg/object_param.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>

#define angleDist 10
#define threshValue 4000
#define max_count 5

typedef enum
{
    unkownObject,             //0
    pillar,              //1标定柱
    basketball,               //2篮球
    volleyball                //3排球
} objectType;

typedef struct
{
    objectType whatObject;
    double objectAngle;
    long distance;
} objectParameter;

extern std::vector<long> laser_data_range; //激光数据

extern cv::Point2f objectCenter;                       //目标的中心点坐标
extern objectType object_type_target;                  //目标类型结构体
extern objectParameter object_detect_result;           //目标角度距离信息结构体
extern std::map<std::string, objectType> multi_object; //目标的类型键值对

extern std::vector<std::pair<double, long>> objectAngleDistance_1; //激光找到的一组标定柱的角度位置
extern std::vector<std::pair<double, long>> objectAngleDistance_2; //激光找到的一组球的角度位置
extern std::pair<double, long> finalobjectAngleDistance_1;         //标定柱最后发送的角度距离
extern std::pair<double, long> finalobjectAngleDistance_2;         //球最后发送给的角度距离

class Vision
{
private:
    ros::NodeHandle nh;
    ros::Publisher detect_data_pub;
    ros::Subscriber laser_data_sub;
    ros::Subscriber object_target_sub;
    ros::Subscriber cnn_sub;

public:
    Vision();
    ~Vision();
    /**************************************回调函数->数据接受**************************************************/
    void cnn_callback(const darknet_ros_msgs::BoundingBoxesConstPtr &box);
    void laser_callback(const sensor_msgs::LaserScanConstPtr &laser);
    void object_target_callback(const std_msgs::Int32ConstPtr &nums);

    /// @brief 结果发送函数；将检测后的结果发送给决策包
    /// @param final_object_param 检测结果
    void send_final_result(std::pair<double, long> &final_object_param);

    /***************************************距离检测函数******************************************/
    /// @brief 标定柱专用激光测距函数，空旷无干扰条件下最远可以测3600
    /// @param data 读取的激光原始数据
    /// @param objectAngleDistance 检测到的一系列角度距离值
    /// @return 激光是否检测到标定柱
    bool find_calibration_object_by_laser_data(std::vector<long> &data, std::vector<std::pair<double, long>> &objectAngleDistance);

    /// @brief 球专用的激光测距函数，空旷有干扰情况下最远可以检测到2500
    /// @param data 读取的激光原始数据
    /// @param objectAngleDistance 检测到的一系列角度距离值
    /// @return 激光是否检测到标定柱
    bool find_nearest_object_by_laser_data_laser_ball(std::vector<long> &data, std::vector<std::pair<double, long>> &objectAngleDistance);

    /// @brief 目标检测结果确定函数；结合视觉和激光检测结果，确定出最吻合的目标方位
    /// @param objectAngleDistance 激光检测到的一系列角度距离值
    /// @param objectCenter 视觉检测到的目标中心点像素坐标值
    /// @param finalobjectAngleDistance 最后吻合的目标方位
    /// @return 激光和视觉检测的结果是否吻合
    bool find_final_object(std::vector<std::pair<double, long>> &objectAngleDistance, cv::Point2f &objectCenter, std::pair<double, long> &finalobjectAngleDistance);

    /// @brief 检测逻辑函数，根据设定好的检测逻辑分别调用相应的检测函数，并将检测结果发布出去
    void cnn_detect();
    /***************************************备用函数******************************************/
    bool find_nearest_object_by_laser_data_plus(std::vector<long> &data, std::vector<std::pair<double, long>> &objectAngleDistance);
    
    bool find_nearest_object_by_laser_data(std::vector<long> &data, std::vector<std::pair<double, long>> &objectAngleDistance);

    bool find_final_object_plus(std::vector<std::pair<double, long>> &objectAngleDistance, cv::Point2f &objectCenter, std::pair<double, long> &finalobjectAngleDistance);
};

#endif