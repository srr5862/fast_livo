/*
 * @Author: songrr 1364117098@qq.com
 * @Date: 2023-02-08 03:18:59
 * @LastEditors: songrr 1364117098@qq.com
 * @LastEditTime: 2023-02-08 03:45:29
 * @FilePath: /FAST-LIVO/src/point2customMsg.cpp
 */
#include <stdio.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>

using namespace std;

void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg){
    cout << msg->height <<endl;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"point2customMsg");
    ros::NodeHandle nh;
    ROS_INFO("start node");
    ros::Subscriber Sub_PointCloud2 = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar",100,PointCloudCallBack);
    ros::spin();

    return 0;
}