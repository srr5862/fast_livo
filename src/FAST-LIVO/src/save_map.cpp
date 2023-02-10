/*
 * @Author: songrr 1364117098@qq.com
 * @Date: 2023-02-11 01:35:40
 * @LastEditors: songrr 1364117098@qq.com
 * @LastEditTime: 2023-02-11 03:07:42
 * @FilePath: /FAST-LIVO/src/save_map.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <common_lib.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <mutex>

using namespace std;

mutex mutex_buffer;
PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
int j = 0;
void map_cbk(const sensor_msgs::PointCloud2::Ptr &msg){
    PointCloudXYZI::Ptr points(new PointCloudXYZI());
    pcl::fromROSMsg(*msg,*points);
    int size = points->points.size();
    for(int i = 0; i < size; i++){
        cloud->points.push_back(points->points[i]);
        j++;
    }
    cloud->height = 1;
    cloud->width = j; 
    cout << cloud->size() <<endl;  
}

void save_global_map(){
    pcl::PCDWriter writer;
    writer.write("/home/srr/test_save.pcd",*cloud);
}

int main(int argc,char **argv){
    ros::init(argc,argv,"save_map");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/cloud_registered",100,map_cbk);
    ros::spin();
    save_global_map();
    
}

