#pragma once

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cmath>

#define PI 3.1415926

class PclTestCore
{
  private:
    std::string sub_topic_name_1_;
    std::string sub_topic_name_2_;
    std::string pub_topic_name_;
    std::string frame_name_;
    
    float alpha_1_;
    float beta_1_;
    float gamma_1_;
    float offset_x_1_;
    float offset_y_1_;
    float offset_z_1_;
    
    float alpha_2_;
    float beta_2_;
    float gamma_2_;
    float offset_x_2_;
    float offset_y_2_;
    float offset_z_2_;
    
    bool show_points_size_;
    
    ros::Publisher pub_point_cloud_;
    
    //坐标转换函数
    void transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud, float rx, float ry, float rz, float ox, float oy, float oz);
    //点云拼接函数
    void combine(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_1, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_2, const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
    //回调函数
    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr_1, const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr_2);

  public:
    PclTestCore(ros::NodeHandle &nh);
    ~PclTestCore();
    void Spin();
};

