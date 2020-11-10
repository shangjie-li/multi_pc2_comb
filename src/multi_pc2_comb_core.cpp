#include "multi_pc2_comb_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh){
    nh.param<std::string>("sub_topic_name_1", sub_topic_name_1_, "/ls_left/lslidar_point_cloud");
    nh.param<std::string>("sub_topic_name_2", sub_topic_name_2_, "/ls_right/lslidar_point_cloud");
    nh.param<std::string>("pub_topic_name", pub_topic_name_, "/lslidar_combined");
    nh.param<std::string>("frame_name", frame_name_, "/ls_comb");
    
    nh.param<float>("alpha_1", alpha_1_, 0);
    nh.param<float>("beta_1", beta_1_, 0);
    nh.param<float>("gamma_1", gamma_1_, 0);
    nh.param<float>("offset_x_1", offset_x_1_, 0);
    nh.param<float>("offset_y_1", offset_y_1_, 0);
    nh.param<float>("offset_z_1", offset_z_1_, 0);
    
    nh.param<float>("alpha_2", alpha_2_, 0);
    nh.param<float>("beta_2", beta_2_, 0);
    nh.param<float>("gamma_2", gamma_2_, 0);
    nh.param<float>("offset_x_2", offset_x_2_, 0);
    nh.param<float>("offset_y_2", offset_y_2_, 0);
    nh.param<float>("offset_z_2", offset_z_2_, 0);
    
    nh.param<bool>("show_points_size", show_points_size_, false);
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point_cloud_1_(nh, sub_topic_name_1_, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point_cloud_2_(nh, sub_topic_name_2_, 10);
    pub_point_cloud_ = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name_, 1);
    
    std::cout<<"successfully subscribed:"<<sub_topic_name_1_<<std::endl;
    std::cout<<"successfully subscribed:"<<sub_topic_name_2_<<std::endl;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_point_cloud_1_, sub_point_cloud_2_);
    sync.registerCallback(boost::bind(&PclTestCore::point_cb, this, _1, _2));
    
    ros::spin();
}

PclTestCore::~PclTestCore(){}

void PclTestCore::Spin(){
}

//坐标转换函数
void PclTestCore::transform(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud, float a, float b, float g, float ox, float oy, float oz){
    a = a * PI / 180;
    b = b * PI / 180;
    g = g * PI / 180;
        
    float t11 = cos(a) * cos(b);
    float t12 = cos(a) * sin(b) * sin(g) - sin(a) * cos(g);
    float t13 = cos(a) * sin(b) * cos(g) + sin(a) * sin(g);
        
    float t21 = sin(a) * cos(b);
    float t22 = sin(a) * sin(b) * sin(g) + cos(a) * cos(g);
    float t23 = sin(a) * sin(b) * cos(g) - cos(a) * sin(g);
        
    float t31 = - sin(b);
    float t32 = cos(b) * sin(g);
    float t33 = cos(b) * cos(g);
    
    *out_cloud = *in_cloud;
//pragma omp for语法是OpenMP的并行化语法，即希望通过OpenMP并行化执行这条语句后的for循环，从而起到加速效果
#pragma omp for
    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        float x = in_cloud->points[i].x;
        float y = in_cloud->points[i].y;
        float z = in_cloud->points[i].z;
        
        out_cloud->points[i].x = t11 * x + t12 * y + t13 * z + ox;
        out_cloud->points[i].y = t21 * x + t22 * y + t23 * z + oy;
        out_cloud->points[i].z = t31 * x + t32 * y + t33 * z + ox;
    }
}

//点云拼接函数
void PclTestCore::combine(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_1, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_2, const pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud){
    *out_cloud = *in_cloud_1 + *in_cloud_2;
}

//回调函数point_cb
void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr_1, const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr_2){
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_ptr_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_ptr_2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*in_cloud_ptr_1, *current_pc_ptr_1);
    pcl::fromROSMsg(*in_cloud_ptr_2, *current_pc_ptr_2);
    
    if (show_points_size_)
    {
        std::cout<<"size of "<<sub_topic_name_1_<<" is "<<current_pc_ptr_1->points.size()<<std::endl;
        std::cout<<"size of "<<sub_topic_name_2_<<" is "<<current_pc_ptr_2->points.size()<<std::endl;
    }
    
    //坐标转换函数
    transform(current_pc_ptr_1, transformed_pc_ptr_1, alpha_1_, beta_1_, gamma_1_, offset_x_1_, offset_y_1_, offset_z_1_);
    transform(current_pc_ptr_2, transformed_pc_ptr_2, alpha_2_, beta_2_, gamma_2_, offset_x_2_, offset_y_2_, offset_z_2_);
    
    //点云拼接函数
    combine(transformed_pc_ptr_1, transformed_pc_ptr_2, output_pc_ptr);
    
    if (show_points_size_)
    {
        std::cout<<"size of "<<pub_topic_name_<<" is "<<output_pc_ptr->points.size()<<std::endl;
        std::cout<<std::endl;
    }
    
    sensor_msgs::PointCloud2 out_cloud_ptr;
    pcl::toROSMsg(*output_pc_ptr, out_cloud_ptr);
    
    out_cloud_ptr.header = in_cloud_ptr_1->header;
    out_cloud_ptr.header.frame_id = frame_name_;
    pub_point_cloud_.publish(out_cloud_ptr);
}

