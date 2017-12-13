#include<ros/ros.h>
#include<iostream>
#include <stdio.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include "std_msgs/String.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>

//receive a file and use VoxelGrid to generate a downsampled file

/*以下为滤波函数，首先需要将sensor_msg::PointCloud2格式转化成pcl::PCLPointCloud2格式
 * 然后再使用VoxelGrid滤波 */
void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& msg){
    std::cout<<"chatterCallbackok\n";
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
        pcl_conversions::toPCL(*msg, *cloud);
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

        pcl::PCLPointCloud2 msg_filtered;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
         //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = boost::make_shared <PointT> (); //

        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloudPtr);                       //设置需要过滤的点云给滤波对象
        sor.setLeafSize(0.02,0.02,0.02);               //设置滤波时创建的体素大小为2cm立方体，通过设置该值可以直接改变滤波结果，值越大结果越稀疏
        sor.filter(msg_filtered);                   //执行滤波处理，存储输出msg_filtered
        pcl::fromPCLPointCloud2 (msg_filtered, *cloud_filtered);
        pcl::PCDWriter writer;
         writer.write<pcl::PointXYZ> ("src/oc_ndt/data/downsampled.pcd", *cloud_filtered, false);
         std::cout<<"ok\n";

}

int main(int argc, char** argv)
   {
     ros::init(argc, argv, "voxel_grid");
     ros::start();
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe ("publisher/cloud", 1000, chatterCallback);
     ros::spin();
     ros::shutdown();
     return 0;
   }
