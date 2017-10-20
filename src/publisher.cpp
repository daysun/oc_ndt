#include "ros/ros.h"
#include "std_msgs/String.h"
#include<pcl/point_cloud.h>
#include<pcl_ros/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include <sstream>
#include <pcl/filters/filter.h>

bool loadCloud(std::string &filename,pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  std::cout << "Loading file " << filename.c_str() << std::endl;
  //read cloud
  if (pcl::io::loadPCDFile(filename, *cloud))
  {
    std::cout << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
    return false;
  }
  //remove NaN Points
  std::vector<int> nanIndexes;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
  std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud2>("publisher/cloud_fullSys", 1000);
   sensor_msgs::PointCloud2 output;
   std::string cloud_path("src/oc_ndt/data/test.pcd");
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

   loadCloud(cloud_path,cloud);
   pcl::toROSMsg(*cloud,output);
   ros::Rate loop_rate(1);
       while(n.ok()){
            chatter_pub.publish(output);
            loop_rate.sleep();
       }
  return 0;
}
