#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <assert.h>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include "robot.h"
#include <vector>



using namespace Eigen;
using namespace std;
using namespace daysun;
typedef multimap<string,daysun::OcNode *>  MAP_INT_MORTON_MULTI;
typedef multimap<string,daysun::OcNode *>::iterator iterIntNode;
RobotSphere robot(0.5); //radius--variable--according to the range of map
daysun::TwoDmap map2D(robot.getR());
ros::Publisher marker_pub,change_pub;

void uniformDivision(const Vec3 cloudFirst, const pcl::PointXYZ temp,const float r,bool change){
    string xy_belong,z_belong,morton_xy,morton_z;
    if(temp.x > cloudFirst.x){
        if(temp.y >cloudFirst.y)
            xy_belong = "A";
        else
            xy_belong = "B";
    }else{
        if(temp.y >cloudFirst.y)
            xy_belong = "C";
        else
            xy_belong = "D";
    }
    if(temp.z > cloudFirst.z) z_belong = "U"; //up
    else z_belong = "D"; //down
    int nx = (int)ceil(float(abs(temp.x-cloudFirst.x)/r)); // belong to which line
    int ny = (int)ceil(float(abs(temp.y-cloudFirst.y)/r)); //belong to which column
    int nz = (int)ceil(float(abs(temp.z-cloudFirst.z)/r)); //belong to which height
    int morton = countMorton(nx,ny); //count morton code-xy
    morton_xy = stringAndFloat( xy_belong, morton);
    morton_z =stringAndFloat( z_belong , nz);   

    //for change-record which xy have been changed
    if(change){
        if(map2D.changeMorton_list.size() != 0){
            list<string>::iterator it = find(map2D.changeMorton_list.begin(), map2D.changeMorton_list.end(), morton_xy);
            if (it == map2D.changeMorton_list.end()){ //not find
                map2D.changeMorton_list.push_back(morton_xy);
            }
        }else{
            map2D.changeMorton_list.push_back(morton_xy);
        }
//        cout<<"change morton: "<<morton_xy<<","<<morton_z<<endl;
    }

    if(map2D.map_xy.count(morton_xy) == 0){ //not found
        //add new node
        daysun::OcNode * node = new daysun::OcNode();
        node->lPoints.push_back(temp);
        node->morton = morton_xy;
        node->z = morton_z;
        map2D.map_xy.insert(MAP_INT_MORTON_MULTI::value_type(morton_xy,node));
        map2D.map_z.insert(MAP_INT_MORTON_MULTI::value_type(morton_z,node));
        map2D.morton_list.push_back(morton_xy);       
    }else{//find        
                    iterIntNode beg = map2D.map_xy.lower_bound(morton_xy);
                     iterIntNode end = map2D.map_xy.upper_bound(morton_xy);
                     bool found = false;
                     while(beg != end){
                         if( (beg->second)->z.compare(morton_z) == 0 ){
                             (beg->second)->lPoints.push_back(temp);
                             found = true;
                             break;
                         }
                         ++beg;
                     }
                     if(!found){
                         daysun::OcNode * node = new daysun::OcNode();
                         node->lPoints.push_back(temp);
                         node->morton = morton_xy;
                         node->z = morton_z;
                         map2D.map_xy.insert(MAP_INT_MORTON_MULTI::value_type(morton_xy,node));
                         map2D.map_z.insert(MAP_INT_MORTON_MULTI::value_type(morton_z,node));
//                         map2D.morton_list.push_back(morton_xy);                    
                     }
    }
}

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr & my_msg)
{
    cout<<"receive ";
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*my_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    cout<<temp_cloud->points.size()<<endl;
    map2D.cloudFirst.x = temp_cloud->points[0].x;
    map2D.cloudFirst.y = temp_cloud->points[0].y;
    map2D.cloudFirst.z = temp_cloud->points[0].z;
    for (int i=1;i<temp_cloud->points.size();i++)
    {
        uniformDivision(map2D.cloudFirst,temp_cloud->points[i],robot.getR(),false);
     }
    cout<<"division done.\n";
    cout<<"morton size: "<<map2D.morton_list.size()<<endl;
    map2D.create2DMap();
    map2D.showInital(marker_pub);
    cout<<"initial done\n";
}

///change points
void changeCallback(const sensor_msgs::PointCloud2::ConstPtr & my_msg){
     cout<<"receiveChange\n";
     pcl::PCLPointCloud2 pcl_pc2;
     pcl_conversions::toPCL(*my_msg,pcl_pc2);
     pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

     map2D.changeMorton_list.clear();
     for (int i=1;i<temp_cloud->points.size();i++)
     {
         uniformDivision(map2D.cloudFirst,temp_cloud->points[i],robot.getR(),true);
      }
     cout<<"change division done\n";
     cout<<"change morton size: "<<map2D.changeMorton_list.size()<<endl;
     map2D.change2DMap();
     map2D.showInital(change_pub,1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fullSys_listener");
  ros::start();
  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", INT_MAX);
  change_pub = n.advertise<visualization_msgs::Marker>("change_marker", INT_MAX);
  ros::Subscriber sub = n.subscribe("publisher/cloud_fullSys", INT_MAX, chatterCallback); //initial
  ros::Subscriber sub_change = n.subscribe("publisher/cloud_change", INT_MAX,changeCallback);//change
  ros::spin();
  ros::shutdown();
  return 0;
}

