#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <assert.h>
#include <cstddef>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "octomap_ros/Id_PointCloud2.h"
#include "octomap_ros/loopId_PointCloud2.h"
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include "robot.h"
#include <vector>
#include <visualization_msgs/Marker.h>
 #include <visualization_msgs/MarkerArray.h>


using namespace Eigen;
using namespace std;
using namespace daysun;


typedef multimap<string,daysun::OcNode *>  MAP_INT_MORTON_MULTI;
typedef multimap<string,daysun::OcNode *>::iterator iterIntNode;
RobotSphere robot(1); //radius--important
daysun::TwoDmap map2D(robot.getR());
ros::Publisher marker_pub;

void uniformDivision(const Vec3 cloudFirst, const pcl::PointXYZ temp,const float r){
    string xy_belong,z_belong,morton_xy,morton_z;
//    MAP_INT_MORTON_MULTI map_xy= map2D.map_xy,map_z = map2D.map_z;
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
//    cout<<morton_xy<<","<<morton_z<<endl;
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

bool create2DMap(TwoDmap & map2D){
    //get all the mortons-new cell, map.push_back
    list<string>::iterator itor = map2D.morton_list.begin();
        while(itor!=map2D.morton_list.end())
        {
            Cell * cell = new Cell(*itor);
            map2D.cell_list.push_back(cell);
//            cout<<"morton: "<<*itor<<endl;
            //for each morton
            //---find the nodes, count the u,c, drop the points inside
            //---determine which nodes has to be stored in the map
            if(map2D.map_xy.count(*itor) == 0){
                cout<<"wrong\n";
                return false;
            }else{
                iterIntNode it = map2D.map_xy.find(*itor);
                while(it != map2D.map_xy.end()){
                    if((it->second)->lPoints.size() >= 3){                      
                        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                        std::list<pcl::PointXYZ>::iterator node_iter = (it->second)->lPoints.begin();
                        while(node_iter != (it->second)->lPoints.end()){
                             point_cloud_ptr->points.push_back (*node_iter);
                             node_iter++;
                        }
                        //compute mean and covariance_matrix, then drop the pcl
                        Eigen::Matrix3f covariance_matrix; //C
                        Eigen::Vector4f xyz_centroid; //mean
                        pcl::compute3DCentroid(*point_cloud_ptr,xyz_centroid);
                        pcl::computeCovarianceMatrix(*point_cloud_ptr,xyz_centroid,covariance_matrix);
                        (it->second)->xyz_centroid = xyz_centroid;
                        (it->second)->covariance_matrix = covariance_matrix;
//                        cout<<"count out mean and C\n";
                        (it->second)->lPoints.clear();
//                        cout<<"drop the points\n";
                        // if this node's up-down neighbors are free
                        //--- store in the slope  (count roughness and Normal vector, (new slope, cell.push_back)
                        // if not free, ignore them
                        if((it->second)->isSlope(map2D.map_xy)){
                            Slope * slope = new Slope();
                            cell->slope_list.push_back(slope);
                            slope->mean.x = xyz_centroid(0);
                            slope->mean.y = xyz_centroid(1);
                            slope->mean.z = xyz_centroid(2);
                            (it->second)->countRoughNormal(slope->rough,slope->normal);
//                             cout<<"store slope: "<<slope->rough<<","<<slope->normal<<endl;
                        }
                    }
                    it++;
                }
            }
            itor++;
        }
    cout<<"create 2D map done.\n";
    return true;
}

void publish_mean(TwoDmap map2D){
    ros::Rate r(1);
    uint32_t shape = visualization_msgs::Marker::CUBE; //SPHERE ARROW CYLINDER
    int i = 0;
    while (ros::ok()){
        //for every cell
                if(map2D.cell_list.size() != 0){
                list<Cell *>::iterator cell_iter= map2D.cell_list.begin();
                while(cell_iter != map2D.cell_list.end()){
                     Cell * cell = (*cell_iter);
                    //for every slope
                    if(cell->slope_list.size() != 0){
                        list<Slope *>::iterator slope_iter = (*cell_iter)->slope_list.begin();
                        while(slope_iter != (*cell_iter)->slope_list.end()){
                             i++;
                            Vector3f normal = (*slope_iter)->normal;
                            Vec3 mean = (*slope_iter)->mean;
                            //add marker
                            visualization_msgs::Marker marker;
                            marker.ns = "basic_shapes";
                            marker.header.frame_id = "/my_frame";
                            marker.header.stamp = ros::Time::now();
                            marker.id = i; //same namespace and id will overwrite the old one
                            marker.type = shape;
                            marker.action = visualization_msgs::Marker::ADD;
                            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                            marker.pose.position.x = mean.x;
                            marker.pose.position.y = mean.y;
                            marker.pose.position.z = mean.z;
                            marker.pose.orientation.x = normal(0);
                            marker.pose.orientation.y = normal(1);
                            marker.pose.orientation.z = normal(2);
                            marker.pose.orientation.w = 1.0;
                            marker.scale.x = 0.2;
                            marker.scale.y = 0.2;
                            marker.scale.z = 0.2;
                            marker.color.a = 1.0;
                            marker.lifetime = ros::Duration();
//                            marker_array.markers.append(marker);
                            if (marker_pub.getNumSubscribers()){
                                marker_pub.publish(marker);
//                                r.sleep();
                            }
//                            pcl::PointXYZ basic_point;
//                            basic_point.x = normal(0);
//                            basic_point.y = normal(1);
//                            basic_point.z = normal(2);
//                            point_cloud_ptr->points.push_back (basic_point);
                            slope_iter++;
                        }
                    }
                    cell_iter++;
                }

        // Publish the marker

//        while (marker_pub.getNumSubscribers() < 1)
//        {
//          if (!ros::ok())
//          {
//            return 0;
//          }
//          ROS_WARN_ONCE("Please create a subscriber to the marker");
//          sleep(1);
//        }
      }
}
}

///initial insert
int times = 0;
void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr & my_msg)
{    
    if(times ==0){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*my_msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    map2D.cloudFirst.x = temp_cloud->points[0].x;
    map2D.cloudFirst.y = temp_cloud->points[0].y;
    map2D.cloudFirst.z = temp_cloud->points[0].z;
    for (int i=1;i<temp_cloud->points.size();i++)
    {
        uniformDivision(map2D.cloudFirst,temp_cloud->points[i],robot.getR());
     }
    cout<<"division done.\n";
    cout<<"morton size: "<<map2D.morton_list.size()<<endl;
    create2DMap(map2D);
    publish_mean(map2D);
}
    times +=1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fullSys_listener");
  ros::start();
  ros::NodeHandle n; 
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("publisher/cloud_fullSys", 1000, chatterCallback); //initial  

//  ros::Subscriber sub_change = n.subscribe("publisher/cloud_changePart", 1000, chatterCallback); //change part
  ros::spin();

  ros::shutdown();

  return 0;
}

