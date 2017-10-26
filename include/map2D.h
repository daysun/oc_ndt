#ifndef MAP2D_H
#define MAP2D_H

#include<iostream>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include<list>
#include"Vec3.h"
#include"Vec2.h"
#include"Stopwatch.h"
#include<float.h>
#include<map>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <visualization_msgs/Marker.h>
using namespace Eigen;
using namespace std;

namespace daysun{
//new one - ABCD UD
struct OcNode
{
//    Vec3 min,max;
    std::list<pcl::PointXYZ> lPoints;
    Eigen::Matrix3f covariance_matrix; //C
    Eigen::Vector3f xyz_centroid; //mean
    int N; //number of points which has been droped(mean having been calculated )
    string morton;
    string z;
    OcNode //Constructor
    (string z ="",string morton = "")
        : z(z){
        covariance_matrix = Eigen::Matrix3f::Zero(3,3);
        xyz_centroid = Eigen::Vector3f::Zero();
        N =0;
    }

    bool isEmpty(){
        if(N == 0 && covariance_matrix == Eigen::Matrix3f::Zero(3,3) && xyz_centroid == Eigen::Vector3f::Zero())
            return true;// emtpty
        else return false; //full
    }

    bool isSlope(multimap<string,OcNode *> map_xy){
        multimap<string,OcNode *>::iterator it = map_xy.find(morton);
        bool up = false, down = false; //false-empty true-full
        while(it != map_xy.end()){
            string currentZ = (it->second)->z;
            int current =strToInt( currentZ.substr(1,currentZ.length()-1));
            int zz = strToInt( z.substr(1,z.length()-1));
            if((current +1 == zz) && ((it->second)->N >=3)){
                down = true;
            }
            if((current -1 == zz) && ((it->second)->N >=3)){
                up = true;
            }
            it++;
        }
        if(up == true && down == true)
            return false;
        else return true;
    }

    void countRoughNormal(float & roughness,Vector3f & normalVector){
        EigenSolver<Matrix3f> es(covariance_matrix);
        Matrix3f eigenvalue  = es.pseudoEigenvalueMatrix(); //value
        Matrix3f eigenvector = es.pseudoEigenvectors();  //vector
        if(eigenvalue(0,0) < eigenvalue(1,1)){
            if(eigenvalue(0,0) < eigenvalue(2,2)){
                roughness = eigenvalue(0,0) ;
                normalVector = eigenvector.col(0);
            }else{
                roughness = eigenvalue(2,2) ;
                normalVector = eigenvector.col(2);
            }
        }else{
            if(eigenvalue(1,1) < eigenvalue(2,2)){
                roughness = eigenvalue(1,1) ;
                normalVector = eigenvector.col(1);
            }else{
                roughness = eigenvalue(2,2) ;
                normalVector = eigenvector.col(2);
            }
        }
    }
};

struct Slope{
    Vector3f normal; //Normal vector of the slope
    float rough;  //roughness of the slope
     Vec3 mean;  //mean value of the slope
   // int index; //need??
};

class Cell{
    string morton;
public:
    list<Slope *> slope_list;
    Cell(const string morton):morton(morton){}
    ~Cell(){
        list<Slope *>::iterator pos,pos2;
        for (pos=slope_list.begin(); pos !=slope_list.end(); ++pos)
        {
                  pos2=pos;
                  delete *pos2;
                  pos++;
                  slope_list.erase(pos2);
        }
    }
    string getMorton(){return morton;}

};

class TwoDmap {
    float gridLen; //resolution
public:
    Vec3 cloudFirst; //initial--the first node
    multimap<string,OcNode *>  map_xy,map_z; //ABCD+morton, UD+height
    TwoDmap(const float res):gridLen(res){}
    float getGridLen(){return gridLen;}
    list<string> morton_list; //xy-morton-all
    list<string> changeMorton_list; //temp-change
    map<string,Cell *> map_cell; //xy_morton, cell

    //inital
    bool create2DMap(){
        cout<<"start create 2D map\n";
        //get all the mortons-new cell, map.push_back
        list<string>::iterator itor = morton_list.begin();
            while(itor!=morton_list.end())
            {
                Cell * cell = new Cell(*itor);
                map_cell.insert(map<string,Cell*>::value_type(cell->getMorton(), cell));
                //for each morton
                //---find the nodes, count the u,c, drop the points inside
                //---determine which nodes has to be stored in the map
                if(map_xy.count(*itor) == 0){
                    cout<<"wrong\n";
                    return false;
                }else{
                    multimap<string,daysun::OcNode *>::iterator  it = map_xy.find(*itor);
                    while(it != map_xy.end()){
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
                            (it->second)->xyz_centroid <<xyz_centroid(0),xyz_centroid(1),xyz_centroid(2);
                            (it->second)->covariance_matrix = covariance_matrix;
                              (it->second)->N += (it->second)->lPoints.size(); //record the num counting mean and C
    //                        cout<<"count out mean and C\n";
                            (it->second)->lPoints.clear();
    //                        cout<<"drop the points\n";
                            // if this node's up-down neighbors are free
                            //--- store in the slope  (count roughness and Normal vector, (new slope, cell.push_back)
                            // if not free, ignore them
                            ////------------------------------------------------for test
                            if(/*(it->second)->isSlope(map_xy)*/ true){
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

    //for visualization-initial
    void showInital(ros::Publisher marker_pub,int color =0){ //0-,1-change
        ros::Rate r(30);
        uint32_t shape = visualization_msgs::Marker::CUBE; //SPHERE ARROW CYLINDER
        int i = 0;
        if (ros::ok()){
            //for every cell
                    if(map_cell.size() != 0){
                    map<string,Cell*>::iterator cell_iter= map_cell.begin();
                    while(cell_iter != map_cell.end()){
                         Cell * cell = cell_iter->second;
                        //for every slope
                        if(cell->slope_list.size() != 0){
                            list<Slope *>::iterator slope_iter = cell->slope_list.begin();
                            while(slope_iter != cell->slope_list.end()){
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
                                marker.scale.x = 0.5;
                                marker.scale.y = 0.5;
                                marker.scale.z = 0.001;
                                marker.color.a = 1.0;
                                marker.color.r = 0.5;
                                if(color == 1){
                                    //for testing change
                                    marker.color.g =0.5;
                                }
                                marker.lifetime = ros::Duration();
                                if (marker_pub.getNumSubscribers() == 1){
                                    marker_pub.publish(marker);
                                    ros::spinOnce();
                                       r.sleep();
//                                    cout<<"publish "<<i<<"\t";
                                }
                                slope_iter++;
                            }
                        }
                        cell_iter++;
                    }
          }
    }
    }

    bool change2DMap(){
        cout<<"start change 2D map\n";
        if(changeMorton_list.size() == 0)
            return false;
        list<string>::iterator itor = changeMorton_list.begin();
        while(itor != changeMorton_list.end()){
            //find if it's contained in cell_morton_list
            if(map_cell.size() == 0)
                return false;
            string changeMorton = *itor;
            map<string,Cell *>::iterator map_it= map_cell.find(changeMorton);
            if(map_it == map_cell.end()) {
                //not find 1
                //create a new cell- the same as the initial
                Cell * cell = new Cell(*itor);
                map_cell.insert(map<string,Cell*>::value_type(cell->getMorton(), cell));
                if(map_xy.count(*itor) == 0){
                    cout<<"wrong\n";
                    return false;
                }else{
                    multimap<string,daysun::OcNode *>::iterator  it = map_xy.find(*itor);
                    while(it != map_xy.end()){
                        if((it->second)->lPoints.size() >= 3){
                            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                            std::list<pcl::PointXYZ>::iterator node_iter = (it->second)->lPoints.begin();
                            while(node_iter != (it->second)->lPoints.end()){
                                 point_cloud_ptr->points.push_back (*node_iter);
                                 node_iter++;
                            }
                            Eigen::Matrix3f covariance_matrix; //C
                            Eigen::Vector4f xyz_centroid; //mean
                            pcl::compute3DCentroid(*point_cloud_ptr,xyz_centroid);
                            pcl::computeCovarianceMatrix(*point_cloud_ptr,xyz_centroid,covariance_matrix);
                            (it->second)->xyz_centroid << xyz_centroid(0),xyz_centroid(1),xyz_centroid(2);
                            (it->second)->covariance_matrix = covariance_matrix;
                              (it->second)->N += (it->second)->lPoints.size();
                            (it->second)->lPoints.clear();
                            ////------------------------------------------------for test
                            if(/*(it->second)->isSlope(map_xy)*/ true){
                                Slope * slope = new Slope();
                                cell->slope_list.push_back(slope);
                                slope->mean.x = xyz_centroid(0);
                                slope->mean.y = xyz_centroid(1);
                                slope->mean.z = xyz_centroid(2);
                                (it->second)->countRoughNormal(slope->rough,slope->normal);
//                                 cout<<"store slope: "<<slope->rough<<","<<slope->normal<<endl;
                            }
                        }
                        it++;
                    }
                }
            }else{
                //find 1-handle the exsit cell-update it
                Cell * cell = map_it->second;
                multimap<string,daysun::OcNode *>::iterator  it = map_xy.find(changeMorton);
                while(it != map_xy.end()){
                    if((it->second)->lPoints.size() >= 3){
                        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                        std::list<pcl::PointXYZ>::iterator node_iter = (it->second)->lPoints.begin();
                        while(node_iter != (it->second)->lPoints.end()){
                             point_cloud_ptr->points.push_back (*node_iter);
                             node_iter++;
                        }
                        if((Eigen::Matrix3f::Zero(3,3) == (it->second)->covariance_matrix )
                                && (Eigen::Vector3f::Zero()==  (it->second)->xyz_centroid)){
                            //not initial
                            Eigen::Matrix3f covariance_matrix; //C
                            Eigen::Vector4f xyz_centroid; //mean
                            pcl::compute3DCentroid(*point_cloud_ptr,xyz_centroid);
                            pcl::computeCovarianceMatrix(*point_cloud_ptr,xyz_centroid,covariance_matrix);
                            (it->second)->xyz_centroid <<xyz_centroid(0),xyz_centroid(1),xyz_centroid(2);
                            (it->second)->covariance_matrix = covariance_matrix;
                        }else{
                            //has initial data--update
                             Eigen::Matrix3f C0 = (it->second)->covariance_matrix;
                             Eigen::Vector3f u0 = (it->second)->xyz_centroid;
                             int N =  (it->second)->N ;
                             int M =  (it->second)->lPoints.size();
                             Eigen::Matrix3f C1,covariance_matrix;
                             Eigen::Vector4f temp_u;
                             Eigen::Vector3f u1,xyz_centroid;
                             pcl::compute3DCentroid(*point_cloud_ptr,temp_u);
                             pcl::computeCovarianceMatrix(*point_cloud_ptr,temp_u,C1);
                             u1<<temp_u(0),temp_u(1),temp_u(2);
                             xyz_centroid =
                                     ( N*u0 + M*u1) / (M+N);
                             covariance_matrix =
                                     ((N-1)*C0 + (M-1)*C1 + M*N/(M+N)*((u0-u1)*((u0-u1).transpose()))) / (M+N-1);
                             (it->second)->xyz_centroid = xyz_centroid;
                             (it->second)->covariance_matrix = covariance_matrix;
//                                      cout<<"has initial data--update\n"<<  (it->second)->xyz_centroid
//                                         <<endl<<(it->second)->covariance_matrix <<endl;
                        }
                        (it->second)->N += (it->second)->lPoints.size();
                        (it->second)->lPoints.clear();
                        ////------------------------------------------------slope for test
                        if(/*(it->second)->isSlope(map_xy)*/ true){
                            Slope * slope = new Slope();
                            cell->slope_list.push_back(slope);
                            slope->mean.x = (it->second)->xyz_centroid(0);
                            slope->mean.y = (it->second)->xyz_centroid(1);
                            slope->mean.z = (it->second)->xyz_centroid(2);
                            (it->second)->countRoughNormal(slope->rough,slope->normal);
//                                 cout<<"store slope: "<<slope->rough<<","<<slope->normal<<endl;
                        }
                    }
                    it++;
                }
            }
/*
            if(cell_morton_list.size() ==0)
                return false;
            list<string>::iterator cIter = cell_morton_list.begin();
            bool found = false;
            while(cIter != cell_morton_list.end()){
                if((*cIter).compare(*itor) == 0){ //find
                    //handle the exsit cell - update it
                    found = true;
                    string morton = *cIter;
                    if(map_xy.count(morton) == 0){
                        cout<<"wrong\n";
                        return false;
                    }else{
                         multimap<string,daysun::OcNode *>::iterator  it = map_xy.find(morton);
                         while(it != map_xy.end()){
                             if((it->second)->lPoints.size() >= 3){
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                                 std::list<pcl::PointXYZ>::iterator node_iter = (it->second)->lPoints.begin();
                                 while(node_iter != (it->second)->lPoints.end()){
                                      point_cloud_ptr->points.push_back (*node_iter);
                                      node_iter++;
                                 }
                                 if((Eigen::Matrix3f::Zero(3,3) == (it->second)->covariance_matrix )
                                         && (Eigen::Vector3f::Zero()==  (it->second)->xyz_centroid)){
                                     //not initial
                                     Eigen::Matrix3f covariance_matrix; //C
                                     Eigen::Vector4f xyz_centroid; //mean
                                     pcl::compute3DCentroid(*point_cloud_ptr,xyz_centroid);
                                     pcl::computeCovarianceMatrix(*point_cloud_ptr,xyz_centroid,covariance_matrix);
                                     (it->second)->xyz_centroid <<xyz_centroid(0),xyz_centroid(1),xyz_centroid(2);
                                     (it->second)->covariance_matrix = covariance_matrix;
                                 }else{
                                     //has initial data--update
                                      Eigen::Matrix3f C0 = (it->second)->covariance_matrix;
                                      Eigen::Vector3f u0 = (it->second)->xyz_centroid;
                                      int N =  (it->second)->N ;
                                      int M =  (it->second)->lPoints.size();
                                      Eigen::Matrix3f C1,covariance_matrix;
                                      Eigen::Vector4f temp_u;
                                      Eigen::Vector3f u1,xyz_centroid;
                                      pcl::compute3DCentroid(*point_cloud_ptr,temp_u);
                                      pcl::computeCovarianceMatrix(*point_cloud_ptr,temp_u,C1);
                                      u1<<temp_u(0),temp_u(1),temp_u(2);
                                      xyz_centroid =
                                              ( N*u0 + M*u1) / (M+N);
                                      covariance_matrix =
                                              ((N-1)*C0 + (M-1)*C1 + M*N/(M+N)*((u0-u1)*((u0-u1).transpose()))) / (M+N-1);
                                      (it->second)->xyz_centroid = xyz_centroid;
                                      (it->second)->covariance_matrix = covariance_matrix;
//                                      cout<<"has initial data--update\n"<<  (it->second)->xyz_centroid
//                                         <<endl<<(it->second)->covariance_matrix <<endl;
                                 }
                                 (it->second)->N += (it->second)->lPoints.size();
                                 (it->second)->lPoints.clear();
                                 ////------------------------------------------------slope for test
                                 if((it->second)->isSlope(map_xy) true){
                                     Slope * slope = new Slope();
                                     cell->slope_list.push_back(slope);
                                     slope->mean.x = xyz_centroid(0);
                                     slope->mean.y = xyz_centroid(1);
                                     slope->mean.z = xyz_centroid(2);
                                     (it->second)->countRoughNormal(slope->rough,slope->normal);
     //                                 cout<<"store slope: "<<slope->rough<<","<<slope->normal<<endl;
                                 }
                             }
                             it++;
                         }
                    }
                    break;
                }
                cIter++;
            }
            if(!found){// not found
                //create a new cell- the same as the initial
                Cell * cell = new Cell(*itor);
//                cell_list.push_back(cell);
                map_cell.insert(map<string,Cell*>::value_type(cell->getMorton(), *cell));
                cell_morton_list.push_back(cell->getMorton());
                if(map_xy.count(*itor) == 0){
                    cout<<"wrong\n";
                    return false;
                }else{
                    multimap<string,daysun::OcNode *>::iterator  it = map_xy.find(*itor);
                    while(it != map_xy.end()){
                        if((it->second)->lPoints.size() >= 3){
                            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
                            std::list<pcl::PointXYZ>::iterator node_iter = (it->second)->lPoints.begin();
                            while(node_iter != (it->second)->lPoints.end()){
                                 point_cloud_ptr->points.push_back (*node_iter);
                                 node_iter++;
                            }
                            Eigen::Matrix3f covariance_matrix; //C
                            Eigen::Vector4f xyz_centroid; //mean
                            pcl::compute3DCentroid(*point_cloud_ptr,xyz_centroid);
                            pcl::computeCovarianceMatrix(*point_cloud_ptr,xyz_centroid,covariance_matrix);
                            (it->second)->xyz_centroid << xyz_centroid(0),xyz_centroid(1),xyz_centroid(2);
                            (it->second)->covariance_matrix = covariance_matrix;
                              (it->second)->N += (it->second)->lPoints.size();
                            (it->second)->lPoints.clear();
                            ////------------------------------------------------for test
                            if((it->second)->isSlope(map_xy) true){
                                Slope * slope = new Slope();
                                cell->slope_list.push_back(slope);
                                slope->mean.x = xyz_centroid(0);
                                slope->mean.y = xyz_centroid(1);
                                slope->mean.z = xyz_centroid(2);
                                (it->second)->countRoughNormal(slope->rough,slope->normal);
//                                 cout<<"store slope: "<<slope->rough<<","<<slope->normal<<endl;
                            }
                        }
                        it++;
                    }
                } // for each cell end
            } //not found end
            */
             itor++;
        } //change morton list end
            cout<<"change 2D map done.\n";
            return true;
    }
};
}
#endif // MAP2D_H
