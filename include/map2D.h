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
using namespace Eigen;
using namespace std;

namespace daysun{
//new one - ABCD UD
struct OcNode
{
//    Vec3 min,max;
    std::list<pcl::PointXYZ> lPoints;
    Eigen::Matrix3f covariance_matrix; //C
    Eigen::Vector4f xyz_centroid; //mean
    string morton;
    string z;
    OcNode //Constructor
    (string z ="",string morton = "")
        : z(z){     }

    bool isSlope(multimap<string,OcNode *> map_xy){
        multimap<string,OcNode *>::iterator it = map_xy.find(morton);
        bool up = false, down = false; //false-empty true-full
        while(it != map_xy.end()){
            string currentZ = (it->second)->z;
            int current =strToInt( currentZ.substr(1,currentZ.length()-1));
            int zz = strToInt( z.substr(1,z.length()-1));
            if((current +1 == zz) && ((it->second)->lPoints.size() >=3)){
                down = true;
            }
            if((current -1 == zz) && ((it->second)->lPoints.size() >=3)){
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

};

class TwoDmap {
    float gridLen; //resolution
public:
    Vec3 cloudFirst; //initial--the first node
    multimap<string,OcNode *>  map_xy,map_z; //ABCD+morton, UD+height
    TwoDmap(const float res):gridLen(res){}
    float getGridLen(){return gridLen;}
    list<string> morton_list; //xy
    list<Cell *> cell_list;
    ~TwoDmap() {
        list<Cell *>::iterator pos,pos2;
        for (pos=cell_list.begin(); pos !=cell_list.end(); ++pos)
        {
                  pos2=pos;
                  delete *pos2;
                  pos++;
                  cell_list.erase(pos2);
        }
    }

    //for visualization
//    void showSlope(){
//        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//        viewer->setBackgroundColor (0, 0, 0);
//        viewer->setRepresentationToSurfaceForAllActors();
//        viewer->addCoordinateSystem (1.0);
//        viewer->initCameraParameters ();
//        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//        //for every cell
//        if(cell_list.size() != 0){
//            int i = 0;
//        list<Cell *>::iterator cell_iter= cell_list.begin();
//        while(cell_iter != cell_list.end()){
//             Cell * cell = (*cell_iter);
//            //for every slope
//            if(cell->slope_list.size() != 0){
//                list<Slope *>::iterator slope_iter = (*cell_iter)->slope_list.begin();
//                while(slope_iter != (*cell_iter)->slope_list.end()){
//                     i++;
//                    Vector3f normal = (*slope_iter)->normal;
//                    Vec3 mean = (*slope_iter)->mean;
//                    //add points
//                    pcl::PointXYZ basic_point;
//                    basic_point.x = normal(0);
//                    basic_point.y = normal(1);
//                    basic_point.z = normal(2);
//                    point_cloud_ptr->points.push_back (basic_point);
////                     viewer->addSphere (basic_point, 0.2, 0.2, 0.2, 0.0,"sphere"+i);
//                    //add plane --some problems
////                    float a= normal(0),b = normal(1), c= normal(2);
////                    float d = -a*mean.x -b*mean.y - c*mean.z;
////                    cout<<a<<","<<b<<","<<c<<","<<d<<endl;
////                    pcl::ModelCoefficients coeffs;
////                    coeffs.values.push_back (a);
////                    coeffs.values.push_back (b);
////                    coeffs.values.push_back (c);
////                    coeffs.values.push_back (d);
////                    viewer->addPlane (coeffs, "plane"+i);
//                    slope_iter++;
//                }
//            }
//            cell_iter++;
//        }
//        }

//        viewer->addPointCloud<pcl::PointXYZ> (point_cloud_ptr, "sample cloud");
//        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//        viewer->setRepresentationToSurfaceForAllActors();
//        viewer->addCoordinateSystem (1.0);
//        viewer->initCameraParameters ();

//        while (!viewer->wasStopped ())
//        {
//          viewer->spinOnce (100);
//          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//        }
//    }

};
}
#endif // MAP2D_H
