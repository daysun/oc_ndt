#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
using namespace Eigen;
using namespace std;

int main ()
 {
//加载点云
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile ("src/oc_ndt/data/table_scene_lms400.pcd", *cloud);

Eigen::Matrix3f covariance_matrix;
Eigen::Vector4f xyz_centroid;
pcl::compute3DCentroid(*cloud,xyz_centroid);
pcl::computeCovarianceMatrix(*cloud,xyz_centroid,covariance_matrix); //u,C

EigenSolver<Matrix3f> es(covariance_matrix);
Matrix3f eigenvalue  = es.pseudoEigenvalueMatrix(); //value
Matrix3f eigenvector = es.pseudoEigenvectors();  //vector
cout<<eigenvalue<<endl<<endl;
cout<<eigenvector<<endl<<endl;

//find the smallest value and the corresponding vector
float roughness; //smallest value
Vector3f normalVector; //normal vector small Vector
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

std::cout<<"normal done.\n";

/*
//估计法线
pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
ne.setInputCloud (cloud);
//创建一个空的kdtree对象，并把它传递给法线估计对象
//基于给出的输入数据集，kdtree将被建立
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
ne.setSearchMethod (tree);
//输出数据集
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
//使用半径在查询点周围3厘米范围内的所有邻元素
ne.setRadiusSearch (0.03);
//计算特征值
ne.compute (*cloud_normals);
// cloud_normals->points.size ()应该与input cloud_downsampled->points.size ()有相同尺寸
//法线可视化
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
viewer.setBackgroundColor (0.0, 0.0, 0.0);
viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);

while (!viewer.wasStopped ())
{
     viewer.spinOnce ();
}*/

return 0;
}
