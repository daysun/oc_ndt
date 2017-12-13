#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

//pcl_view test
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud添加点云到视窗实例代码-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setRepresentationToSurfaceForAllActors();
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  /************************************************************************************************
  绘制形状的实例代码，绘制点之间的连线，
*************************************************************************************************/
//  viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],                                     cloud->points[cloud->size() - 1], "line");
  //添加点云中第一个点为中心，半径为0.2的球体，同时可以自定义颜色
//  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations添加绘制平面使用标准平面方程ax+by+cz+d=0来定义平面，这个平面以原点为中心，方向沿着Z方向-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (-3.0);
  viewer->addPlane (coeffs, "plane");

    coeffs.values.clear ();
    coeffs.values.push_back (1);
    coeffs.values.push_back (1);
    coeffs.values.push_back (1);

  viewer->addCircle(coeffs,"circles");
  //添加锥形的参数
  coeffs.values.clear ();
  coeffs.values.push_back (1);
  coeffs.values.push_back (1);
  coeffs.values.push_back (1);
  coeffs.values.push_back (1);
  coeffs.values.push_back (1);
  coeffs.values.push_back (1);
  coeffs.values.push_back (1);
  viewer->addCylinder (coeffs, "cylinder");


  return (viewer);
}

int showSlope (int argc, char** argv)
{

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = shapesVis();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
