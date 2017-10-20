//// NormalEstimation：PCL中计算法向量的类。原理是通过对邻域点集求PCA来得到法向量。
////对领域点的三维坐标进协方差矩阵并对行中心化，求得角化，求得三个特征值，
////最小特征值对应的特征向量就是法向量。
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/io/io.h>
//#include <pcl/visualization/cloud_viewer.h>

//int main(int argc, char** argv)
//{

//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
//    // 填入点云数据
//    pcl::io::loadPCDFile("src/oc_ndt/data/test.pcd", *cloud);

//    // 创建滤波器对象
//    pcl::VoxelGrid<pcl::PointXYZI> sor;//滤波处理对象
//    sor.setInputCloud(cloud);
//    sor.setLeafSize(0.01f, 0.01f, 0.01f);//设置滤波器处理时采用的体素大小的参数
//    sor.filter(*cloud_filtered);
//    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
//        << " data points.";

//    pcl::visualization::CloudViewer viewer("Cloud Viewer");
//    //showCloud函数是同步的，在此处等待直到渲染显示为止
//    viewer.showCloud(cloud_filtered);

//    while (!viewer.wasStopped())
//    {
//        //在此处可以添加其他处理

//    }

//    return (0);
//}

// NormalEstimation.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/integral_image_normal.h>


int main()
{
    //加载点云模型
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("src/oc_ndt/data/test.pcd", *cloud)==-1)
    {
        PCL_ERROR("Could not read file\n");
    }

    //计算法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //建立kdtree来进行近邻点集搜索
    //而我是需要对建立好的一团点求它的法向量
    //它是求的一个点的法向量，找了它周围的点
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //为kdtree添加点运数据
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    //点云法向计算时，需要所搜的近邻点大小
    n.setKSearch(20);
    //开始进行法向计算
    n.compute(*normals);

    /*图形显示模块*/
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    //viewer->initCameraParameters();

    //添加点云法向量的另一种方式；
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//    viewer->addPointCloudNormals<pcl::PointNormal>(cloud_with_normals, 50, 0.01, "normals");


    //another
           //将点云数据与法向信息拼接
           pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
           pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

           viewer->setBackgroundColor (0, 0, 0.7);
           pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
           viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
           viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
           //添加需要显示的点云法向。cloud为原始点云模型，normal为法向信息，10表示需要显示法向的点云间隔，即每10个点显示一次法向，５表示法向长度。
           viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 50, 0.5, "normals");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}
