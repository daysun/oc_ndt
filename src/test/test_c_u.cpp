#include<iostream>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include<list>
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
#include <sys/time.h>
#include <cmath>
using namespace Eigen;
using namespace std;

double stopwatch()
{
    struct timeval time;
    gettimeofday(&time, 0 );
    return 1.0 * time.tv_sec + time.tv_usec / (double)1e6;
}

void netCount( Matrix3f K){
    cout<<"net \n";
    double start = stopwatch();
    for(int i =0;i<100;i++){
EigenSolver<Matrix3f> es(K);
MatrixXcf evecs = es.eigenvectors();//获取矩阵特征向量4*4，
          //这里定义的MatrixXcd必须有c，表示获得的是complex复数矩阵
MatrixXcf evals = es.eigenvalues();//获取矩阵特征值 4*1
MatrixXf evalsReal;//注意这里定义的MatrixXd里没有c
evalsReal=evals.real();//获取特征值实数部分
MatrixXf::Index evalsMax;
evalsReal.rowwise().sum().minCoeff(&evalsMax);//得到最大特征值的位置
Vector3f q;
//得到对应特征向量
q << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
evecs.real()(2, evalsMax);
    }
cout<<stopwatch()-start<<endl;
//cout<<"net :"<<q<<endl;
}

void countRoughNormal(Matrix3f covariance_matrix ){
    cout<<"mine \n";
    double start = stopwatch();
    for(int i=0;i<100;i++){
    EigenSolver<Matrix3f> es(covariance_matrix);
    Matrix3f eigenvalue  = es.pseudoEigenvalueMatrix(); //value
    Matrix3f eigenvector = es.pseudoEigenvectors();  //vector
    float roughness;
    Vector3f normalVector;
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
    cout<<stopwatch()-start<<endl;
//    cout<<"mine: "<<roughness<<endl<<normalVector<<endl;
}

int main(){

    cout<< ceil(float(0.1/1))<<endl;
//    Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero(3,3); //c
//    Matrix3f K;
//    Eigen::Vector4f xyz_centroid = Eigen::Vector4f::Zero(); //mean
//    Eigen::Vector4f com ;
//    com<< 0,0,0,0;

//    K<<0,0,0,0,0,0,0,0,0;
//    if(K == covariance_matrix){
//        cout<<"matrix the same,right\n";
//    }
//    K<< 5, 4 ,2 ,0 ,-3, 4, 0, 4 ,3;
//    if(K == covariance_matrix){
//        cout<<"matrix wrong\n";
//    }

//    if(xyz_centroid == com)
//        cout<<"vector right\n";
//    com<<1,2,4,6;
//    if(xyz_centroid == com)
//        cout<<"vector wrong\n";


}
