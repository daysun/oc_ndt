#ifndef TWODMAP_H
#define TWODMAP_H

#include<iostream>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include<list>
#include"Vec3.h"
#include"Vec2.h"
#include<float.h>
#include<map>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
using namespace Eigen;
using namespace std;



struct OcNode
{
    Vec3 min,max;
    std::list<Vec3> lPoints;
    Eigen::Matrix3f covariance_matrix; //C
    Eigen::Vector4f xyz_centroid; //mean
    int morton;
    int z;
    OcNode //Constructor
    (Vec3 m_min = Vec3(FLT_MAX,FLT_MAX,FLT_MAX),
     Vec3 m_max = Vec3(-FLT_MAX,-FLT_MAX,-FLT_MAX),int morton = -1,int z =-1)
        : min(m_min), max(m_max),z(z)/*,morton(morton)*/{     }

    bool isSlope(multimap<int,OcNode *> map_xy){
        multimap<int,OcNode *>::iterator it = map_xy.find(morton);
        int status=0;
        while(it != map_xy.end()){
            int currentZ = (it->second)->z;
            if((currentZ +1 == z) && ((it->second)->lPoints.size() >=3)){
                status++;
            }else if((currentZ -1 == z) && ((it->second)->lPoints.size() >=3)){
                status++;
            }
            it++;
        }
        //up-down nodes exist and not empty
        if(status ==2) return false;
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
    int morton;    
public:
    list<Slope *> slope_list;
    Cell(const int morton):morton(morton){}
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
    TwoDmap(const float res):gridLen(res){}
    float getGridLen(){return gridLen;}
    list<int> morton_list;
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

};

#endif // TWODMAP_H
