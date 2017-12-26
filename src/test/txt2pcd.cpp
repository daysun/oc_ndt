#include"omp.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <cstring>
#include <vector>
#include <sstream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
using namespace std;

//turn *.txt into pcd file

float strToFloat(string s){
    float num;
       stringstream ss(s);
       ss >> num;
       return num;
}

void SplitString(const string& s, vector<string>& v, const string& c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}

void readTxt(string file)
{
    pcl::PointCloud<pcl::PointXYZ>cloud;
        cloud.width = 6000;
        cloud.height = 2150;
        cloud.is_dense = false;
    cloud.points.resize(cloud.width*cloud.height);
    ifstream infile;
    infile.open(file.data());
    assert(infile.is_open());

    string s;    
    int p = 0;
    getline(infile,s);//1
    while(getline(infile,s))
    {
        vector<string> v;
        SplitString(s, v," ");
            cloud.points[p].x = strToFloat(v[0]);
            cloud.points[p].y = strToFloat(v[1]);
            cloud.points[p].z = strToFloat(v[2]);
            p++;
    }
    pcl::io::savePCDFileASCII("Site_45_public_favorite.pts.pcd",cloud);
    infile.close();
}

int main(int argc,char *argv[])
{
    readTxt("/home/daysun/rros/src/data/Site_45_public_favorite.pts");
  return 0;
}
