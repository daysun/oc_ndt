#include <cstdlib>
#include <iostream>
#include <vector>
#include <algorithm>

#include "Octree.h"
#include "Stopwatch.h"
#include <pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/filter.h>

using namespace daysun;

//methods for octree from github
//not suitable for this one

//octree.h,octreePoint.h,Stopwatch.h,vec3.h
//hellogithub.cpp
// Used for testing
std::vector<Vec3> points;
Octree *octree;
OctreePoint *octreePoints;
Vec3 qmin, qmax;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

float rand11() // Random number between [-1,1]
{ return -1.f + (2.f*rand()) * (1.f / RAND_MAX); }

Vec3 randVec3() // Random vector with components in the range [-1,1]
{ return Vec3(rand11(), rand11(), rand11()); }

// Determine if 'point' is within the bounding box [bmin, bmax]
bool naivePointInBox(const Vec3& point, const Vec3& bmin, const Vec3& bmax) {
    return
        point.x >= bmin.x &&
        point.y >= bmin.y &&
        point.z >= bmin.z &&
        point.x <= bmax.x &&
        point.y <= bmax.y &&
        point.z <= bmax.z;
}

void init() {
    // Create a new Octree centered at the origin
    // with physical dimension 2x2x2
    octree = new Octree(Vec3(0,0,0), Vec3(1,1,1));

    // Create a bunch of random points
    const int nPoints = 1 * 1000 * 1000;
    for(int i=0; i<nPoints; ++i) {
        points.push_back(randVec3());
    }
       std::cout<<"Created "<<points.size()<<" points\n ";
   // printf("Created %ld points\n", points.size());       fflush(stdout);

    // Insert the points into the octree
    octreePoints = new OctreePoint[nPoints];
    for(int i=0; i<nPoints; ++i) {
        octreePoints[i].setPosition(points[i]);
        octree->insert(octreePoints + i);
    }
       std::cout<<"Inserted points to octree\n ";
   // printf("Inserted points to octree\n");       fflush(stdout);

    // Create a very small query box. The smaller this box is
    // the less work the octree will need to do. This may seem
    // like it is exagerating the benefits, but often, we only
    // need to know very nearby objects.
    qmin = Vec3(-.05,-.05,-.05);
    qmax = Vec3(.05,.05,.05);

    // Remember: In the case where the query is relatively close
    // to the size of the whole octree space, the octree will
    // actually be a good bit slower than brute forcing every point!
}

// Query using brute-force
void testNaive() {
    double start = stopwatch();

    std::vector<int> results;
    for(int i=0; i<points.size(); ++i) {
        if(naivePointInBox(points[i], qmin, qmax)) {
            results.push_back(i);
        }
    }

    double T = stopwatch() - start;
     std::cout<<"testNaive found "<<results.size()<<" points in "<<T<<" sec.\n";
    //printf("testNaive found %ld points in %.5f sec.\n", results.size(), T);
}

// Query using Octree
void testOctree() {
    double start = stopwatch();

    std::vector<OctreePoint*> results;
    octree->getPointsInsideBox(qmin, qmax, results);

    double T = stopwatch() - start;
    std::cout<<"testOctree found "<<results.size()<<" points in "<<T<<" sec.\n";
    //printf("testOctree found %ld points in %.5f sec.\n", results.size(), T);
}

bool loadCloud(std::string &filename)
{
  std::cout << "Loading file " << filename.c_str() << std::endl;
  //read cloud
  if (pcl::io::loadPCDFile("src/oc_ndt/data/test.pcd", *cloud))
  {
    std::cout << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
    return false;
  }

  //remove NaN Points
  std::vector<int> nanIndexes;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndexes);
  std::cout << "Loaded " << cloud->points.size() << " points" << std::endl;

  ///get data-insert them into the octree

//  //create octree structure
//  octree.setInputCloud(cloud);
//  //update bounding box automatically
//  octree.defineBoundingBox();
//  //add points in the tree
//  octree.addPointsFromInputCloud();
  return true;
}

int main(int argc, char **argv) {
    ////error---- Assertion `px != 0' failed.
   std::string cloud_path("src/oc_ndt/data/test.pcd");
   if (!loadCloud(cloud_path))
     return -1;
   init();
   testNaive();
   testOctree();
    //std::cout<<"ok..";

    return 0;
}
