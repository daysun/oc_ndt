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
#include<list>


using namespace daysun;

//std::vector<Vec3> points;
//Octree *octree;
//OctreePoint *octreePoints;
//Vec3 qmin, qmax;
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

 struct OctreeNode
 {
     Vec3 min,max;
     std::list<Vec3> lPoints;
     OctreeNode *children[8];

     OctreeNode //Constructor
     (Vec3 m_min = Vec3(FLT_MAX,FLT_MAX,FLT_MAX),
      Vec3 m_max = Vec3(-FLT_MAX,-FLT_MAX,-FLT_MAX))
         : min(m_min), max(m_max){
         for(int i=0; i<8; ++i)
             children[i] = NULL;
     }

     bool  needDivision(float gridLen){
         if(max.x-min.x > gridLen)
             return true;
         else return false;
     }
 };

 class OctreeNDT {
     float gridLen;
     OctreeNode *root;
     float deep;

     public:
     OctreeNDT():gridLen(-1),root(NULL){ }

     OctreeNDT(OctreeNode *root, const float & gridLen)
         : root(root), gridLen(gridLen),deep(-1){
         createOctree(root);
         }

     void createOctree(OctreeNode *root){
         if(root->needDivision(gridLen)){
             deep+=1/8;
             float mx = (root->max.x-root->min.x)/2 + root->min.x ; //middle x
             float my = (root->max.y-root->min.y)/2 + root->min.y ;
             float mz = (root->max.z-root->min.z)/2 + root->min.z ;
             root->children[0] = new OctreeNode(Vec3(root->min.x,root->min.y,root->min.z),Vec3(mx,my,mz));
             root->children[1] = new OctreeNode(Vec3(mx,root->min.y,root->min.z),Vec3(root->max.x,my,mz));
             root->children[2] = new OctreeNode(Vec3(root->min.x,my,root->min.z),Vec3(mx,root->max.y,mz));
             root->children[3] = new OctreeNode(Vec3(mx,my,root->min.z),Vec3(root->max.x,root->max.y,mz));
             root->children[4] = new OctreeNode(Vec3(root->min.x,root->min.y,mz),Vec3(mx,my,root->max.z));
             root->children[5] = new OctreeNode(Vec3(mx,root->min.y,mz),Vec3(root->max.x,my,root->max.z));
             root->children[6] = new OctreeNode(Vec3(root->min.x,my,mz),Vec3(mx,root->max.y,root->max.z));
             root->children[7] = new OctreeNode(Vec3(mx,my,mz),Vec3(root->max.x,root->max.y,root->max.z));
             while(root->lPoints.size() != 0){
                 Vec3 temp = root->lPoints.front();
                 root->lPoints.pop_front();
                 //decide point belong to which part
                 if(temp.z >root->min.z && temp.z <mz){ //0123
                     if(temp.x >root->min.x && temp.x <mx){ //02
                         if(temp.y >root->min.y && temp.y <my)//0
                             root->children[0]->lPoints.push_back(temp);
                         else root->children[2]->lPoints.push_back(temp);
                     }else{//13
                         if(temp.y >root->min.y && temp.y <my)
                             root->children[1]->lPoints.push_back(temp);
                         else root->children[3]->lPoints.push_back(temp);
                     }
                 }else{ //4567
                     if(temp.x >root->min.x && temp.x <mx){
                         if(temp.y >root->min.y && temp.y <my)
                             root->children[4]->lPoints.push_back(temp);
                         else root->children[6]->lPoints.push_back(temp);
                     }else{
                         if(temp.y >root->min.y && temp.y <my)
                             root->children[5]->lPoints.push_back(temp);
                         else root->children[7]->lPoints.push_back(temp);
                     }
                 }
             }
             for(int i=0;i<8;i++)
                 createOctree(root->children[i]);
             //写好长度关系，根据坐标来划分pointclouds
             //划分为叶子之后，计算每个叶子的需要的信息，然后删除这些点
             //未写完，太慢了没必要
         }else{
             //no need to divide
             //too slow
             //std::cout<<"no need to divide\n"<<"deep:"<<deep<<std::endl;
         }
     }

//     OctreeNDT(const OctreeNDT& copy)
//         : origin(copy.origin), halfDimension(copy.halfDimension), data(copy.data) {
//         }

     //not yet
     ~OctreeNDT() {
         // Recursively destroy octants
     }

     // Determine which octant of the tree would contain 'point'
//     int getOctantContainingPoint(const Vec3& point) const {
//         int oct = 0;
//         if(point.x >= origin.x) oct |= 4;
//         if(point.y >= origin.y) oct |= 2;
//         if(point.z >= origin.z) oct |= 1;
//         return oct;
//     }

//     bool isLeafNode() const {
//         // This is correct, but overkill. See below.
//         /*
//              for(int i=0; i<8; ++i)
//              if(children[i] != NULL)
//              return false;
//              return true;
//          */

//         // We are a leaf iff we have no children. Since we either have none, or
//         // all eight, it is sufficient to just check the first.
//         return children[0] == NULL;
//     }

//     void insert(OctreePoint* point) {
//         // If this node doesn't have a data point yet assigned
//         // and it is a leaf, then we're done!
//         if(isLeafNode()) {
//             if(data==NULL) {
//                 data = point;
//                 return;
//             } else {
//                 // We're at a leaf, but there's already something here
//                 // We will split this node so that it has 8 child octants
//                 // and then insert the old data that was here, along with
//                 // this new data point

//                 // Save this data point that was here for a later re-insert
//                 OctreePoint *oldPoint = data;
//                 data = NULL;

//                 // Split the current node and create new empty trees for each
//                 // child octant.
//                 for(int i=0; i<8; ++i) {
//                     // Compute new bounding box for this child
//                     Vec3 newOrigin = origin;
//                     newOrigin.x += halfDimension.x * (i&4 ? .5f : -.5f);
//                     newOrigin.y += halfDimension.y * (i&2 ? .5f : -.5f);
//                     newOrigin.z += halfDimension.z * (i&1 ? .5f : -.5f);
//                     children[i] = new OctreeNDT(newOrigin, halfDimension*.5f);
//                 }

//                 // Re-insert the old point, and insert this new point
//                 // (We wouldn't need to insert from the root, because we already
//                 // know it's guaranteed to be in this section of the tree)
//                 children[getOctantContainingPoint(oldPoint->getPosition())]->insert(oldPoint);
//                 children[getOctantContainingPoint(point->getPosition())]->insert(point);
//             }
//         } else {
//             // We are at an interior node. Insert recursively into the
//             // appropriate child octant
//             int octant = getOctantContainingPoint(point->getPosition());
//             children[octant]->insert(point);
//         }
//     }

     // This is a really simple routine for querying the tree for points
     // within a bounding box defined by min/max points (bmin, bmax)
     // All results are pushed into 'results'
     void getPointsInsideBox(const Vec3& bmin, const Vec3& bmax, std::vector<OctreePoint*>& results) {

     }

 };

 /*
 void createOctree(OctreeNode * &root, int maxdepth, double xMin, double xMax, double yMin, double yMax, double zMin, double zMax)
 {
         root = new OctreeNode<T>();
         root->data = 9; //为节点赋值，可以存储节点信息，如物体可见性。由于是简单实现八叉树功能，简单赋值为。
         root->xMin = xMin; //为节点坐标赋值
         root->xMax = xMax;
         root->yMin = yMin;
         root->yMax = yMax;
         root->zMin = zMin;
         root->zMax = zMax;
         double xMind = (xMax - xMin) / 2;//计算节点个维度上的半边长
         double yMind = (yMax - yMin) / 2;
         double zMind = (zMax - zMin) / 2;
         //递归创建子树，根据每一个节点所处（是几号节点）的位置决定其子结点的坐标。
         createOctreeNDT(root->top_left_front, maxdepth, xMin, xMax - xMind, yMax - yMind, yMax, zMax - zMind, zMax);
         createOctreeNDT(root->top_left_back, maxdepth, xMin, xMax - xMind, yMin, yMax - yMind, zMax - zMind, zMax);
         createOctreeNDT(root->top_right_front, maxdepth, xMax - xMind, xMax, yMax - yMind, yMax, zMax - zMind, zMax);
         createOctreeNDT(root->top_right_back, maxdepth, xMax - xMind, xMax, yMin, yMax - yMind, zMax - zMind, zMax);
         createOctreeNDT(root->bottom_left_front, maxdepth, xMin, xMax - xMind, yMax - yMind, yMax, zMin, zMax - zMind);
         createOctreeNDT(root->bottom_left_back, maxdepth, xMin, xMax - xMind, yMin, yMax - yMind, zMin, zMax - zMind);
         createOctreeNDT(root->bottom_right_front, maxdepth, xMax - xMind, xMax, yMax - yMind, yMax, zMin, zMax - zMind);
         createOctreeNDT(root->bottom_right_back, maxdepth, xMax - xMind, xMax, yMin, yMax - yMind, zMin, zMax - zMind);
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
}*/

//get data from pcd file
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
  return true;
}

//get current bounding box
 void getBondingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vec3 & min, Vec3 & max, std::list<Vec3> & lPoints){
     for (int i=0;i<cloud->points.size();i++)
     {
         Vec3 temp(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
         lPoints.push_back(temp);
         float maxTemp = temp.maxComponent();
         float minTemp = temp.minComponent();
         if(maxTemp > max.x)
             max.x = max.y = max.z = maxTemp;
         if(minTemp < min.x)
             min.x = min.y = min.z = minTemp;
      }
 }

int main(int argc, char **argv) {
   std::string cloud_path("src/oc_ndt/data/test.pcd");
   float gridLen = 2.0; //related with the robot's radius-changeable
   OctreeNode  * rootNode = new  OctreeNode();
   if (!loadCloud(cloud_path))
     return -1;
   getBondingBox(cloud, rootNode->min,rootNode->max,rootNode->lPoints); //get bound box and initiate rootNode
   OctreeNDT *octree = new  OctreeNDT(rootNode,gridLen);
   //createOctree(rootNode,);

    return 0;
}
