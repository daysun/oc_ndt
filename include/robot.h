#ifndef ROBOT_H
#define ROBOT_H
#include"map2D.h"
using namespace daysun;
class RobotSphere{
    float r;
public:
    list<Slope *> trajectory;
    RobotSphere(const float rr):r(rr){}
    ~RobotSphere(){
//        list<Slope *>::iterator pos,pos2;
//        for (pos=slope_list.begin(); pos !=slope_list.end(); ++pos)
//        {
//                  pos2=pos;
//                  delete *pos2;
//                  pos++;
//                  slope_list.erase(pos2);
//        }
    }
    float getR(){return r;}

};

#endif // ROBOT_H
