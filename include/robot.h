#ifndef ROBOT_H
#define ROBOT_H
#include "Vec3.h"
#define ROBOT_TAKEUP_CELL 1   //suppose the robot take up 1*1 cells
using namespace std;

class RobotSphere{
    float r; //radius
    Vec3 position;
    Vec3 goal;
    float reachableHeight;//height that it can reach
public:
//    list<Slope *> trajectory;
    RobotSphere(const float rr, Vec3 pos= Vec3(1.02276,0.505226,1.67917/*2.75533,1.36108,1.67499*/),
                Vec3 goal=Vec3(2.02865,1.00212,1.66626)):r(rr),position(pos),goal(goal){
    }
    float getRobotR() {return r;}
    float getR(){return r/ROBOT_TAKEUP_CELL;} //get cell radius
    Vec3 getPosition(){return position;}
    Vec3 getGoal(){return goal;}
    float getReachableHeight(){
        reachableHeight = r/0.5;
        return reachableHeight;}
    float getRough(){
        //should be changed
        return 10000;
    }
    float getAngle(){
        return 60;
    }

    };


#endif // ROBOT_H
