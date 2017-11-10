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
    RobotSphere(const float rr, Vec3 pos= Vec3(0,0,0), Vec3 goal=Vec3(1.3055,0.640172,-1.23853 )):r(rr),position(pos),goal(goal){
        //0.0264822,0.012986,-0.105972
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
        return 100;
    }
    float getAngle(){
        return 45;
    }

    };


#endif // ROBOT_H
