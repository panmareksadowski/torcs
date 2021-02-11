#include "BrakeTester.h"
#include "DriverParameters.h"
#include <algorithm>

float
BrakeTester::getAccel(CarState &cs, float steer)
{
    if(accelPhase && cs.getSpeedX() > 150){
        accelPhase = false;
        startDitance = cs.getDistRaced();
        std::cout<<"Speed reached. Start breaking."<<std::endl;
    }
    if(accelPhase)
        return MyDriver1::getAccel(cs, steer);
    else{
        std::cout<<cs.getDistRaced()-startDitance<<" : "<<150-cs.getSpeedX()<<std::endl;
        return -1;
    }

}


float
BrakeTester::getSteer(CarState &cs)
{
    if(accelPhase)
        return MyDriver1::getSteer(cs);
    else{
        return 0;
    }
}