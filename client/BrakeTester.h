#pragma once

#include <array>
#include "MyDriver1.h"
#include "DriverParameters.h"

class BrakeTester : public MyDriver1
{
    public:
    BrakeTester(DriverParameters parameters):MyDriver1(parameters){}
    virtual ~BrakeTester(){}

    protected:
    // Solves the accel changing subproblems
	virtual float getAccel(CarState &cs, float steer);
    
    virtual float getSteer(CarState &cs);

    bool accelPhase = true;
    float startDitance;
};