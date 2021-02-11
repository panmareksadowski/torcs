#pragma once

#include <array>
#include "SimpleDriver.h"
#include "DriverParameters.h"

class MyDriver1 : public SimpleDriver
{
    public:
    MyDriver1(DriverParameters parameters):SimpleDriver(parameters){}
    virtual ~MyDriver1(){}
    virtual CarControl wDrive(CarState cs);

    // Initialization of the desired angles for the rangefinders
	virtual void init(float *angles);

    protected:
    // Solves the accel changing subproblems
	virtual float getAccel(CarState &cs, float steer);
    
    virtual float getSteer(CarState &cs);

    static constexpr int MAX_TRACK_LENGTH_M = 25000;
    std::array<float, MAX_TRACK_LENGTH_M> steerRecords;
    std::array<int, MAX_TRACK_LENGTH_M> nOfRecords;

    constexpr static std::array<float, 19> angles = {-50,-40,-32,-24,-16,-8,-4,-2,-1,0, 1,2,4,8, 16,24,32,40,50};
    int trackDistance = 0;
    int prevDistance = 0;
    int lap = 0;
};