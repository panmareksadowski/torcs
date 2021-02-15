#pragma once

#include "SimpleDriver.h"
#include "DriverParameters.h"

class MyDriver1 : public SimpleDriver
{
    public:
    MyDriver1(DriverParameters parameters):SimpleDriver(parameters){}
    virtual ~MyDriver1(){}
    virtual CarControl wDrive(CarState cs);

    // Initialization of the desired sensorsAngles for the rangefinders
	virtual void init(float *sensorsAngles);

    protected:
    // Solves the accel changing subproblems
	virtual float getAccel(CarState &cs, float steer);
    
    virtual float getSteer(CarState &cs);

    static double speedToBrakingDistance(double speed);
    static double brakingDistanceToSpeed(double distance);

    static constexpr int MAX_TRACK_LENGTH_M = 25000;
    std::array<float, MAX_TRACK_LENGTH_M> steerRecords;
    std::array<int, MAX_TRACK_LENGTH_M> nOfRecords;

    int trackDistance = 0;
    int prevDistance = 0;
    int lap = 0;

    int keepLeftUntil = -1;
    int keepRightUntil = -1;
};