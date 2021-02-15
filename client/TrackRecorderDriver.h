#pragma once

#include <array>
#include "SimpleDriver.h"
#include "DriverParameters.h"

class TrackRecorderDriver : public SimpleDriver
{
    public:
    TrackRecorderDriver(DriverParameters parameters):SimpleDriver(parameters){}
    virtual CarControl wDrive(CarState cs);

    // Initialization of the desired sensorsAngles for the rangefinders
	virtual void init(float *sensorsAngles);

    private:
    constexpr static std::array<int, 19> sensorsAngles = {-90,-75,-60,-45,-30,20,15,10,5,0, 5, 10,15,20,30,45,60,75,90};
};