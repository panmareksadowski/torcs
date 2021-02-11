#pragma once

#include "SimpleDriver.h"
#include "DriverParameters.h"
#include <array>

class MyDriver : public SimpleDriver
{
    public:
    MyDriver(DriverParameters parameters):SimpleDriver(parameters){}
    virtual CarControl wDrive(CarState cs);

    protected:
    // Solves the accel changing subproblems
	virtual float getAccel(CarState &cs);

    private:
    static constexpr int MAX_TRACK_LENGTH_M = 10000;
    int lastDistance = 1;
    struct RecordPerMeter{
        float lastSpeed = -1;
        float lastAngle = -1;
        float currentSumSpeed = 0;
        float currentSumAngle = 0;
        float numbersOfRecords = 0;
    };
    int recoveringCnt = 0;
    std::array<RecordPerMeter, MAX_TRACK_LENGTH_M> recordsPerMeter{};
};