#include "TrackRecorderDriver.h"
#include "DriverParameters.h"

constexpr std::array<int, 19> TrackRecorderDriver::sensorsAngles;

CarControl TrackRecorderDriver::wDrive(CarState cs){
    std::cout<<"-------"<<std::endl;
    std::cout<<"Dist: " << cs.getDistFromStart()<<std::endl;
    std::cout<<"Dist1: " <<cs.getDistRaced()<<std::endl;
    std::cout<<"Angle: " <<cs.getAngle()<<std::endl;
    std::cout<<"Trak: " <<cs.getTrackPos()<<std::endl;
    for(uint i = 0; i < TrackRecorderDriver::sensorsAngles.size()/2; ++i){
        std::cout<<"Trak " << TrackRecorderDriver::sensorsAngles[i] << ": " << cs.getTrack(i);
        int i1 = TrackRecorderDriver::sensorsAngles.size()-1-i;
        std::cout<<"Trak " << TrackRecorderDriver::sensorsAngles[i1] << ": " << cs.getTrack(i1)<<std::endl;
    }
    int midIndex = TrackRecorderDriver::sensorsAngles.size()/2+1;
    std::cout<<"Trak " << TrackRecorderDriver::sensorsAngles[midIndex] << ": " << cs.getTrack(midIndex);
    return SimpleDriver::wDrive(cs);
}

void
TrackRecorderDriver::init(float *sensorsAngles)
{
    for(uint i = 0; i < TrackRecorderDriver::sensorsAngles.size(); ++i)
        sensorsAngles[i] = TrackRecorderDriver::sensorsAngles[i];
}