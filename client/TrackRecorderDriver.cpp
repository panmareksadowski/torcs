#include "TrackRecorderDriver.h"
#include "DriverParameters.h"

constexpr std::array<int, 19> TrackRecorderDriver::angles;

CarControl TrackRecorderDriver::wDrive(CarState cs){
    std::cout<<"-------"<<std::endl;
    std::cout<<"Dist: " << cs.getDistFromStart()<<std::endl;
    std::cout<<"Dist1: " <<cs.getDistRaced()<<std::endl;
    std::cout<<"Angle: " <<cs.getAngle()<<std::endl;
    std::cout<<"Trak: " <<cs.getTrackPos()<<std::endl;
    for(int i = 0; i < TrackRecorderDriver::angles.size()/2; ++i){
        std::cout<<"Trak " << TrackRecorderDriver::angles[i] << ": " << cs.getTrack(i);
        int i1 = TrackRecorderDriver::angles.size()-1-i;
        std::cout<<"Trak " << TrackRecorderDriver::angles[i1] << ": " << cs.getTrack(i1)<<std::endl;
    }
    int midIndex = TrackRecorderDriver::angles.size()/2+1;
    std::cout<<"Trak " << TrackRecorderDriver::angles[midIndex] << ": " << cs.getTrack(midIndex);
    return SimpleDriver::wDrive(cs);
}

void
TrackRecorderDriver::init(float *angles)
{
    for(int i = 0; i < TrackRecorderDriver::angles.size(); ++i)
        angles[i] = TrackRecorderDriver::angles[i];
}