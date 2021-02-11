#include "MyDriver.h"
#include "DriverParameters.h"

CarControl MyDriver::wDrive(CarState cs){
    static int cnt = 0;
    cnt++;
    int distance = cs.getDistFromStart();
    if(distance > lastDistance)
        lastDistance = distance;
    if(cnt > parameters.startRecording){
        int prevDistance = distance > 0 ? distance-1:lastDistance;
        int prevNumbersOfRecords = recordsPerMeter[prevDistance].numbersOfRecords;
        if(prevNumbersOfRecords > 0){
            recordsPerMeter[prevDistance].lastAngle = recordsPerMeter[prevDistance].currentSumAngle/prevNumbersOfRecords;
            recordsPerMeter[prevDistance].lastSpeed = recordsPerMeter[prevDistance].currentSumSpeed/prevNumbersOfRecords;
            recordsPerMeter[prevDistance].currentSumAngle = 0;
            recordsPerMeter[prevDistance].currentSumSpeed = 0;
            recordsPerMeter[prevDistance].numbersOfRecords = 0;
        }else{
            recordsPerMeter[prevDistance].lastAngle = -1;
            recordsPerMeter[prevDistance].lastSpeed = -1;
            recordsPerMeter[prevDistance].currentSumAngle = 0;
            recordsPerMeter[prevDistance].currentSumSpeed = 0;
            recordsPerMeter[prevDistance].numbersOfRecords = 0;
        }
        if ( stuck < parameters.stuckTime && recoveringCnt == 0 ){
            recordsPerMeter[distance].currentSumAngle += cs.getAngle();
            recordsPerMeter[distance].currentSumSpeed += cs.getSpeedX();
            recordsPerMeter[distance].numbersOfRecords++;
        }
    }
    std::cout<<"-------"<<std::endl;
    std::cout<<cs.getDistFromStart()<<std::endl;
    std::cout<<cs.getAngle()<<std::endl;
    	// check if car is currently stuck
	if ( fabs(cs.getAngle()) > parameters.stuckAngle )
    {
		// update stuck counter
        stuck++;
        std::cout<<"Stuck tick"<<std::endl;
    }
    else
    {
    	// if not stuck reset stuck counter
        stuck = 0;
    }

    if(recoveringCnt > 0)
        recoveringCnt--;

	// after car is stuck for a while apply recovering policy
    if (stuck > parameters.stuckTime)
    {
        std::cout<<"Totally stuck"<<std::endl;
        recoveringCnt = parameters.stuckRecoverTime;
    	/* set gear and sterring command assuming car is 
    	 * pointing in a direction out of track */
    	
    	// to bring car parallel to track axis
        float steer = - cs.getAngle() / parameters.steerLock; 
        int gear=-1; // gear R
        
        // if car is pointing in the correct direction revert gear and steer  
        if (cs.getAngle()*cs.getTrackPos()>0)
        {
            gear = 1;
            steer = -steer;
        }

        // Calculate clutching
        clutching(cs,clutch);

        // build a CarControl variable and return it
        CarControl cc (1.0,0.0,gear,steer,clutch);
        return cc;
    }
    else // car is not stuck
    {
    	// compute accel/brake command
        float accel_and_brake = getAccel(cs);

        // compute gear 
        int gear = getGear(cs);
        // compute steering
        float steer = getSteer(cs);
        

        // normalize steering
        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;
        
        // set accel and brake from the joint accel/brake command 
        float accel,brake;
        if (accel_and_brake>0)
        {
            accel = accel_and_brake;
            brake = 0;
        }
        else
        {
            accel = 0;
            // apply ABS to brake
            brake = filterABS(cs,-accel_and_brake);
        }

        // Calculate clutching
        clutching(cs,clutch);

        // build a CarControl variable and return it
        CarControl cc(accel,brake,gear,steer,clutch);
        return cc;
    }
}

float MyDriver::getAccel(CarState &cs){
    if(recoveringCnt > 0){
        std::cout<<"Recovering after stuck policy\n";
        return SimpleDriver::getAccel(cs);
    }
    if (cs.getTrackPos() >= 1 || cs.getTrackPos() <= -1){
         std::cout<<"Out fo track\n";
        return 0.3;
    }
    int distance = cs.getDistFromStart();
    if(recordsPerMeter[distance].lastSpeed == -1){
        std::cout<<"First Lap\n";
        return SimpleDriver::getAccel(cs);
    }
    else{
        float sumAngle = 0;
        float sumSpeed = 0;
        int cnt = 0;
        for(int i = parameters.sforwardDistance; i <= parameters.eforwardDistance; ++i){
            int index = (distance+i)%(lastDistance+1);
            if(recordsPerMeter[index].lastSpeed != 1){  
                sumAngle += fabs(recordsPerMeter[index].lastAngle);
                sumSpeed  += recordsPerMeter[index].lastSpeed;
                cnt++;
            }
        }
        if(cnt == 0){
            std::cout<<"No history data\n";
            return SimpleDriver::getAccel(cs);
        }
        float avgAngle = sumAngle/cnt;;
        if(avgAngle > parameters.stuckAngle){
            std::cout<<"Avg stucked\n";
            return SimpleDriver::getAccel(cs);
        }
        float avgSpeed = sumSpeed/cnt;
        float newSpeed = avgSpeed + (parameters.tresholdAngle-avgAngle)*parameters.angleToSpeedCoef;
        if(newSpeed < parameters.minSpeed){
            std::cout<<"Speed bewlow min speed\n";
            return 0.3;
        }
        std::cout<<"Avg speed:"<<avgSpeed<<" avg angle:"<<avgAngle<<" new speed:"<<newSpeed<<std::endl;
        return 2/(1+exp(cs.getSpeedX() - newSpeed)) - 1;
    }
}