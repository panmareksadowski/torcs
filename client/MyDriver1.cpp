#include "MyDriver1.h"
#include "DriverParameters.h"
#include <algorithm>

float
MyDriver1::getAccel(CarState &cs, float steer)
{
    // checks if car is out of track
    if (cs.getTrackPos() <= 1.0 && cs.getTrackPos() >= -1.0)
    {
        steer = std::fabs(steer);
        float steerPenalty = steer - parameters.maxSpeedSteer;
        steerPenalty = std::max(0.0f, steerPenalty);
        int distance = cs.getDistFromStart();
        float maxSpeed = lap > 1 ? parameters.maxSpeed : parameters.firstLapSpeed;
        float targetSpeed = maxSpeed*std::pow((1.0-steerPenalty),parameters.powSteerPenalty);
        targetSpeed = std::max((float)parameters.minSpeed, targetSpeed);
        if(lap > 1)
            for(int i = 1; i < parameters.explorationDistance; ++i){
                int distIndex = (distance+i)%(trackDistance+1);
                if(nOfRecords[distIndex] < 1){
                    continue;
                }
                float steer = std::fabs(steerRecords[distIndex])/nOfRecords[distIndex];
                float penalty =  steer - parameters.maxSpeedSteer;
                penalty = std::max(0.0f, penalty);
                float speed = maxSpeed*std::pow((1.0-penalty),parameters.powSteerPenalty);
                if((i < 25 && parameters.logLevel > 1) ||
                   (i >= 25 && i < 100 && parameters.logLevel > 2) ||
                   parameters.logLevel > 3){
                    std::cerr<<i<<": Steer: "<<std::fabs(steerRecords[distIndex])/nOfRecords[distIndex]<<" Penalty: "<<penalty<<" Speed: "<<speed;
                }
                speed = std::max((float)parameters.minSpeed, speed);
                float startBrakingDistance = speedToBrakingDistance(speed);
                if((i < 25 && parameters.logLevel > 1) ||
                (i >= 25 && i < 100 && parameters.logLevel > 2) ||
                parameters.logLevel > 3){
                    std::cerr<<", startBrakingDistance: "<<startBrakingDistance;
                }
                speed = brakingDistanceToSpeed(startBrakingDistance-i);
                if(i > parameters.brakeDelay)
                    speed -= parameters.losingSpeedPerMeter*parameters.brakeDelay;
                if((i < 25 && parameters.logLevel > 1) ||
                   (i >= 25 && i < 100 && parameters.logLevel > 2) ||
                   parameters.logLevel > 3){
                    std::cerr<<" Speed1 :"<<speed<<"\n";
                }
                if(speed < targetSpeed)
                    targetSpeed = speed;
            }
        
        if(parameters.logLevel > 0){
            std::cout<<"Target speed: "<<targetSpeed<<"\n";
        }
        // accel/brake command is expontially scaled w.r.t. the difference between target speed and current one
        return 2/(1+exp(cs.getSpeedX() - targetSpeed)) - 1;
    }
    else
        return 0.3; // when out of track returns a moderate acceleration command

}

CarControl
MyDriver1::wDrive(CarState cs)
{
    
    std::cout<<"-------\n";
    int distance = cs.getDistFromStart();
    if(distance > trackDistance)
        trackDistance = distance;
    if(distance < prevDistance){
        ++lap;
    }
    prevDistance = distance;
    if(parameters.logLevel > 1){
        std::cout<<"Lap: "<<lap<<"\n";
    }
    if(parameters.logLevel > 0){
        std::cout<<"Distance: "<<cs.getDistFromStart()<<"\n";
        std::cout<<"Speed: "<<cs.getSpeedX()<<"\n";
        std::cout<<"Track pos: "<<cs.getTrackPos()<<"\n";
    }
	// check if car is currently stuck
	if ( fabs(cs.getAngle()) > parameters.stuckAngle )
    {
        //std::cout<<"Tmp stuck"<<"\n";
		// update stuck counter
        stuck++;
    }
    else
    {
    	// if not stuck reset stuck counter
        stuck = 0;
    }

	// after car is stuck for a while apply recovering policy
    if (stuck > parameters.stuckTime)
    {
        
        //std::cout<<"Totally stuck"<<"\n";

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

        if (steer < -1)
            steer = -1;
        if (steer > 1)
            steer = 1;


        //std::cout<<"Gear: "<<gear<<"\n";
        //std::cout<<"Steer: "<<steer<<std::endl;

        // Calculate clutching
        clutching(cs,clutch);

        // build a CarControl variable and return it
        CarControl cc (1.0,0.0,gear,steer,clutch);
        return cc;
    }

    else // car is not stuck
    {
        //std::cout<<"All good"<<"\n";

        // compute steering
        float steer = getSteer(cs);
    	// compute accel/brake command
        float accel_and_brake = getAccel(cs, steer);
        // compute gear 
        int gear = getGear(cs);
        
        if(parameters.logLevel > 0){
            std::cout<<"Accel: "<<accel_and_brake<<"\n";
            std::cout<<"Steer: "<<steer<<std::endl;
        }
        
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

float
MyDriver1::getSteer(CarState &cs)
{

    if (cs.getTrackPos() > 1.0 || cs.getTrackPos() < -1.0){
        float targetAngle = cs.getAngle()-cs.getTrackPos()*parameters.returnTrackFactor;
        //std::cout<<"Angle: "<<targetAngle*180.0/M_PI<<"\n";
        float steer = targetAngle/parameters.steerLock;
        steer = std::max(std::min(1.0f, steer),-1.0f);
        return steer;
    }

    float maxDistance = cs.getTrack(0);
    float maxAngle = parameters.sensorsAngles[0];
    for(uint i = 1; i < parameters.sensorsAngles.size(); ++i){
        float distance = cs.getTrack(i);
        //std::cerr<<parameters.sensorsAngles[i]<<":"<<distance<<"\n";
        if(distance > maxDistance){
            maxDistance = distance;
            maxAngle = parameters.sensorsAngles[i];
        }
    }
    

    int distance = cs.getDistFromStart();
    /*if(distance % (trackDistance+1) < keepRightUntil){
        if(parameters.logLevel > 0){
            std::cout<<"Left curve at the end. Keep right until "<<keepRightUntil<<"\n";
        }
        float targetAngle = cs.getAngle()+(-1.0-cs.getTrackPos())*0.05;
        float steer = targetAngle/parameters.steerLock;
        steer = std::max(std::min(0.045f, steer),-0.045f);
        return steer;
    }
    if(distance % (trackDistance+1)  < keepLeftUntil){
        if(parameters.logLevel > 0){
            std::cout<<"Right curve at the end. Keep left "<<keepLeftUntil<<"\n";
        }
        float targetAngle = cs.getAngle()+(1.0-cs.getTrackPos())*0.05;
        float steer = targetAngle/parameters.steerLock;
        steer = std::max(std::min(0.045f, steer),-0.045f);
        return steer;
    }*/

    int minDistance = std::max((int)cs.getSpeedX() - 140, parameters.minDistanceToSharpCurve);
    if(lap > 1 && maxAngle < 6 && std::any_of(parameters.lowAnglesSensorsIndexes.begin(), parameters.lowAnglesSensorsIndexes.end(), 
                    [minDistance, &cs](int angleIndex){return cs.getTrack(angleIndex) >  minDistance;})){
        //int breakIndex = 600;
        for(int i = minDistance+20; i < 600; ++i){
            //if(i > breakIndex)
            //   break;
            int distIndex = (distance+i)%(trackDistance+1);
            if(steerRecords[distIndex]/nOfRecords[distIndex] > parameters.sharpCurveSteerTreshold){
                if(parameters.logLevel > 0){
                    std::cout<<"Left curve at the end\n";
                }
                //keepRightUntil = distIndex - parameters.minDistanceToSharpCurve-15;
                float targetAngle = cs.getAngle()+(-1.0-cs.getTrackPos())*parameters.adjustTrackBeforeSharpCurveFactor;
                float steer = targetAngle/parameters.steerLock;
                steer = std::max(std::min(parameters.maxSteerBeforeSharpCurveFactor, steer),-parameters.maxSteerBeforeSharpCurveFactor);
                return steer;
            }else if(steerRecords[distIndex]/nOfRecords[distIndex] < -parameters.sharpCurveSteerTreshold){
                if(parameters.logLevel > 0){
                    std::cout<<"Right curve at the end\n";
                }
                //keepLeftUntil = distIndex - parameters.minDistanceToSharpCurve-15;
                float targetAngle = cs.getAngle()+(1.0-cs.getTrackPos())*parameters.adjustTrackBeforeSharpCurveFactor;
                float steer = targetAngle/parameters.steerLock;
                steer = std::max(std::min(parameters.maxSteerBeforeSharpCurveFactor, steer),-parameters.maxSteerBeforeSharpCurveFactor);
                return steer;
            }/*else if(std::fabs(steerRecords[distIndex])/nOfRecords[distIndex] > 0.15){
                breakIndex = i + parameters.minDistanceToSharpCurve+30;
           }*/
        }
    }
    keepLeftUntil = -1;
    keepRightUntil = -1;
    float trackPos = cs.getTrackPos();
    if(parameters.logLevel > 0){
        std::cout<<"Angle1: "<<maxAngle<<"\n";
        std::cout<<"trackPos1: "<<trackPos<<"\n";
    }
    steerRecords[distance] += -(maxAngle*M_PI/180.0)/parameters.steerLock;
    nOfRecords[distance]++;
    if(maxAngle > 0){
        trackPos += 1;
        trackPos/=2.0;
    }else{
        trackPos -= 1;
        trackPos/=-2.0;
    }
    maxAngle *= (trackPos+parameters.desiredToSteerAngleFactor);
    if(parameters.logLevel > 0){
        std::cout<<"trackPos2: "<<trackPos<<"\n";
        std::cout<<"Angle2: "<<maxAngle<<"\n";
    }
    if(parameters.logLevel > 1){
        std::cout<<"Angle2: "<<maxAngle<<"\n";
        std::cout<<"Angle: "<<-(maxAngle*M_PI/180.0)<<"\n";
    }
    float steer = -(maxAngle*M_PI/180.0)/parameters.steerLock;
    steer = std::max(std::min(1.0f, steer),-1.0f);
    return steer;
}

void
MyDriver1::init(float *sensorsAngles)
{

    for(uint i = 0; i < parameters.sensorsAngles.size(); ++i)
        sensorsAngles[i] = parameters.sensorsAngles[i];
}

double MyDriver1::speedToBrakingDistance(double x){
    double x9 = std::pow(x,9);
    double x8 = std::pow(x,8);
    double x7 = std::pow(x,7);
    double x6 = std::pow(x,6);
    double x5 = std::pow(x,5);
    double x4 = std::pow(x,4);
    double x3 = std::pow(x,3);
    double x2 = std::pow(x,2);
    return -2.2459e-17*x9 + 3.24638e-14*x8 - 2.02889e-11*x7 + 7.18002e-9*x6 - 1.5817e-6*x5 + 0.000224312 * x4 - 0.020418*x3 + 1.1449 *x2 - 35.9635*x + 632.37;
}

double MyDriver1::brakingDistanceToSpeed(double x){
    if(x < 58)
        return 290.733 - 1.12171*x;
    double x9 = std::pow(x,9);
    double x8 = std::pow(x,8);
    double x7 = std::pow(x,7);
    double x6 = std::pow(x,6);
    double x5 = std::pow(x,5);
    double x4 = std::pow(x,4);
    double x3 = std::pow(x,3);
    double x2 = std::pow(x,2);
    return -2.45679e-14*x9 + 1.99336e-11*x8 - 6.93801e-9*x7 + 1.34994e-6*x6 - 0.000160213*x5 + 0.0118504*x4 - 0.533242*x3 + 13.4521*x2 - 155.929*x + 669.339;
}