#include "MyDriver1.h"
#include "DriverParameters.h"
#include <algorithm>

constexpr std::array<float, 19> MyDriver1::angles;

float
MyDriver1::getAccel(CarState &cs, float steer)
{
    // checks if car is out of track
    if (cs.getTrackPos() <= 1.0 && cs.getTrackPos() >= -1.0)
    {
        steer = std::fabs(steer);
        float steerPenalty = steer - parameters.safeSteer;
        steerPenalty = std::max(0.0f, steerPenalty);
        int distance = cs.getDistFromStart();
        float maxSpeed = lap > 1 ? parameters.maxSpeed : 100.0;
        float targetSpeed = maxSpeed*std::pow((1.0-steerPenalty),parameters.powSteerPenalty);
        targetSpeed = std::max((float)parameters.minSpeed, targetSpeed);
        if(lap > 1)
            for(int i = 1; i < parameters.followingSteerArea; ++i){
                int distIndex = (distance+i)%(trackDistance+1);
                if(nOfRecords[distIndex] < 1){
                    continue;
                }
                float penalty =  steerRecords[distIndex]/nOfRecords[distIndex] - parameters.safeSteer;
                penalty = std::max(0.0f, penalty);
                float speed = maxSpeed*std::pow((1.0-penalty),parameters.powSteerPenalty);
                if(i < 100){
                    //std::cerr<<i<<": Steer: "<<steerRecords[distIndex]/nOfRecords[distIndex]<<" Penalty: "<<penalty<<" Speed: "<<speed;
                }
                speed = std::max((float)parameters.minSpeed, speed);
                if(i > 10)
                    speed += parameters.breakCoeficient*(i-10);
                if(i < 100){
                    //std::cerr<<" Speed1 :"<<speed<<"\n";
                }
                if(speed < targetSpeed)
                    targetSpeed = speed;
            }
        
        //std::cout<<"Target speed: "<<targetSpeed<<"\n";
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
    //std::cout<<"Distance: "<<cs.getDistFromStart()<<"\n";
    //std::cout<<"Lap: "<<lap<<"\n";
    std::cout<<"Speed: "<<cs.getSpeedX()<<"\n";
    std::cout<<"Track pos: "<<cs.getTrackPos()<<"\n";
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
        

        std::cout<<"Accel: "<<accel_and_brake<<"\n";
        std::cout<<"Steer: "<<steer<<std::endl;
        
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
        float targetAngle = cs.getAngle()-cs.getTrackPos()*0.1;
        //std::cout<<"Angle: "<<targetAngle*180.0/M_PI<<"\n";
        float steer = targetAngle/parameters.steerLock;
        steer = std::max(std::min(1.0f, steer),-1.0f);
        return steer;
    }

    /*bool straight = true;
    for(int i = 1; i < 300 && nOfRecords[distance+i] > 0; ++i){
        if(std::fabs(steerRecords[distance+i]/nOfRecords[distance+i]) > 0.2){
            straight = false;
            break;
        }
    }
    if(straight)
        for(int i = 300; i < 1500 && nOfRecords[distance+i] > 0; ++i){
            if(steerRecords[distance+i]/nOfRecords[distance+i] > 0.2617993878){
                float steer = (1.0-cs.getTrackPos())*0.1;
                steer = std::max(std::min(1.0f, steer),-1.0f);
                //std::cout<<"################################"<<steer;
                return steer;
            }
            else if(steerRecords[distance+i]/nOfRecords[distance+i] < 0.2617993878){
                float steer = (-1.0-cs.getTrackPos())*0.1;
                steer = std::max(std::min(1.0f, steer),-1.0f);
                //std::cout<<"################################"<<steer;
                return steer;
            }
        }*/

    float maxDistance = cs.getTrack(0);
    float maxAngle = MyDriver1::angles[0];
    for( int i = 1; i < MyDriver1::angles.size(); ++i){
        float distance = cs.getTrack(i);
        //std::cerr<<angles[i]<<":"<<distance<<"\n";
        if(distance > maxDistance){
            maxDistance = distance;
            maxAngle = MyDriver1::angles[i];
        }
    }
    

    int distance = cs.getDistFromStart();
    int minDistance = std::max((int)cs.getSpeedX() - 140, 40);
    if(lap > 1 && (cs.getTrack(9) >  minDistance 
                    || cs.getTrack(8) >  minDistance 
                    || cs.getTrack(7) >  minDistance 
                    || cs.getTrack(10) >  minDistance
                    || cs.getTrack(11) >  minDistance)){
        for(int i = minDistance; i < 600; ++i){
            int distIndex = (distance+i)%(trackDistance+1);
            if(steerRecords[distIndex]/nOfRecords[distIndex] > 0.35){
                std::cout<<"Left curve at the end\n";
                float targetAngle = cs.getAngle()+(-1.0-cs.getTrackPos())*0.1;
                float steer = targetAngle/parameters.steerLock;
                steer = std::max(std::min(0.095f, steer),-0.095f);
                return steer;
            }else if(steerRecords[distIndex]/nOfRecords[distIndex] < -0.35){
                std::cout<<"Right curve at the end\n";
                float targetAngle = cs.getAngle()+(1.0-cs.getTrackPos())*0.1;
                float steer = targetAngle/parameters.steerLock;
                steer = std::max(std::min(0.095f, steer),-0.0952f);
                return steer;
            }
        }
    }
    std::cout<<"Angle1: "<<maxAngle<<"\n";
    float trackPos = cs.getTrackPos();
    std::cout<<"trackPos1: "<<trackPos<<"\n";
    steerRecords[distance] += -(maxAngle*M_PI/180.0)/parameters.steerLock;
    nOfRecords[distance]++;
    if(maxAngle > 0){
        trackPos += 1;
        trackPos/=2.0;
    }else{
        trackPos -= 1;
        trackPos/=-2.0;
    }
    maxAngle *= (trackPos+0.25);
    std::cout<<"trackPos2: "<<trackPos<<"\n";
    std::cout<<"Angle2: "<<maxAngle<<"\n";
    //std::cout<<"Angle2: "<<maxAngle<<"\n";
    //std::cout<<"Angle: "<<-(maxAngle*M_PI/180.0)<<"\n";
    float steer = -(maxAngle*M_PI/180.0)/parameters.steerLock;
    steer = std::max(std::min(1.0f, steer),-1.0f);
    return steer;
}

void
MyDriver1::init(float *angles)
{

    for(int i = 0; i < MyDriver1::angles.size(); ++i)
        angles[i] = MyDriver1::angles[i];
}