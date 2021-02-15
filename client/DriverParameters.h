#pragma once

#include <array>
#include <vector>
#include <cmath>

struct DriverParameters{
	/* Gear Changing Constants*/
	// RPM values to change gear 
	std::array<int, 6> gearUp = {7800,8000,8500,8500,8500,0};
	std::array<int, 6> gearDown = {0,4300,5000,6000,6000,6000};
		
	/* Stuck constants*/
	
	// How many time steps the controller wait before recovering from a stuck position
	int stuckTime = 30;
	// When car angle w.r.t. track axis is grather tan stuckAngle, the car is probably stuck
	float stuckAngle = 2*M_PI/6; //60 degree
	
	/* Steering constants*/
	
	// Angle associated to a full steer command
	float steerLock = 0.366519;	
	// Min speed to reduce steering command 
	float steerSensitivityOffset = 80.0;
	// Coefficient to reduce steering command at high speed (to avoid loosing the control)
	float wheelSensitivityCoeff = 1;
	
	/* Accel and Brake Constants*/
	
	// max speed allowed
	float maxSpeed = 300;
	// Min distance from track border to drive at  max speed
	float maxSpeedDist = 70;

	
	/* ABS Filter Constants */
	
	// Radius of the 4 wheels of the car
	float wheelRadius[4] = {0.3306,0.3306,0.3276,0.3276};
	// min slip to prevent ABS
	float absSlip = 2.0;						
	// range to normalize the ABS effect on the brake
	float absRange = 3.0;
	// min speed to activate ABS
	float absMinSpeed = 3.0;

	/* Clutch constants */
	float clutchMax = 0.5;
	float clutchDelta = 0.05;
	float clutchRange = 0.82;
	float clutchDeltaTime = 0.02;
	float clutchDeltaRaced = 10;
	float clutchDec = 0.01;
	float clutchMaxModifier = 1.3;
	float clutchMaxTime = 1.5;

	//MyDriver
	int stuckRecoverTime = 200;

	int startRecording = 150;
	float tresholdAngle = 2*M_PI/72; //5 degree
	float angleToSpeedCoef = 3 / (M_PI/180); //3 km/h per 1 degree
	int sforwardDistance = 0;
	int eforwardDistance = 25;
	
	int minSpeed = 50;

	//MyDriver1
	std::array<float, 19> sensorsAngles = {-50,-40,-32,-24,-16,-8,-4,-2,-1,0, 1,2,4,8, 16,24,32,40,50};

	float firstLapSpeed = 100;

	float explorationDistance = 200; //m
	float maxSpeedSteer = 0.1; //2 degree 
	float losingSpeedPerMeter = 2.0;
	int brakeDelay = 5;
	float powSteerPenalty = 1.1;

	float returnTrackFactor = 0.1;

	float sharpCurveSteerTreshold = 0.375; 
	int minDistanceToSharpCurve = 45;
	std::vector<int> lowAnglesSensorsIndexes = {7,8,9,10,11};
	float desiredToSteerAngleFactor = 0.25;
	float adjustTrackBeforeSharpCurveFactor = 0.05;
	float maxSteerBeforeSharpCurveFactor = 0.008;

	//BrakeTester
	float brakeStartTestingSpeed = 290;

	int logLevel = 1;
};