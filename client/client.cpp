/***************************************************************************
 
    file                 : client.cpp
    copyright            : (C) 2007 Daniele Loiacono
 
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
/* Uncomment the following lines under windows */
//#define WIN32 // maybe not necessary because already define
//#define __DRIVER_CLASS__ SimpleDriver     // put here the name of your driver class
//#define __DRIVER_INCLUDE__ "SimpleDriver.h" // put here the filename of your driver h\\eader

#ifdef WIN32
#include <WinSock.h>
#else
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#endif

#include <iostream>
#include <fstream>
#include "json/json.hpp"
#include <cstdlib>
#include <cstdio>
#include __DRIVER_INCLUDE__
#include "BrakeTester.h"
#include "DriverParameters.h"

/*** defines for UDP *****/
#define UDP_MSGLEN 1000
#define UDP_CLIENT_TIMEUOT 1000000
//#define __UDP_CLIENT_VERBOSE__
/************************/

#ifdef WIN32
typedef sockaddr_in tSockAddrIn;
#define CLOSE(x) closesocket(x)
#define INVALID(x) x == INVALID_SOCKET
#else
typedef int SOCKET;
typedef struct sockaddr_in tSockAddrIn;
#define CLOSE(x) close(x)
#define INVALID(x) x < 0
#endif

class __DRIVER_CLASS__;
typedef __DRIVER_CLASS__ tDriver;


using namespace std;

//void parse_args(int argc, char *argv[], char *hostName, unsigned int &serverPort, char *id, unsigned int &maxEpisodes,unsigned int &maxSteps,
//		bool &noise, double &noiseAVG, double &noiseSTD, long &seed, char *trackName, BaseDriver::tstage &stage);
void parse_args(int argc, char *argv[], char *hostName, unsigned int &serverPort, char *id, unsigned int &maxEpisodes,
		  unsigned int &maxSteps, char *trackName, BaseDriver::tstage &stage, bool &brakeTest, DriverParameters& parameters);
void parse_json(char* filename, DriverParameters& parameters);

int main(int argc, char *argv[])
{
    SOCKET socketDescriptor;
    int numRead;

    char hostName[1000];
    unsigned int serverPort;
    char id[1000];
    unsigned int maxEpisodes;
    unsigned int maxSteps;
//    bool noise;
//    double noiseAVG;
//    double noiseSTD;
//    long seed;
    char trackName[1000];
    BaseDriver::tstage stage;
    bool brakeTest;
    DriverParameters parameters;

    tSockAddrIn serverAddress;
    struct hostent *hostInfo;
    struct timeval timeVal;
    fd_set readSet;
    char buf[UDP_MSGLEN];


#ifdef WIN32 
     /* WinSock Startup */

     WSADATA wsaData={0};
     WORD wVer = MAKEWORD(2,2);
     int nRet = WSAStartup(wVer,&wsaData);

     if(nRet == SOCKET_ERROR)
     {
 	std::cout << "Failed to init WinSock library" << std::endl;
	exit(1);
     }
#endif

//    parse_args(argc,argv,hostName,serverPort,id,maxEpisodes,maxSteps,noise,noiseAVG,noiseSTD,seed,trackName,stage);
    parse_args(argc,argv,hostName,serverPort,id,maxEpisodes,maxSteps,trackName,stage, brakeTest, parameters);

//    if (seed>0)
//    	srand(seed);
//    else
//    	srand(time(NULL));

    hostInfo = gethostbyname(hostName);
    if (hostInfo == NULL)
    {
        cout << "Error: problem interpreting host: " << hostName << "\n";
        exit(1);
    }

    // Print command line option used
    cout << "***********************************" << endl;

    cout << "HOST: "   << hostName    << endl;

    cout << "PORT: " << serverPort  << endl;

    cout << "ID: "   << id     << endl;

    cout << "MAX_STEPS: " << maxSteps << endl; 

    cout << "MAX_EPISODES: " << maxEpisodes << endl;

//    if (seed>0)
//    	cout << "SEED: " << seed << endl;

    cout << "TRACKNAME: " << trackName << endl;

    if (stage == BaseDriver::WARMUP)
		cout << "STAGE: WARMUP" << endl;
	else if (stage == BaseDriver::QUALIFYING)
		cout << "STAGE:QUALIFYING" << endl;
	else if (stage == BaseDriver::RACE)
		cout << "STAGE: RACE" << endl;
	else
		cout << "STAGE: UNKNOWN" << endl;

	cout << "***********************************" << endl;
    // Create a socket (UDP on IPv4 protocol)
    socketDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
    if (INVALID(socketDescriptor))
    {
        cerr << "cannot create socket\n";
        exit(1);
    }

    // Set some fields in the serverAddress structure.
    serverAddress.sin_family = hostInfo->h_addrtype;
    memcpy((char *) &serverAddress.sin_addr.s_addr,
           hostInfo->h_addr_list[0], hostInfo->h_length);
    serverAddress.sin_port = htons(serverPort);

    SimpleDriver *driver;
    if(brakeTest)
        driver = new BrakeTester(parameters);
    else 
        driver = new tDriver(parameters);
    SimpleDriver &d = *driver;
    strcpy(d.trackName,trackName);
    d.stage = stage;

    bool shutdownClient=false;
    unsigned long curEpisode=0;
    do
    {
        /***********************************************************************************
        ************************* UDP client identification ********************************
        ***********************************************************************************/
        do
        {
        	// Initialize the sensorsAngles of rangefinders
        	float sensorsAngles[19];
        	d.init(sensorsAngles);
        	string initString = SimpleParser::stringify(string("init"),sensorsAngles,19);
            cout << "Sending id to server: " << id << endl;
            initString.insert(0,id);
            cout << "Sending init string to the server: " << initString << endl;
            if (sendto(socketDescriptor, initString.c_str(), initString.length(), 0,
                       (struct sockaddr *) &serverAddress,
                       sizeof(serverAddress)) < 0)
            {
                cerr << "cannot send data ";
                CLOSE(socketDescriptor);
                exit(1);
            }

            // wait until answer comes back, for up to UDP_CLIENT_TIMEUOT micro sec
            FD_ZERO(&readSet);
            FD_SET(socketDescriptor, &readSet);
            timeVal.tv_sec = 0;
            timeVal.tv_usec = UDP_CLIENT_TIMEUOT;

            if (select(socketDescriptor+1, &readSet, NULL, NULL, &timeVal))
            {
                // Read data sent by the solorace server
                memset(buf, 0x0, UDP_MSGLEN);  // Zero out the buffer.
                numRead = recv(socketDescriptor, buf, UDP_MSGLEN, 0);
                if (numRead < 0)
                {
                    cerr << "didn't get response from server...";
                }
		else
		{
                	cout << "Received: " << buf << endl;

                	if (strcmp(buf,"***identified***")==0)
                    		break;
            	}
	      }

        }  while(1);

	unsigned long currentStep=0; 

        while(1)
        {
            // wait until answer comes back, for up to UDP_CLIENT_TIMEUOT micro sec
            FD_ZERO(&readSet);
            FD_SET(socketDescriptor, &readSet);
            timeVal.tv_sec = 0;
            timeVal.tv_usec = UDP_CLIENT_TIMEUOT;

            if (select(socketDescriptor+1, &readSet, NULL, NULL, &timeVal))
            {
                // Read data sent by the solorace server
                memset(buf, 0x0, UDP_MSGLEN);  // Zero out the buffer.
                numRead = recv(socketDescriptor, buf, UDP_MSGLEN, 0);
                if (numRead < 0)
                {
                    cerr << "didn't get response from server?";
                    CLOSE(socketDescriptor);
                    exit(1);
                }

#ifdef __UDP_CLIENT_VERBOSE__
                cout << "Received: " << buf << endl;
#endif

                if (strcmp(buf,"***shutdown***")==0)
                {
                    d.onShutdown();
                    shutdownClient = true;
                    cout << "Client Shutdown" << endl;
                    break;
                }

                if (strcmp(buf,"***restart***")==0)
                {
                    d.onRestart();
                    cout << "Client Restart" << endl;
                    break;
                }
                /**************************************************
                 * Compute The Action to send to the solorace sever
                 **************************************************/

		if ( (++currentStep) != maxSteps)
		{
                	string action = d.drive(string(buf));
                	memset(buf, 0x0, UDP_MSGLEN);
			sprintf(buf,"%s",action.c_str());
		}
		else
			sprintf (buf, "(meta 1)");

                if (sendto(socketDescriptor, buf, strlen(buf)+1, 0,
                           (struct sockaddr *) &serverAddress,
                           sizeof(serverAddress)) < 0)
                {
                    cerr << "cannot send data ";
                    CLOSE(socketDescriptor);
                    exit(1);
                }
#ifdef __UDP_CLIENT_VERBOSE__
                else
                    cout << "Sending " << buf << endl;
#endif
            }
            else
            {
                cout << "** Server did not respond in 1 second.\n";
            }
        }
    } while(shutdownClient==false && ( (++curEpisode) != maxEpisodes) );

    if (shutdownClient==false)
	d.onShutdown();
    CLOSE(socketDescriptor);
#ifdef WIN32
    WSACleanup();
#endif
    return 0;

}

//void parse_args(int argc, char *argv[], char *hostName, unsigned int &serverPort, char *id, unsigned int &maxEpisodes,
//		  unsigned int &maxSteps,bool &noise, double &noiseAVG, double &noiseSTD, long &seed, char *trackName, BaseDriver::tstage &stage)
void parse_args(int argc, char *argv[], char *hostName, unsigned int &serverPort, char *id, unsigned int &maxEpisodes,
		  unsigned int &maxSteps, char *trackName, BaseDriver::tstage &stage, bool &brakeTest, DriverParameters& parameters)
{
    int		i;

    // Set default values
    maxEpisodes=0;
    maxSteps=0;
    serverPort=3001;
    strcpy(hostName,"localhost");
    strcpy(id,"SCR");
//    noise=false;
//    noiseAVG=0;
//    noiseSTD=0.05;
//    seed=0;
    strcpy(trackName,"unknown");
    stage=BaseDriver::UNKNOWN;
    brakeTest = false;


    i = 1;
    while (i < argc)
    {
    	if (strncmp(argv[i], "host:", 5) == 0)
    	{
    		sprintf(hostName, "%s", argv[i]+5);
    		i++;
    	}
    	else if (strncmp(argv[i], "port:", 5) == 0)
    	{
    		sscanf(argv[i],"port:%d",&serverPort);
    		i++;
    	}
    	else if (strncmp(argv[i], "id:", 3) == 0)
    	{
    		sprintf(id, "%s", argv[i]+3);
    		i++;
	    }
    	else if (strncmp(argv[i], "maxEpisodes:", 12) == 0)
    	{
    		sscanf(argv[i],"maxEpisodes:%ud",&maxEpisodes);
    	    i++;
    	}
    	else if (strncmp(argv[i], "maxSteps:", 9) == 0)
    	{
    		sscanf(argv[i],"maxSteps:%ud",&maxSteps);
    		i++;
    	}
//    	else if (strncmp(argv[i], "seed:", 5) == 0)
//    	{
//    	    sscanf(argv[i],"seed:%ld",&seed);
//    	    i++;
//    	}
    	else if (strncmp(argv[i], "maxSpeed:", 9) == 0)
    	{
            sscanf(argv[i],"maxSpeed:%f",&parameters.maxSpeed);
            i++;
    	}
    	else if (strncmp(argv[i], "track:", 6) == 0)
    	{
            sscanf(argv[i],"track:%s",trackName);
            i++;
    	}
    	else if (strncmp(argv[i], "stage:", 6) == 0)
    	{
            int temp;
            sscanf(argv[i],"stage:%d",&temp);
            stage = (BaseDriver::tstage) temp;
            i++;
            if (stage<BaseDriver::WARMUP || stage > BaseDriver::RACE)
                stage = BaseDriver::UNKNOWN;
    	}
        else if (strncmp(argv[i], "json:", 5) == 0)
    	{
            char filename[99];
            sscanf(argv[i],"json:%s",filename);
            parse_json(filename, parameters);
            i++;
    	}
        else if (strncmp(argv[i], "brakeTest", 9) == 0)
    	{
            brakeTest = true;
            i++;
    	}
    	else {
    		i++;		/* ignore bad args */
    	}
    }
}

void parse_json(char* filename, DriverParameters& parameters){
    std::ifstream file(filename);
    nlohmann::json j;
    file >> j;

    if(j.contains("sensorsAngles")){
        std::cout<<"Read from file sensorsAngles: ";
        const auto sensorsAngles = j["sensorsAngles"];
        for(uint i = 0; i < parameters.sensorsAngles.size(); ++i){
            std::cout<<sensorsAngles.at(i)<<", ";
            parameters.sensorsAngles[i] = sensorsAngles.at(i);
        }
        std::cout<<std::endl;
    }
    if(j.contains("lowAnglesSensorsIndexes")){
        std::cout<<"Read from file lowAnglesSensorsIndexes: ";
        const auto indexes = j["lowAnglesSensorsIndexes"];
        parameters.lowAnglesSensorsIndexes.clear ();
        for(int index: indexes){
            std::cout<<index<<", ";
            parameters.lowAnglesSensorsIndexes.push_back(index);
        }
        std::cout<<std::endl;
    }
    if(j.contains("gearUp")){
        std::cout<<"Read from file gearUp array: ";
        const auto gearUp = j["gearUp"];
        for(uint i = 0; i < parameters.gearUp.size(); ++i){
            std::cout<<gearUp.at(i)<<", ";
            parameters.gearUp[i] = gearUp.at(i);
        }
        std::cout<<std::endl;
    }
    if(j.contains("gearDown")){
        std::cout<<"Read from file gearDown array: ";
        const auto gearDown = j["gearDown"];
        for(uint i = 0; i < parameters.gearDown.size(); ++i){
            std::cout<<gearDown.at(i)<<", ";
            parameters.gearDown[i] = gearDown.at(i);
        }
        std::cout<<std::endl;
    }
    if(j.contains("firstLapSpeed")){
        std::cout<<"Read from file firstLapSpeed: "<<j["firstLapSpeed"]<<std::endl;
        parameters.firstLapSpeed = j["firstLapSpeed"];
    }
    if(j.contains("maxSpeedSteer")){
        std::cout<<"Read from file maxSpeedSteer: "<<j["maxSpeedSteer"]<<std::endl;
        parameters.maxSpeedSteer = j["maxSpeedSteer"];
    }  
    if(j.contains("losingSpeedPerMeter")){
        std::cout<<"Read from file losingSpeedPerMeter: "<<j["losingSpeedPerMeter"]<<std::endl;
        parameters.losingSpeedPerMeter = j["losingSpeedPerMeter"];
    }  
    if(j.contains("brakeDelay")){
        std::cout<<"Read from file brakeDelay: "<<j["brakeDelay"]<<std::endl;
        parameters.brakeDelay = j["brakeDelay"];
    } 
    if(j.contains("powSteerPenalty")){
        std::cout<<"Read from file powSteerPenalty: "<<j["powSteerPenalty"]<<std::endl;
        parameters.powSteerPenalty = j["powSteerPenalty"];
    } 
    if(j.contains("returnTrackFactor")){
        std::cout<<"Read from file returnTrackFactor: "<<j["returnTrackFactor"]<<std::endl;
        parameters.returnTrackFactor = j["returnTrackFactor"];
    }
    if(j.contains("explorationDistance")){
        std::cout<<"Read from file explorationDistance: "<<j["explorationDistance"]<<std::endl;
        parameters.explorationDistance = j["explorationDistance"];
    }
    if(j.contains("sharpCurveSteerTreshold")){
        std::cout<<"Read from file sharpCurveSteerTreshold: "<<j["sharpCurveSteerTreshold"]<<std::endl;
        parameters.sharpCurveSteerTreshold = j["sharpCurveSteerTreshold"];
    }
    if(j.contains("minDistanceToSharpCurve")){
        std::cout<<"Read from file minDistanceToSharpCurve: "<<j["minDistanceToSharpCurve"]<<std::endl;
        parameters.minDistanceToSharpCurve = j["minDistanceToSharpCurve"];
    }
    if(j.contains("minSpeed")){
        std::cout<<"Read from file minSpeed: "<<j["minSpeed"]<<std::endl;
        parameters.minSpeed = j["minSpeed"];
    }
    if(j.contains("maxSpeed")){
        std::cout<<"Read from file maxSpeed: "<<j["maxSpeed"]<<std::endl;
        parameters.maxSpeed = j["maxSpeed"];
    }
    if(j.contains("logLevel")){
        std::cout<<"Read from file logLevel: "<<j["logLevel"]<<std::endl;
        parameters.logLevel = j["logLevel"];
    }
    if(j.contains("desiredToSteerAngleFactor")){
        std::cout<<"Read from file desiredToSteerAngleFactor: "<<j["desiredToSteerAngleFactor"]<<std::endl;
        parameters.desiredToSteerAngleFactor = j["desiredToSteerAngleFactor"];
    }
    if(j.contains("adjustTrackBeforeSharpCurveFactor")){
        std::cout<<"Read from file adjustTrackBeforeSharpCurveFactor: "<<j["adjustTrackBeforeSharpCurveFactor"]<<std::endl;
        parameters.adjustTrackBeforeSharpCurveFactor = j["adjustTrackBeforeSharpCurveFactor"];
    }
    if(j.contains("maxSteerBeforeSharpCurveFactor")){
        std::cout<<"Read from file maxSteerBeforeSharpCurveFactor: "<<j["maxSteerBeforeSharpCurveFactor"]<<std::endl;
        parameters.maxSteerBeforeSharpCurveFactor = j["maxSteerBeforeSharpCurveFactor"];
    }
    if(j.contains("brakeStartTestingSpeed")){
        std::cout<<"Read from file brakeStartTestingSpeed: "<<j["brakeStartTestingSpeed"]<<std::endl;
        parameters.brakeStartTestingSpeed = j["brakeStartTestingSpeed"];
    }
}
