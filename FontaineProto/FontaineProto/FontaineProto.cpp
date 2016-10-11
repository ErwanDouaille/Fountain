// SourceCalibration.cpp : Defines the entry point for the console application.
//

#include "lo/lo.h"
//Always put Windows.h after lo/lo.h
#include <Windows.h>
// DO NOT FORGET TO NOT USE MSVCRT in LINKER

#include <stdio.h>      /* printf */
#include <signal.h>     /* signal, raise, sig_atomic_t */
#include <iostream> 
#include <stdlib.h>
#include <math.h>  
#include <fstream>
#include <string>
#include <stdlib.h>
#include <vector>

#include "LgEnvironment.h"

#include "DepthHandsFromSkyGenerator2.h"
#include "OneEuroFilterProcessor.h"
#include "BlasterObserver.h"
#include "OpenCVDrawObserver.h"
#include "CmdGlobObserver.h"
#include "OneDollarRecognizerObserver.h"

using namespace std;
using namespace lg;

bool sortie = false, debugWindow = false;
bool hasDoneAimant = false, hasDoneGesture = false, controlPosition = false;
int fountainHeight, bodySize, pastUsers = -1, removeNbFrames = 10;

__int64 gestureDelay, savedGestureTime;
__int64 aimantationToGestureDelay, savedAimantationToGestureTime;
__int64 beforeAimantationDelay, savedBeforeAimantationTime;

//, aimantationDelayForGesture, savedDelayAimantation, savedDelayBeforeAimantation, delayBeforeAimantation;
float handsPerimeter;
lo_address client;
string ipAdress, port, removeBackgroudDirectory;
Environment* myEnv;

void killHandler (int param)
{
	sortie = true;
}

bool canDoAimant()
{
	return savedBeforeAimantationTime + beforeAimantationDelay <  myEnv->getTime();
}

bool canDoGesture()
{
	return (savedGestureTime + gestureDelay < myEnv->getTime() &&
		savedAimantationToGestureTime + aimantationToGestureDelay < myEnv->getTime() &&
		!hasDoneAimant) || 
		controlPosition;
}

void gestureTimer()
{
	savedGestureTime = hasDoneGesture ? myEnv->getTime() : savedGestureTime + gestureDelay < myEnv->getTime() ? 0 : savedGestureTime;
}

void aimantTimer()
{
	savedBeforeAimantationTime = myEnv->getTime();
}

void sendNbUsers(DepthHandsFromSkyGenerator2* gt)
{
	int currentUsers =  gt->getEllipses().size();
	if(currentUsers != pastUsers)
	{
		if (lo_send(client, "/qqun", "i", currentUsers) == -1) // controlled blaster and hauteur
			printf("OSC error %d: %s\n", lo_address_errno(client), lo_address_errstr(client));
		pastUsers = currentUsers;
	}
}

bool globalCommand(CmdGlobObserver* cmd)
{
	if(cmd->getCmdName().compare("") == 0)
	{
		hasDoneGesture = false;
		return false;
	}
	string name = "/cmdGlob/" + cmd->getCmdName();
	if(debugWindow)
		cout << name << "\t" << cmd->getSpeed() << "\t" << cmd->getAmplitude() << endl;
	controlPosition = name.compare("/cmdGlob/ctrlPos") == 0 ? true : false;
	if(lo_send(client, name.c_str(), "fffff", cmd->getSpeed(), cmd->getAmplitude(), cmd->getDirection().getX(), cmd->getDirection().getY(), cmd->getDirection().getZ()) == -1) // controlled blaster and hauteur
		printf("OSC error %d: %s\n", lo_address_errno(client), lo_address_errstr(client));
	hasDoneGesture = true;
	return hasDoneGesture;
}

bool gestureRecognition( OneDollarRecognizerObserver* odr)
{
	float highest = -1.0f;
	string highestGroup = "";
	map<string,float> probas;
	probas = odr->getProbabilities();
	for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++){
		if(pit->second > highest){
			highest = pit->second;
			highestGroup = pit->first;
		}
	}
	if(highest>0.7 )
	{
		hasDoneGesture = true;
		if(highestGroup.compare("Circle") == 0)
		{
			if(debugWindow)
				cout << "Circle " << endl;
			if (lo_send(client, "/cmdAnim/tourne", "i" , 1) == -1) // controlled blaster and hauteur
				printf("OSC error %d: %s\n", lo_address_errno(client), lo_address_errstr(client));
		}
		if(highestGroup.compare("Circle_Inv") == 0)
		{			
			if(debugWindow)
				cout << "Circle Inv " << endl;
			if (lo_send(client, "/cmdAnim/tourne", "i" , -1) == -1) // controlled blaster and hauteur
				printf("OSC error %d: %s\n", lo_address_errno(client), lo_address_errstr(client));
		}
	}
	controlPosition = false;
	return hasDoneGesture;
}

bool blasterControl( BlasterObserver* bobs)
{
	//hasDoneAimant = false;
	bool doingAimant = false;
	bool canReboot = true;
	map<string,float> probas = bobs->getProbabilities();
	for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++){
		if(pit->second > 0.8f){
			canReboot = false;
			if(hasDoneAimant)
			{
				if(debugWindow) 
					cout << "aimant " << bobs->getJet(pit->first) << " " << bobs->getHauteur(pit->first) << endl;
				if (lo_send(client, "/aimant", "if" ,bobs->getJet(pit->first) , bobs->getHauteur(pit->first)) == -1) // controlled blaster and hauteur
					printf("OSC error %d: %s\n", lo_address_errno(client), lo_address_errstr(client));
			}
			else 
			{
				doingAimant = true;
			}
		}
	}
	if (canReboot)
		hasDoneAimant = false;

	if (doingAimant)
	{
		hasDoneAimant = true;
		aimantTimer();
	}
	controlPosition = hasDoneAimant ? false : controlPosition;
	return hasDoneAimant;
}

bool setup()
{
	string line;
	ifstream myfile ("init_proto.txt");
	if (myfile.is_open())
	{
		getline (myfile,line);
		getline (myfile,line);
		ipAdress = line;

		getline (myfile,line);
		getline (myfile,line);
		port = line;

		getline (myfile,line);
		getline (myfile,line);
		fountainHeight = atoi(line.c_str());

		getline (myfile,line);
		getline (myfile,line);
		debugWindow = atoi(line.c_str());

		getline (myfile,line);
		getline (myfile,line);
		bodySize = atoi(line.c_str());

		getline (myfile,line);
		getline (myfile,line);
		gestureDelay = atoi(line.c_str());

		getline (myfile,line);
		getline (myfile,line);
		aimantationToGestureDelay = atoi(line.c_str());

		getline (myfile,line);
		getline (myfile,line);
		removeNbFrames = atoi(line.c_str());

		getline (myfile,line);
		getline (myfile,line);
		removeBackgroudDirectory = line;

		getline (myfile,line);
		getline (myfile,line);
		handsPerimeter = atof(line.c_str());

		getline (myfile,line);
		getline (myfile,line);
		beforeAimantationDelay = atoi(line.c_str());

		myfile.close();
	}
	else 
		return false;
	return true;
}

int main(int argc, char* argv[])
{
	/************************************************************** INITIALISATION
	******************************************************************************/
	signal (SIGBREAK, killHandler);

	if (setup())
		printf("Init ... OK\n");
	else
	{
		printf("Init ... error.\n");
		return 2;
	}

	myEnv = new Environment();
	myEnv->enableDataCopy(false);
	myEnv->setHistoricLength(10);

	savedGestureTime = myEnv->getTime();
	savedBeforeAimantationTime = myEnv->getTime();
	savedAimantationToGestureTime = myEnv->getTime();

	/******************************************************************* GENERATOR
	******************************************************************************/
	DepthHandsFromSkyGenerator2* gt = new DepthHandsFromSkyGenerator2("DepthHandsFromSkyGenerator"); 
	gt->setBodySize(bodySize);
	gt->setFountainHeight(fountainHeight);
	gt->setRemoveNBFrames(removeNbFrames);
	gt->setRemoveBackgroundDirectory(removeBackgroudDirectory);
	gt->setHandsPerimeter(handsPerimeter);
	if(myEnv->registerNode(gt))
		printf("Register DepthHandsFromSkyGenerator OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());

	/******************************************************************* PROCESSOR
	******************************************************************************/
	OneEuroFilterProcessor* fp = new OneEuroFilterProcessor("One Euro Filter Processor");
	fp->onlyProcessGroupType("Kinectv2_Up_RightHands");
	if(myEnv->registerNode(fp))
		printf("Register Filter OK.\n");
	else{
		printf("Register Filter NOT OK.\n");
		return 2;
	}

	/******************************************************************** OBSERVER
	******************************************************************************/
	BlasterObserver* bobs = new BlasterObserver(); 
	bobs->onlyObserveGroupType(fp->getGeneratedGroupType());
	bobs->setFountainHeight(fountainHeight);
	if(myEnv->registerNode(bobs))
		printf("Register BlasterObserver OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());

	CmdGlobObserver* globCmd = new CmdGlobObserver();
	globCmd->onlyObserveGroupType(fp->getGeneratedGroupType());
	globCmd->setHauteurCamera(fountainHeight);
	if(myEnv->registerNode(globCmd))
		printf("Register CmdGlobObserver OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());
	
	OneDollarRecognizerObserver* odr = new OneDollarRecognizerObserver("OneDollarRecognizerObserver");
	odr->onlyObserveGroupType(fp->getGeneratedGroupType());
	if(myEnv->registerNode(odr))
		printf("Register OneDollarRecognizerObserver OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());

	if(debugWindow) 
	{
		OpenCVDrawObserver* drawer = new OpenCVDrawObserver(); 
		drawer->setDepthHandsFromSkyGenerator(gt);
		drawer->setBlasterObserver(bobs);
		drawer->onlyObserveGroupType(fp->getGeneratedGroupType());
		if(myEnv->registerNode(drawer))
			printf("Register OpenCVDrawObserver OK.\n");
		else
			printf("%s.\n",myEnv->getLastError().c_str());
	}

	/*********************************************************************** START 
	******************************************************************************/
	if(myEnv->start())
		printf("Start environment OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());
		
	client = lo_address_new(ipAdress.c_str(), port.c_str());

	/***************************************************************** UPDATE LOOP
	******************************************************************************/
	while(!sortie){		
		myEnv->update();
		
		if(canDoAimant())	
		{
			blasterControl(bobs);
		}
		
		if(canDoGesture())
		{
			globalCommand(globCmd);
			if(!hasDoneGesture)
				gestureRecognition(odr);
			gestureTimer();
		}
		sendNbUsers(gt);
		Sleep(1);
	}

	myEnv->stop();
	return 0;
}

