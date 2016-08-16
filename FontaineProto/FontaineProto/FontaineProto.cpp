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
#include <vector>

#include "LgEnvironment.h"

#include "DepthHandsFromSkyGenerator2.h"
#include "FilteringProcessor.h"
#include "BlasterObserver.h"
#include "SynchroDualUpObserver.h"
#include "SynchroDualDownObserver.h"

using namespace std;
using namespace lg;


bool sortie = false;

void killHandler (int param)
{
	sortie = true;
}


int main(int argc, char* argv[])
{
	signal (SIGBREAK, killHandler);


	Environment* myEnv = new Environment();

	//cout << myEnv->dataCopyEnabled();
	// Disable data copy --> TODO retirer cet option
	myEnv->enableDataCopy(false);


	//TODO debug aimant

	// TODO add generator and observers

	DepthHandsFromSkyGenerator2* gt = new DepthHandsFromSkyGenerator2("DepthHandsFromSkyGenerator"); 
	if(myEnv->registerNode(gt))
		printf("Register DepthHandsFromSkyGenerator OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());



	//  a filtering processor
	FilteringProcessor* fp = new FilteringProcessor("Filtering Processor");
	fp->onlyProcessGroupType("Kinectv2_Up_RightHands");
	fp->setCutOffFrequency(2.0);
	if(myEnv->registerNode(fp))
		printf("Register Filter OK.\n");
	else{
		printf("Register Filter NOT OK.\n");
		return 2;
	}

	BlasterObserver* bobs = new BlasterObserver(); 
	bobs->onlyObserveGroupType(fp->getGeneratedGroupType());
	if(myEnv->registerNode(bobs))
		printf("Register BlasterObserver OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());
	
	SynchroDualUpObserver* syncUp = new SynchroDualUpObserver(); 
	syncUp->onlyObserveGroupType(fp->getGeneratedGroupType());
	if(myEnv->registerNode(syncUp))
		printf("Register SynchroDualUpObserver OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());

	//SynchroDualDownObserver* syncDown= new SynchroDualDownObserver(); 
	//syncDown->onlyObserveGroupType(fp->getGeneratedGroupType());
	//if(myEnv->registerNode(syncDown))
	//	printf("Register SynchroDualDownObserver OK.\n");
	//else
	//	printf("%s.\n",myEnv->getLastError().c_str());

	if(myEnv->start())
		printf("Register BlasterObserver OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());

	// create client
	lo_address client;
	//client = lo_address_new("134.206.11.110", "3333");
	client = lo_address_new("192.168.1.1", "9998");

	while(!sortie){
		myEnv->update();

		//cout << "Nb groups = " << myEnv->getGroups3D().size() << endl;
		//cout << "Nb proba groups = " << bobs->getProbabilities().size() << endl;

		bool hasDoneGesture = false;



		// If observer de main au dessus de blaster a vrai -> rien d autres
		map<string,float> probas = bobs->getProbabilities();
		for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++){
			if(pit->second > 0.8f){
				hasDoneGesture = true;
				//highest = pit->second;
				//highestGroup = pit->first;

				//cout << "/aimant " << bobs->getJet(pit->first) << " with " << bobs->getHauteur(pit->first) << endl;
				//  send message with parameters
				if (lo_send(client, "/aimant", "if" ,bobs->getJet(pit->first) , bobs->getHauteur(pit->first)) == -1) // controlled blaster and hauteur
					printf("OSC error %d: %s\n", lo_address_errno(client), lo_address_errstr(client));
			}
		}

		if(!hasDoneGesture){
			
			if(syncUp->getProbability(to_string(0)) == 1.0)
				cout << "up " << syncUp->getSpeed() << endl;
				//if (lo_send(client, "/cmdGlobe/leve ", "ff" ,syncUp->getSpeed(), syncUp->getAmplitude()) == -1)
				//	printf("OSC error %d: %s\n", lo_address_errno(client), lo_address_errstr(client));
			
			//if(syncDown->getProbability(to_string(0)) == 1.0)
				/*if (lo_send(client, "/cmdGlobe/baisse", "ff" ,syncDown->getSpeed(), syncDown->getAmplitude()) == -1)
					printf("OSC error %d: %s\n", lo_address_errno(client), lo_address_errstr(client));*/
		}

		Sleep(1);
	}

	myEnv->stop();


	//system("pause");

	return 0;
}

