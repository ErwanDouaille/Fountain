// FontaineGestureTest.cpp : Defines the entry point for the console application.
//
#include <signal.h>  

#include "LgEnvironment.h"

//#include "XtionGenerator.h"
#include "MicrosoftKinectV2Generator.h"
#include "SpeedObserver.h"
#include "SynchroObserver.h"
#include "OneDollarRecognizerObserver.h"

bool sortie = false;

void killHandler (int param)
{
	sortie = true;
}

int main(int argc, char* argv[])
{
	int nbUser = 0;

	signal (SIGBREAK, killHandler);
	
	Environment* myEnv = new Environment();

	//cout << myEnv->dataCopyEnabled();
	// Disable data copy --> TODO retirer cet option
	myEnv->enableDataCopy(false);

	MicrosoftKinectV2Generator* gt = new MicrosoftKinectV2Generator("xtionGen"); 
	if(myEnv->registerNode(gt))
		printf("Register Kinect on Kinect OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());

	/*DualHandProcessor* dualHand = new DualHandProcessor("DualHandProcessor");
	dualHand->onlyObservePointType(LG_ORIENTEDPOINT3D_RIGHT_HAND);
	if(myEnv->registerNode(dualHand))
		printf("Register DualHandProcessor OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());*/
	
	//// TODO ajouter observer de vitesse
	SpeedObserver* speedo = new SpeedObserver("SpeedRightObserver");
	speedo->setSpeedThreshold(1.0);
	speedo->onlyObservePointType(LG_ORIENTEDPOINT3D_RIGHT_HAND);
	if(myEnv->registerNode(speedo))
		printf("Register Left Plane Breaker Observer OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());


	//// TODO ajouter Observer de multiple
	SynchroObserver* sync = new SynchroObserver("SynchroObserver");
	//sync->addObserver(speedo,1);
	//sync->addObserver(lspeedo,1);
	if(myEnv->registerNode(sync))
		printf("Register Synchro Observer OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());

	OneDollarRecognizerObserver* odr = new OneDollarRecognizerObserver("OneDollar");
	if(myEnv->registerNode(odr))
		printf("Register Synchro Observer OK.\n");
	else
		printf("%s.\n",myEnv->getLastError().c_str());

	if(myEnv->start())
		printf("Start OK.\n");
	else{
		printf("Start not OK.\n");
		printf("%s.\n",myEnv->getLastError().c_str());
		return 1;
	}


	while(!sortie){

		if(!myEnv->update())
			printf("%s.\n",myEnv->getLastError().c_str());

		//cout << myEnv->getTime() << endl;

		if(myEnv->getGroups3D().size() > 0)
			cout << "Someone is here" << endl;

		int n = myEnv->getGroups3D().size();
		if((nbUser ==0)&&(n>0))
			cout << "New user" << endl;
		if((n ==0)&&(nbUser>0))
			cout << "Lost user" << endl;
		nbUser = n;

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
		if(highest>0.75 )
			cout << "Circle hand " << myEnv->getTime() << endl;

		map<string,float> proba = sync->getProbabilities();
		if(proba["0"]>0.5) cout << "Gesture synchro !" << endl;
		
	}

	myEnv->stop();

	delete myEnv;

	//system("pause");

	return 0;
}

