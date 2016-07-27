#pragma once

#include "LgObserver.h"

using namespace lg;




class SynchroObserver :
	public Observer
{
private:
	vector<Observer*> _observers;
	vector<int> _timeDoneObserver;
	vector<float> _thresholds;

	int _timeToSwitch;

	int _lastProbaOne;

public:
	SynchroObserver(string);
	~SynchroObserver(void);

	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	void addObserver(Observer* obs,int numberOfTimeDone,float threshold);

	void setTimeToWait(int msTimer){_timeToSwitch = msTimer;}

};

