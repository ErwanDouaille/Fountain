#pragma once

#include "LgObserver.h"

using namespace lg;

#define AT_START 0
#define AT_END 1



class SuccessionObserver :
	public Observer
{
private:
	vector<Observer*> _observers;
	vector<int> _stateForSwitchObserver;
	vector<float> _thresholds;
	vector<int> _timerForWaitingNextState;

	map<string,int> _state;
	map<string,bool> _hasBeenTrue;
	map<string,int> _switchedTime;

public:
	SuccessionObserver(string);
	~SuccessionObserver(void);

	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	void addObserver(Observer* obs,int state, float threshold,int timer);

};

