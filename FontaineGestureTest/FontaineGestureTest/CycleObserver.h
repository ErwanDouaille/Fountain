#pragma once

#include "LgObserver.h"

using namespace lg;

#define AT_START 0
#define AT_END 1



class CycleObserver :
	public Observer
{
private:
	// Observers to cycle
	Observer* _obs;

	// Detect a cycle
	float _threshold;

	// Time to wait the next cycle; if next cycle happens before that time, proba increase, unless, proba decrease
	int _timing; 

	// At_start (when proba go up the threshold) or At end(when proba go down)
	int _state;

	// The increase, decrease applied to current proba
	float _step;



	// Time of last detection
	map<string,int> _lastDetection;

	// For AT end, must have been true
	map<string,bool> _hasBeenTrue;

public:
	CycleObserver(Observer*);
	~CycleObserver(void);

	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	void setThreshold(float thres){_threshold = thres;}
	void setTiming(int timing){_timing = timing;}
	void setSwitchingState(int state){_state = state;}
	void setProbaStep(float step){_step = step;}

};

