#pragma once

#include "LgObserver.h"

using namespace lg;




class Delayer :
	public Observer
{
private:
	Observer* _obs;
	int _timing;
	map<string,int> _lastOnes;
	float _threshold;

public:
	Delayer(Observer*);
	~Delayer(void);

	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	void setThreshold(int thres){_threshold = thres;}
	void setTiming(int timing){_timing = timing;}

};

