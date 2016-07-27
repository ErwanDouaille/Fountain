#pragma once

#include "LgObserver.h"

using namespace lg;

class CircularSwype :
	public Observer
{
private:
	float _threshold; 

public:
	CircularSwype(string);
	~CircularSwype(void);

	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	void setSpeedThreshold(float thres){_threshold = thres;}

};

