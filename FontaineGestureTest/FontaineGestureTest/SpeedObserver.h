#pragma once

#include "LgObserver.h"

using namespace lg;

class SpeedObserver :
	public Observer
{
private:
	float _threshold; // Threshold before coming back to not crossed
	float _downThreshold;

	map<string, bool> _crossed;

public:
	SpeedObserver(string);
	~SpeedObserver(void);

	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	void setSpeedThreshold(float thres){_threshold = thres;}
	void setSpeedDownThreshold(float thres){_downThreshold = thres;}

};

