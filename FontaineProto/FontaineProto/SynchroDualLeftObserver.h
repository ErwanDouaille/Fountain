#pragma once

#include "LgObserver.h"
#include "DualHandContainer.h"

#define thresholdDistance 20

using namespace lg;

class SynchroDualLeftObserver :
	public Observer
{
private:
	int hauteurCamera;
	float blasterWidth;
	vector<float> xBlaster;
	vector<float> yBlaster;
	vector<DualHandContainer> synchroDual;

	float _speed, _amplitude;

	map<string,int> _jets;
	map<string,float> _hauteurs;

	void synchroDualUp(string idFirst, Point3D positionFirst, string idSecond, Point3D positionSecond);

public:
	SynchroDualLeftObserver(void);
	~SynchroDualLeftObserver(void);
	
	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 
	float getSpeed() {return _speed;}
	float getAmplitude() {return _amplitude;} 
	void setAmplitude(float value) { _amplitude = value;}
	void setSpeed(float value) { _speed = value;}
};

