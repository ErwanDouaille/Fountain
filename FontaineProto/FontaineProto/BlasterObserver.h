#pragma once

#define BLASTERS_THRESHOLDS_DISTANCE 0

#include "LgObserver.h"

using namespace lg;

class BlasterObserver :
	public Observer
{
private:
	int hauteurCamera;
	int fountainHeight;
	float blasterWidth;
	vector<float> xBlaster;
	vector<float> yBlaster;

	map<string,int> _jets;
	map<string,float> _hauteurs;

public:
	BlasterObserver(void);
	~BlasterObserver(void);
	
	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	float getHauteur(string);
	int getJet(string);

	void setFountainHeight(int value) { fountainHeight = value;}
};

