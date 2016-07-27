#pragma once

#include "LgProcessor.h"

#include"filtering.h"
#include "vecteur3d.h"

using namespace lg;

class FilteringProcessor :
	public Processor
{
private:
	string _generatedGroupType;

	LowPassFilter<Vecteur3D> _filter;

	float _cutOff;
public:
	FilteringProcessor(string);
	~FilteringProcessor(void);

	Node* clone(string) const;

	bool start();
	bool stop();

	bool update(map<string,Group3D*>& groups3D, map<string,Group2D*>& groups2D, map<string,Group1D*>& groups1D, map<string,GroupSwitch*>& groupsSwitch);

	set<string> need() const; 
	set<string> consume() const; 
	set<string> produce() const; 

	string getGeneratedGroupType(){return _generatedGroupType;}
	void setGeneratedGroupType(string newType){_generatedGroupType = newType;}

	float getCutOffFrequency(){return _cutOff;}
	void setCutOffFrequency(float newCutOff){_cutOff = newCutOff;}
};

