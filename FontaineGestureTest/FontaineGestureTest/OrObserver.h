#pragma once

#include "LgObserver.h"

using namespace lg;




class OrObserver :
	public Observer
{
private:
	vector<Observer*> _observers;

public:
	OrObserver(string);
	~OrObserver(void);

	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	void addObserver(Observer* obs);

};

