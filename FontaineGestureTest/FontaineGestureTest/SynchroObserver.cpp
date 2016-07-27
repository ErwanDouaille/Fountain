#include "SynchroObserver.h"


SynchroObserver::SynchroObserver(string name) : Observer(name)
{
	_timeToSwitch = 70;

	_lastProbaOne = -1000;
}


SynchroObserver::~SynchroObserver(void)
{
}


Node* SynchroObserver::clone(string) const
{
	return new SynchroObserver("SynchroObserver");
}

bool SynchroObserver::start()
{
	//cout << "Start Observer" << endl;
	bool res = true;

	for(vector<Observer*>::iterator oit = _observers.begin();oit != _observers.end();oit++){
		res = res && (*oit)->start();
	}

	return res;
}

bool SynchroObserver::stop()
{
	bool res = true;

	for(vector<Observer*>::iterator oit = _observers.begin();oit != _observers.end();oit++){
		res = res && (*oit)->stop();
	}
	//cout << "Stop Observer" << endl;
	return true;
}

bool SynchroObserver::observe(map<string,Group3D*> g3D,map<string,Group2D*> g2D,map<string,Group1D*> g1D, map<string,GroupSwitch*> gs)
{

	bool res = false;
	vector<int>::iterator nit = _timeDoneObserver.begin();
	vector<float>::iterator thit = _thresholds.begin();
	for(vector<Observer*>::iterator oit = _observers.begin();oit != _observers.end();oit++){
		int count = 0;
		(*oit)->observe(g3D,g2D,g1D,gs);

		map<string,float> probas = (*oit)->getProbabilities();
		for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++){
			if(pit->second > *thit) count++;
		}

		res = res || (count >= *nit);

		nit++;
		thit++;
	}

	if(res){
		updateProbability("0",1.0);
		_lastProbaOne = _timestamp;
	}
	else{
		if(_timestamp - _lastProbaOne > _timeToSwitch)
			updateProbability("0",0.0);
		else
			updateProbability("0",1.0);
	}

	return true;
}

set<string> SynchroObserver::need() const
{
	return _observedPointType;
}

void SynchroObserver::addObserver(Observer* obs,int nb,float threshold)
{
	_observers.push_back(obs);
	obs->setEnvironment(_environment);
	obs->setID(0);
	_timeDoneObserver.push_back(nb);
	_thresholds.push_back(threshold);
}