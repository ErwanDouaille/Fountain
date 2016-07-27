#include "AndObserver.h"


AndObserver::AndObserver(string name) : Observer(name)
{
}


AndObserver::~AndObserver(void)
{
}


Node* AndObserver::clone(string) const
{
	return new AndObserver("AndObserver");
}

bool AndObserver::start()
{
	//cout << "Start Observer" << endl;
	bool res = true;

	for(vector<Observer*>::iterator oit = _observers.begin();oit != _observers.end();oit++){
		res = res && (*oit)->start();
	}

	return res;
}

bool AndObserver::stop()
{
	bool res = true;

	for(vector<Observer*>::iterator oit = _observers.begin();oit != _observers.end();oit++){
		res = res && (*oit)->stop();
	}
	//cout << "Stop Observer" << endl;
	return true;
}

bool AndObserver::observe(map<string,Group3D*> g3D,map<string,Group2D*> g2D,map<string,Group1D*> g1D, map<string,GroupSwitch*> gs)
{

	bool res = true;
	if(_observers.size()>1){
		vector<Observer*>::iterator oit = _observers.begin();
		res = res && (*oit)->observe(g3D,g2D,g1D,gs);
		map<string,float> probas = (*oit)->getProbabilities();
		oit++;
		for(;oit != _observers.end();oit++){
			res = res && (*oit)->observe(g3D,g2D,g1D,gs);

			map<string,float> probas2 = (*oit)->getProbabilities();
			for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++){
				probas[pit->first] += probas2[pit->first];
			}
		}

		for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++){
			updateProbability(pit->first,probas[pit->first] / _observers.size());
		}
	}

	return res;
}

set<string> AndObserver::need() const
{
	set<string> res;
	for(vector<Observer*>::const_iterator oit = _observers.begin();oit != _observers.end();oit++){
		set<string> ores = (*oit)->need();
		res.insert(ores.begin(),ores.end());
	}
	return res;
}

void AndObserver::addObserver(Observer* obs)
{
	_observers.push_back(obs);
}