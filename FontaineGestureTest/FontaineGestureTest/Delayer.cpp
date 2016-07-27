#include "Delayer.h"


Delayer::Delayer(Observer* obs) : Observer("Delayer")
{
	_obs = obs;
	_timing = 100;
	_threshold = 0.8; 
}


Delayer::~Delayer(void)
{
}


Node* Delayer::clone(string) const
{
	return new Delayer(_obs);
}

bool Delayer::start()
{
	//cout << "Start Observer" << endl;

	return _obs->start();
}

bool Delayer::stop()
{
	
	//cout << "Stop Observer" << endl;
	return _obs->stop();
}

bool Delayer::observe(map<string,Group3D*> g3D,map<string,Group2D*> g2D,map<string,Group1D*> g1D, map<string,GroupSwitch*> gs)
{
	_obs->observe(g3D,g2D,g1D,gs);

	map<string,float> probas = _obs->getProbabilities();

	for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++){
		if(pit->second >= _threshold){
			updateProbability(pit->first,pit->second);
			_lastOnes[pit->first] = _timestamp;
		}
		else
		{
			if(_lastOnes[pit->first] - _timestamp  > _timing)
				updateProbability(pit->first,_threshold);
			else
				updateProbability(pit->first,pit->second);
		}
	}

	return true;
}

set<string> Delayer::need() const
{
	return _obs->need();
}
