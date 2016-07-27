#include "SuccessionObserver.h"


SuccessionObserver::SuccessionObserver(string name) : Observer(name)
{
}


SuccessionObserver::~SuccessionObserver(void)
{
}


Node* SuccessionObserver::clone(string) const
{
	return new SuccessionObserver("SuccessionObserver");
}

bool SuccessionObserver::start()
{
	//cout << "Start Observer" << endl;
	bool res = true;

	for(vector<Observer*>::iterator oit = _observers.begin();oit != _observers.end();oit++){
		res = res && (*oit)->start();
	}

	return res;
}

bool SuccessionObserver::stop()
{
	bool res = true;

	for(vector<Observer*>::iterator oit = _observers.begin();oit != _observers.end();oit++){
		res = res && (*oit)->stop();
	}
	//cout << "Stop Observer" << endl;
	return res;
}

bool SuccessionObserver::observe(map<string,Group3D*> g3D,map<string,Group2D*> g2D,map<string,Group1D*> g1D, map<string,GroupSwitch*> gs)
{

	// update all observers
	bool res = true;

	for(vector<Observer*>::iterator oit = _observers.begin();oit != _observers.end();oit++)
		res = res && (*oit)->observe(g3D,g2D,g1D,gs);

	// TODO parcours all groups to watch and check their state

	for(map<string,Group3D*>::iterator mit = g3D.begin();mit != g3D.end();mit++){
		if(isObservedGroup(mit->first,mit->second->getType())){
			int state = _state[mit->first];
			int hasBeenTrue = _hasBeenTrue[mit->first];

			Observer* curObs = _observers[state];
			int curStateToWait = _stateForSwitchObserver[state];
			float curThres = _thresholds[state];
			int timing = _timerForWaitingNextState[state];

			float proba = curObs->getProbability(mit->first);
			
			if(proba >= curThres)
			{
				if(curStateToWait == AT_START){
					_state[mit->first]++;
					_hasBeenTrue[mit->first] = false;
					_switchedTime[mit->first] = _timestamp;
					if(_state[mit->first] >= _observers.size()){
						_state[mit->first] = 0;
						updateProbability(mit->first,1.0);
					}
					else
						updateProbability(mit->first,(1.0+_state[mit->first])/_observers.size());
				}
				else
				{
					_hasBeenTrue[mit->first] = true;
					updateProbability(mit->first,(1.0+_state[mit->first])/_observers.size());
				}
			}
			else{
				if((state != 0)&&(_timestamp - _switchedTime[mit->first] > timing)){
					// Go back to state 0;
					_state[mit->first] = 0;
					_hasBeenTrue[mit->first] = false;
					updateProbability(mit->first,0.0);
				}
				else
				{
					if((curStateToWait == AT_END)&&(hasBeenTrue)){
						_state[mit->first]++;
						_hasBeenTrue[mit->first] = false;
						_switchedTime[mit->first] = _timestamp;
						if(_state[mit->first] >= _observers.size()){
							_state[mit->first] = 0;
							updateProbability(mit->first,1.0);
						}
						else
							updateProbability(mit->first,(1.0+_state[mit->first])/_observers.size());
					}
					else
					{
						updateProbability(mit->first,0.0);
					}
				}
			}

		}
		else
			updateProbability(mit->first,0.0);
	}





	/*if(_observers.size()>0){
		Observer* curObs = _observers[_state];
		int curStateToWait = _stateForSwitchObserver[_state];
		float curThres = _thresholds[_state];

		if(!curObs->observe(g3D,g2D,g1D,gs)) return false;

		float highestProba = 0.0;

		//  update all observers then parcours probas and check for each group what needed


		if(highestProba > curThres)
		{
			if(curStateToWait == AT_START){
				_state++;
				_hasBeenTrue = false;
			}
			else
			{
				_hasBeenTrue = true;
				_lastProbaOne = _timestamp;
			}
			updateProbability("0",(1+_state)/_observers.size());
		}
		else{
			if(_hasBeenTrue){
				_state++;
				_hasBeenTrue = false;
				updateProbability("0",(1+_state)/_observers.size());
			}
			else
			{
				updateProbability("0",0.0);
			}
		}
	}*/

	return true;
}

set<string> SuccessionObserver::need() const
{
	return _observedPointType;
}

void SuccessionObserver::addObserver(Observer* obs,int state, float threshold,int timer)
{
	_observers.push_back(obs);
	obs->setEnvironment(_environment);
	obs->setID(0);
	_stateForSwitchObserver.push_back(state);
	_thresholds.push_back(threshold);
	_timerForWaitingNextState.push_back(timer);
}