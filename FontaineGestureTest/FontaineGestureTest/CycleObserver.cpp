#include "CycleObserver.h"

CycleObserver::CycleObserver(Observer* obs) : Observer("CycleObserver")
{
}


CycleObserver::~CycleObserver(void)
{
}


Node* CycleObserver::clone(string) const
{
	return new CycleObserver(_obs);
}

bool CycleObserver::start()
{
	//cout << "Start Observer" << endl;

	return _obs->start();
}

bool CycleObserver::stop()
{
	//cout << "Stop Observer" << endl;
	return _obs->stop();
}

bool CycleObserver::observe(map<string,Group3D*> g3D,map<string,Group2D*> g2D,map<string,Group1D*> g1D, map<string,GroupSwitch*> gs)
{

	// update all observers

	_obs->observe(g3D,g2D,g1D,gs);


	for(map<string,Group3D*>::iterator mit = g3D.begin();mit != g3D.end();mit++){
		if(isObservedGroup(mit->first,mit->second->getType())){
			int lastTime = _lastDetection[mit->first];
			int hasBeenTrue = _hasBeenTrue[mit->first];

			float proba = _obs->getProbability(mit->first);
			
			if(proba >= _threshold)
			{
				if(_state == AT_START){
					_hasBeenTrue[mit->first] = false;
					_lastDetection[mit->first] = _timestamp;
					updateProbability(mit->first,getProbability(mit->first)+_step); // add step to current proba

				}
				else
				{
					_hasBeenTrue[mit->first] = true;
					updateProbability(mit->first,getProbability(mit->first));
				}
			}
			else{
				if(_timestamp - _lastDetection[mit->first] > _timing){
					_hasBeenTrue[mit->first] = false;
					updateProbability(mit->first,0.0);
				}
				else
				{
					if((_state == AT_END)&&(hasBeenTrue)){
						_hasBeenTrue[mit->first] = false;
						_lastDetection[mit->first] = _timestamp;
						updateProbability(mit->first,getProbability(mit->first)+_step); // add step to current proba
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


	return true;
}

set<string> CycleObserver::need() const
{
	return _observedPointType;
}
