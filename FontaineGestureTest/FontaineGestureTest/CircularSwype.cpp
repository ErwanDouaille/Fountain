#include "CircularSwype.h"

CircularSwype::CircularSwype(string name) : Observer(name)
{
	_threshold = 2.0f;

}


CircularSwype::~CircularSwype(void)
{
}


Node* CircularSwype::clone(string) const
{
	return new CircularSwype("CircularSwype");
}

bool CircularSwype::start()
{
	//cout << "Start Observer" << endl;
	return true;
}

bool CircularSwype::stop()
{
	//cout << "Stop Observer" << endl;
	return true;
}

bool CircularSwype::observe(map<string,Group3D*> g3D,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>)
{
	//cout << "Update Observer" << endl;
	
	for(map<string,Group3D*>::iterator mit = g3D.begin();mit != g3D.end();mit++){
		//updateProbability(mit->first,0.0);
		//cout << " 3D Groups " << mit->first.c_str() << endl;
		if(isObservedGroup(mit->first,mit->second->getType())){
			map<string,HOrientedPoint3D*> elements = mit->second->getAll();
			for(map<string,HOrientedPoint3D*>::iterator eit = elements.begin();eit != elements.end();eit++){
				//cout << "1" << endl;
				if(isObservedType(eit->second->getType())){
					//cout << "O" << endl;
					HOrientedPoint3D* thp = eit->second;
					map<int,OrientedPoint3D> hist = thp->getHistoric();
					if(hist.size()>1){
						map<int,OrientedPoint3D>::reverse_iterator endIt = hist.rbegin();
						OrientedPoint3D last = endIt->second;
						int lastTime = endIt->first;
						endIt++;
						OrientedPoint3D previous = endIt->second;
						int previousTime = endIt->first;

						// TODO reposition previous at x = 0, and reposition current with the same rotation

						
					}
					else 
					{
						//_crossed[mit->first] = false;;
						updateProbability(mit->first,0.0);
					}
				}
			}
		}
	}

	return true;
}

set<string> CircularSwype::need() const
{
	return _observedPointType;
}