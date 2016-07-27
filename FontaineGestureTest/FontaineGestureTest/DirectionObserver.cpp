/*
 * DirectionObserver.cpp
 *
 *  Created on: 4 avr. 2011
 *      Author: nicolas
 */

#include "DirectionObserver.h"


DirectionObserver::DirectionObserver() : Observer("DirectionObserver") {

	//printf("Received int : %i %i %i\n",moveToX,moveToY,moveToZ);

	_moveToX = 0;
	_moveToY = 0;
	_moveToZ = 0;

	_typeX = DIRECTION_TYPE_IGNORE;
	_typeY = DIRECTION_TYPE_IGNORE;
	_typeZ = DIRECTION_TYPE_IGNORE;
}


DirectionObserver::~DirectionObserver() {
	// TODO Auto-generated destructor stub
}


Node* DirectionObserver::clone(string) const
{
	return new DirectionObserver();
}

bool DirectionObserver::start()
{
	//cout << "Start Observer" << endl;

	return true;
}

bool DirectionObserver::stop()
{
	//cout << "Stop Observer" << endl;
	return true;
}

bool DirectionObserver::observe(map<string,Group3D*> g3D,map<string,Group2D*> g2D,map<string,Group1D*> g1D, map<string,GroupSwitch*> gs)
{

	for(map<string,Group3D*>::iterator mit = g3D.begin();mit != g3D.end();mit++){
		//updateProbability(mit->first,0.0);
		//cout << " 3D Groups " << mit->first.c_str() << endl;
		if(isObservedGroup(mit->first,mit->second->getType())){
			map<string,HOrientedPoint3D*> elements = mit->second->getAll();
			for(map<string,HOrientedPoint3D*>::iterator eit = elements.begin();eit != elements.end();eit++){
				//cout << "1" << endl;
				if(isObservedType(eit->second->getType())){

					HOrientedPoint3D* thp = eit->second;
					map<int,OrientedPoint3D> hist = thp->getHistoric();
					if(hist.size()>1){
						map<int,OrientedPoint3D>::reverse_iterator endIt = hist.rbegin();
						OrientedPoint3D last = endIt->second;
						int lastTime = endIt->first;
						endIt++;
						OrientedPoint3D previous = endIt->second;
						int previousTime = endIt->first;

						float goodDir = 0.0;
						int cpt = 0;
						if(_typeX != DIRECTION_TYPE_IGNORE){
							if(_typeX == DIRECTION_TYPE_MINIMUM)
							{
								if(last.getPosition().getX()-previous.getPosition().getX() <= _moveToX)
									goodDir+=1;
							}
							else
							{
								goodDir += goToDirection(_moveToX,last.getPosition().getX()-previous.getPosition().getX());
							}
							cpt++;
						}

						if(_typeY != DIRECTION_TYPE_IGNORE){
							if(_typeY == DIRECTION_TYPE_MINIMUM)
							{
								if(last.getPosition().getY()-previous.getPosition().getY() <= _moveToY)
									goodDir+=1;
							}
							else
							{
								goodDir += goToDirection(_moveToY,last.getPosition().getY()-previous.getPosition().getY());
							}
							cpt++;
						}

						if(_typeZ != DIRECTION_TYPE_IGNORE){
							if(_typeZ == DIRECTION_TYPE_MINIMUM)
							{
								if(last.getPosition().getZ()-previous.getPosition().getZ() <= _moveToZ)
									goodDir+=1;
							}
							else
							{
								goodDir += goToDirection(_moveToZ,last.getPosition().getZ()-previous.getPosition().getZ());
							}
							cpt++;
						}

						goodDir = goodDir / (float)cpt;

						if(goodDir<0.0f) goodDir = 0.0f;
						if(goodDir>1.0f) goodDir = 1.0f;

						if(cpt!=0)
							updateProbability(mit->first,goodDir);
						else
							updateProbability(mit->first,0.0);

					}
					else 
					{
						updateProbability(mit->first,0.0);
					}

				}
			}
		}
	}

	return true;
}

set<string> DirectionObserver::need() const
{
	return _observedPointType;
}

float DirectionObserver::goToDirection(int direction,float difference){
	float step = difference/(float)direction;
	if(step<-1.0f) step = -1.0f;
	if(step>1.0f) step = 1.0f;
	return step;
}

void DirectionObserver::observeAxis(int axis,int type, int value){
	if(axis == DIRECTION_AXIS_X){
		_typeX = type;
		_moveToX = value;
	}
	if(axis == DIRECTION_AXIS_Y){
		_typeY = type;
		_moveToY = value;
	}
	if(axis == DIRECTION_AXIS_Z){
		_typeZ = type;
		_moveToZ = value;
	}
}