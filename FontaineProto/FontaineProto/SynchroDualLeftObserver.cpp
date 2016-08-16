#include "SynchroDualLeftObserver.h"

SynchroDualLeftObserver::SynchroDualLeftObserver(void) : Observer("SynchroDualLeftObserver")
{
}

SynchroDualLeftObserver::~SynchroDualLeftObserver(void)
{
}

Node* SynchroDualLeftObserver::clone(string cloneName) const
{
	return new SynchroDualLeftObserver();
}

bool SynchroDualLeftObserver::start()
{
	cout << "Start Observer" << endl;
	return true;
}

bool SynchroDualLeftObserver::stop()
{
	cout << "Stop Observer" << endl;
	return true;
}

void SynchroDualLeftObserver::synchroDualUp(string idFirst, Point3D positionFirst, string idSecond, Point3D positionSecond)
{
	DualHandContainer container(idFirst, positionFirst, idSecond, positionSecond, _timestamp);
	bool newValue = true;
	for(vector<DualHandContainer>::iterator it = synchroDual.begin(); it != synchroDual.end(); it++)
	{
		if(strncmp(container.getIdFirst().c_str(), it->getIdFirst().c_str(), container.getIdFirst().size())== 0)
		{
			int timeDelay = it->getTimestamp() - _timestamp;
			float amplitude = ((positionFirst.getX() + positionSecond.getX())/2);
			float distance = amplitude - (it->getPositionFirst().getX() + it->getPositionSecond().getX())/2;
			float speed = distance / timeDelay;
			
			// SynchroDualRight
			if(distance > 0 )
				return;

			// saving
			it->setPositionFirst(positionFirst);
			it->setPositionSecond(positionSecond);
			
			// old gesture
			if(timeDelay > 100 )
				return;

			// current better hands gesture
			if(getSpeed() > speed)
				return;
			
			updateProbability(to_string(0), 1.0);
			setAmplitude(amplitude);
			setSpeed(speed);
			return;
		}
	}
	if (newValue)
		synchroDual.push_back(container);
}		

bool SynchroDualLeftObserver::observe(map<string,Group3D*> g3D,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>)
{
	//cout << "Update Observer" << endl;
	map<string,float> probas = getProbabilities();
	for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++)
		updateProbability(pit->first,0.0f);
	setSpeed(0.0);
	setAmplitude(0.0);

	for(map<string,Group3D*>::iterator git = g3D.begin();git != g3D.end(); git++)
	{
		if(isObservedGroup(git->first,git->second->getType()))
		{
			Group3D* g = git->second;
			set<HOrientedPoint3D*> rhs = g->getElementsByType(LG_ORIENTEDPOINT3D_RIGHT_HAND);
			for(set<HOrientedPoint3D*> ::iterator sit = rhs.begin();sit != rhs.end();sit++)
			{
				HOrientedPoint3D* rh = *sit;
				Point3D pos = rh->getLast()->getPosition();
				for(map<string,Group3D*>::iterator sgit = g3D.begin();sgit != g3D.end(); sgit++)
				{
					if(sgit->first == git->first)
						continue;
					if(isObservedGroup(sgit->first,sgit->second->getType()))
					{
						Group3D* sg = sgit->second;
						set<HOrientedPoint3D*> rhs = sg->getElementsByType(LG_ORIENTEDPOINT3D_RIGHT_HAND);
						for(set<HOrientedPoint3D*> ::iterator ssit = rhs.begin();ssit != rhs.end();ssit++)
						{
							HOrientedPoint3D* srh = *ssit;
							Point3D spos = srh->getLast()->getPosition();
							if(pos.getY() + thresholdDistance > spos.getY() && pos.getY() - thresholdDistance < spos.getY())
							{
								cout << " same height ! " << endl;
								synchroDualUp(git->first, pos, sgit->first, spos);
							}
						}
					}
				}
			}
		}
	}

	return true;
}

set<string> SynchroDualLeftObserver::need() const
{
	set<string> needed;
	needed.insert(LG_ORIENTEDPOINT3D_RIGHT_HAND);
	return needed;
}
