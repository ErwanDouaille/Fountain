#include "CmdGlobObserver.h"

CmdGlobObserver::CmdGlobObserver(void) : Observer("CmdGlobObserver")
{
}

CmdGlobObserver::~CmdGlobObserver(void)
{
}

Node* CmdGlobObserver::clone(string cloneName) const
{
	return new CmdGlobObserver();
}

set<string> CmdGlobObserver::need() const
{
	set<string> needed;
	needed.insert(LG_ORIENTEDPOINT3D_RIGHT_HAND);
	return needed;
}

bool CmdGlobObserver::start()
{
	cout << "Start Observer" << endl;
	return true;
}

bool CmdGlobObserver::stop()
{
	cout << "Stop Observer" << endl;
	return true;
}

Point3D historicDirection(HOrientedPoint3D* h)
{
	map<int, OrientedPoint3D> historicMap = h->getHistoric();
	map<int, OrientedPoint3D>::iterator it = historicMap.begin();
	map<int, OrientedPoint3D>::reverse_iterator rit = historicMap.rbegin();
	Point3D last = it->second.getPosition();
	Point3D first = rit->second.getPosition();
	return Point3D(first.getX() - last.getX(), first.getY() - last.getY(), first.getZ() - last.getZ());
}

Point3D historicSpeed(HOrientedPoint3D* h)
{
	map<int, OrientedPoint3D> historicMap = h->getHistoric();
	map<int, OrientedPoint3D>::iterator it = historicMap.begin();
	map<int, OrientedPoint3D>::reverse_iterator rit = historicMap.rbegin();
	Point3D last = it->second.getPosition();
	Point3D first = rit->second.getPosition();
	return Point3D((first.getX() - last.getX())/historicMap.size(), (first.getY() - last.getY())/historicMap.size(), (first.getZ() - last.getZ())/historicMap.size());
}

void CmdGlobObserver::recognition(HOrientedPoint3D* rh, HOrientedPoint3D* srh)
{
	Point3D dir = historicDirection(rh);
	Point3D sdir = historicDirection(srh);
	Point3D speed = historicSpeed(rh);
	Point3D sspeed = historicSpeed(srh);
	if( speed.getZ() < -30 && sspeed.getZ() < -30)
	{
		setCmdName("leve");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
		setDirection(dir);
	}
	else if (speed.getZ() > 30 && sspeed.getZ() > 30)
	{
		setCmdName("baisse");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
		setDirection(dir);
	}
	else if (speed.getY() < -30 && sspeed.getY() < -30)
	{
		setCmdName("gauche");
		setAmplitude((abs(dir.getY()) + abs(sdir.getY()))/2.0);
		setSpeed((abs(speed.getY()) + abs(sspeed.getY()))/2.0);
		setDirection(dir);
	}
	else if (speed.getY() > 30 && sspeed.getY() > 30)
	{
		setCmdName("droite");
		setAmplitude((abs(dir.getY()) + abs(sdir.getY()))/2.0);
		setSpeed((abs(speed.getY()) + abs(sspeed.getY()))/2.0);
		setDirection(dir);
	}
	else if (speed.getY() > 30 && sspeed.getY() < -30)
	{
		setCmdName("Ecarte");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
	}
	else if (speed.getY() < -30 && sspeed.getY() > 30)
	{
		setCmdName("Reserre");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
	}
}

bool CmdGlobObserver::observe(map<string,Group3D*> g3D,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>)
{
	map<string,float> probas = getProbabilities();		
	for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++)		
		updateProbability(pit->first,0.0f);		
	setSpeed(0);
	setAmplitude(0);
	setCmdName("");

	for(map<string,Group3D*>::iterator git = g3D.begin();git != g3D.end(); git++)		
	{		
		if(isObservedGroup(git->first,git->second->getType()))		
		{		
			Group3D* g = git->second;		
			set<HOrientedPoint3D*> rhs = g->getElementsByType(LG_ORIENTEDPOINT3D_RIGHT_HAND);		
			for(set<HOrientedPoint3D*> ::iterator sit = rhs.begin();sit != rhs.end();sit++)		
			{		
				HOrientedPoint3D* rh = *sit;		
				if(rh->getHistoric().size() < 3)
					continue;
				for(map<string,Group3D*>::iterator sgit = git; sgit != g3D.end(); sgit++)		
				{		
					if(sgit->first.compare(git->first) == 0)		
						continue;		
					if(isObservedGroup(sgit->first,sgit->second->getType()))		
					{		
						Group3D* sg = sgit->second;		
						set<HOrientedPoint3D*> rhs = sg->getElementsByType(LG_ORIENTEDPOINT3D_RIGHT_HAND);		
						for(set<HOrientedPoint3D*> ::iterator ssit = rhs.begin();ssit != rhs.end();ssit++)		
						{		
							HOrientedPoint3D* srh = *ssit;		
							if(srh->getHistoric().size() < 10)
								continue;
							recognition(rh, srh);

						}		
					}		
				}		
			}		
		}		
	}		
		
	return true;
}