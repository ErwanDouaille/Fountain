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
	Point3D last = it->second.getPosition();	map<int, OrientedPoint3D>::reverse_iterator rit = historicMap.rbegin();

	Point3D first = rit->second.getPosition();
	return Point3D((first.getX() - last.getX())/historicMap.size(), (first.getY() - last.getY())/historicMap.size(), (first.getZ() - last.getZ())/historicMap.size());
}

float angleBetweenHands(HOrientedPoint3D* h, HOrientedPoint3D* rh)
{
	map<int, OrientedPoint3D> historicMapFirst = h->getHistoric(), historicMapSecond = rh->getHistoric();
	map<int, OrientedPoint3D>::iterator it = historicMapFirst.begin(), its = historicMapSecond.begin();
	map<int, OrientedPoint3D>::reverse_iterator rit = historicMapFirst.rbegin(), rits = historicMapSecond.rbegin();
	Point3D last = it->second.getPosition(), lasts = its->second.getPosition();
	Point3D first = rit->second.getPosition(), firsts = rits->second.getPosition();

	float before = atan2(first.getY(), first.getX()) - atan2(firsts.getY(), firsts.getX());
	float after = atan2(last.getY(), last.getX()) - atan2(lasts.getY(), lasts.getX());

	before = before < 0 ? before += 2 * 3.14 : before;
	after = after < 0 ? after += 2 * 3.14 : after;
	cout << "between " << before << "\t " << after << endl; 
	return after - before;
}

float angleMouvement(HOrientedPoint3D* h, HOrientedPoint3D* rh)
{
	map<int, OrientedPoint3D> historicMapFirst = h->getHistoric(), historicMapSecond = rh->getHistoric();
	map<int, OrientedPoint3D>::iterator it = historicMapFirst.begin(), its = historicMapSecond.begin();
	map<int, OrientedPoint3D>::reverse_iterator rit = historicMapFirst.rbegin(), rits = historicMapSecond.rbegin();
	Point3D last = it->second.getPosition(), lasts = its->second.getPosition();
	Point3D first = rit->second.getPosition(), firsts = rits->second.getPosition();

	float before = atan2((first.getY() + firsts.getY()) / 2.0, (first.getX() + firsts.getX()) / 2.0);
	float after = atan2((last.getY() + lasts.getY()) / 2.0, (last.getX() + lasts.getX()) / 2.0);

	/*before = before < 0 ? before += 2 * 3.14 : before;
	after = after < 0 ? after += 2 * 3.14 : after;*/
	cout << "mouvement " << before << "\t " << after << endl; 
	return after - before;
}

bool gotSimilarDistanceFromOrigin(HOrientedPoint3D* rh, HOrientedPoint3D* srh, int threshold)
{
	map<int, OrientedPoint3D> historicMapFirst = rh->getHistoric(), historicMapSecond = srh->getHistoric();
	map<int, OrientedPoint3D>::iterator it = historicMapFirst.begin(), its = historicMapSecond.begin();
	map<int, OrientedPoint3D>::reverse_iterator rit = historicMapFirst.rbegin(), rits = historicMapSecond.rbegin();
	Point3D last = it->second.getPosition(), lasts = its->second.getPosition(), origin(0.0, 0.0, 0.0);
	cout << "DISTANCE " << last.distanceTo(origin) << "\t" << lasts.distanceTo(origin) << endl;
	return last.distanceTo(origin)+ threshold > lasts.distanceTo(origin) && last.distanceTo(origin) - threshold < lasts.distanceTo(origin);	
}

bool gotSimilarHeight(HOrientedPoint3D* rh, HOrientedPoint3D* srh, int threshold)
{
	map<int, OrientedPoint3D> historicMapFirst = rh->getHistoric(), historicMapSecond = srh->getHistoric();
	map<int, OrientedPoint3D>::iterator it = historicMapFirst.begin(), its = historicMapSecond.begin();
	map<int, OrientedPoint3D>::reverse_iterator rit = historicMapFirst.rbegin(), rits = historicMapSecond.rbegin();
	Point3D last = it->second.getPosition(), lasts = its->second.getPosition();
	cout << "HEIGHT " << last.getZ() << "\t" << lasts.getZ() << endl;
	return last.getZ() + threshold > lasts.getZ() && last.getZ() - threshold < lasts.getZ() && gotSimilarDistanceFromOrigin(rh, srh, threshold);	
}

void CmdGlobObserver::recognition(HOrientedPoint3D* rh, HOrientedPoint3D* srh)
{
	Point3D dir = historicDirection(rh);
	Point3D sdir = historicDirection(srh);
	Point3D speed = historicSpeed(rh);
	Point3D sspeed = historicSpeed(srh);
	cout << "\n\nUPDATE RECOGNITION" << endl;
	int threshold = 20;
	float angleDifference = angleBetweenHands(rh, srh);
	float angleMouvementDiff = angleMouvement(rh, srh);
	if( speed.getZ() < -40 && sspeed.getZ() < -40)// && gotSimilarHeight(rh, srh, threshold))
	{
		setCmdName("leve");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
		setDirection(dir);
	}
	else if (speed.getZ() > 40 && sspeed.getZ() > 40)// && gotSimilarHeight(rh, srh, threshold))
	{
		setCmdName("baisse");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
		setDirection(dir);
	}
	/*else if (angleMouvementDiff < -0.7)
	{
		setCmdName("gauche");
		setAmplitude((abs(dir.getY()) + abs(sdir.getY()))/2.0);
		setSpeed((abs(speed.getY()) + abs(sspeed.getY()))/2.0);
		setDirection(dir);
	}
	else if (angleMouvementDiff > 0.7)
	{
		setCmdName("droite");
		setAmplitude((abs(dir.getY()) + abs(sdir.getY()))/2.0);
		setSpeed((abs(speed.getY()) + abs(sspeed.getY()))/2.0);
		setDirection(dir);
	}*/
	else if (angleDifference > 0.7 && gotSimilarDistanceFromOrigin(rh, srh, threshold))
	{
		setCmdName("Reserre");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
	}
	else if (angleDifference < -0.7 && gotSimilarDistanceFromOrigin(rh, srh, threshold))
	{
		setCmdName("Ecarte");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
	}
}
bool notSameLevel(HOrientedPoint3D* rh, HOrientedPoint3D* srh)
{
	int thresholdDistance = 10;
	Point3D first = rh->getLast()->getPosition(), second = srh->getLast()->getPosition();
	if ((first.getX() + thresholdDistance > second.getX() && first.getX() - thresholdDistance < second.getX()) ||
		(first.getY() + thresholdDistance > second.getY() && first.getY() - thresholdDistance < second.getY()) ||
		(first.getZ() + thresholdDistance > second.getZ() && first.getZ() - thresholdDistance < second.getZ())
		)
		return false;
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
				if(rh->getHistoric().size() < 3 || rh->getLastTimestamp() != _timestamp)
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
							if(srh->getHistoric().size() < 3 || srh->getLastTimestamp() != _timestamp || notSameLevel(rh, srh))
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