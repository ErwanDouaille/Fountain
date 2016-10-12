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
	controlPosIteration = 0;
	controlPosLastTimestamp = _timestamp;
	return true;
}

bool CmdGlobObserver::stop()
{
	cout << "Stop Observer" << endl;
	return true;
}

Point3D getLastPosition(HOrientedPoint3D* rh)
{
	map<int, OrientedPoint3D> historicMapFirst = rh->getHistoric();
	map<int, OrientedPoint3D>::reverse_iterator rit = historicMapFirst.rbegin();
	return rit->second.getPosition();
}

Point3D getFirstPosition(HOrientedPoint3D* rh)
{
	map<int, OrientedPoint3D> historicMapFirst = rh->getHistoric();
	map<int, OrientedPoint3D>::iterator it = historicMapFirst.begin();
	return it->second.getPosition();
}

Point3D historicDirection(HOrientedPoint3D* h)
{
	Point3D last = getLastPosition(h);
	Point3D first = getFirstPosition(h);
	return Point3D(first.getX() - last.getX(), first.getY() - last.getY(), first.getZ() - last.getZ());
}

Point3D historicSpeed(HOrientedPoint3D* h)
{
	Point3D last = getLastPosition(h);
	Point3D first = getFirstPosition(h);
	return Point3D((first.getX() - last.getX())/h->getHistoric().size(), 
		(first.getY() - last.getY())/h->getHistoric().size(), 
		(first.getZ() - last.getZ())/h->getHistoric().size());
}

float historicDistance(HOrientedPoint3D* h)
{
	Point3D last = getLastPosition(h);
	Point3D first = getFirstPosition(h);
	return first.distanceTo(last);
}

float angleBetweenHands(HOrientedPoint3D* h, HOrientedPoint3D* rh)
{
	Point3D last = getLastPosition(h);
	Point3D first = getFirstPosition(h);
	Point3D lasts = getLastPosition(rh);
	Point3D firsts = getFirstPosition(rh);

	float dot = (last.getX() * lasts.getX()) + (last.getY() * lasts.getY()) + (last.getZ() * lasts.getZ());
	float len1 = sqrtf((last.getX() * last.getX()) + (last.getY() * last.getY()) + (last.getZ() * last.getZ()));
	float len2 = sqrtf((lasts.getX() * lasts.getX()) + (lasts.getY() * lasts.getY()) + (lasts.getZ() * lasts.getZ()));
	float theta1 = acosf(dot / (len1 * len2));

	dot = (first.getX() * firsts.getX()) + (first.getY() * firsts.getY()) + (first.getZ() * firsts.getZ());
	len1 = sqrtf((first.getX() * first.getX()) + (first.getY() * first.getY()) + (first.getZ() * first.getZ()));
	len2 = sqrtf((firsts.getX() * firsts.getX()) + (firsts.getY() * firsts.getY()) + (firsts.getZ() * firsts.getZ()));
	float theta2 = acosf(dot / (len1 * len2));

	return theta2 - theta1;
	/*
	float before = atan2(first.getY(), first.getX()) - atan2(firsts.getY(), firsts.getX());
	float after = atan2(last.getY(), last.getX()) - atan2(lasts.getY(), lasts.getX());
	return after - before;
	*/


}

float angleMouvement(HOrientedPoint3D* h, HOrientedPoint3D* rh)
{
	Point3D last = getLastPosition(h);
	Point3D first = getFirstPosition(h);
	Point3D lasts = getLastPosition(rh);
	Point3D firsts = getFirstPosition(rh);
	
	last = Point3D( (last.getX() + lasts.getX())/2.0, (last.getY() + lasts.getY())/2.0, (last.getZ() + lasts.getZ())/2.0 );
	lasts = Point3D( (first.getX() + firsts.getX())/2.0, (first.getY() + firsts.getY())/2.0, (first.getZ() + firsts.getZ())/2.0 );

	float dot = (last.getX() * lasts.getX()) + (last.getY() * lasts.getY()) + (last.getZ() * lasts.getZ());
	float len1 = sqrtf((last.getX() * last.getX()) + (last.getY() * last.getY()) + (last.getZ() * last.getZ()));
	float len2 = sqrtf((lasts.getX() * lasts.getX()) + (lasts.getY() * lasts.getY()) + (lasts.getZ() * lasts.getZ()));
	float theta1 = acosf(dot / (len1 * len2));
	return theta1;
	/*Point3D last = getLastPosition(h);
	Point3D first = getFirstPosition(h);
	Point3D lasts = getLastPosition(rh);
	Point3D firsts = getFirstPosition(rh);

	float before = atan2((first.getY() + firsts.getY()) / 2.0, (first.getX() + firsts.getX()) / 2.0);
	float after = atan2((last.getY() + lasts.getY()) / 2.0, (last.getX() + lasts.getX()) / 2.0);
	return after - before;*/
}

bool gotSimilarDistanceFromOrigin(HOrientedPoint3D* rh, HOrientedPoint3D* srh, int threshold)
{
	Point3D last = getLastPosition(rh);
	Point3D lasts = getLastPosition(srh);
	Point3D origin(0.0, 0.0, 0.0);
	return last.distanceTo(origin)+ threshold > lasts.distanceTo(origin) && last.distanceTo(origin) - threshold < lasts.distanceTo(origin);	
}

bool gotSimilarHeight(HOrientedPoint3D* rh, HOrientedPoint3D* srh, int threshold)
{
	Point3D last = getLastPosition(rh);
	Point3D lasts = getLastPosition(srh);
	return last.getZ() + threshold > lasts.getZ() && last.getZ() - threshold < lasts.getZ();// && gotSimilarDistanceFromOrigin(rh, srh, threshold);	
}

float getAngleFromHandsToCenter(HOrientedPoint3D* rh, HOrientedPoint3D* srh)
{
	Point3D last = getLastPosition(rh);
	Point3D lasts = getLastPosition(srh);
	Point3D p1(0.0, 0.0, 0.0);
	Point3D p2((last.getX() + lasts.getX())/2.0, (last.getY() + lasts.getY())/2.0, 0.0);

	float x = atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX());
	return (x > 0 ? x : (2* 3.14159 + x)) * 360 / (2* 3.14159);
}

float getAngleFromOneHandToCenter(HOrientedPoint3D* rh)
{
	Point3D last = getLastPosition(rh);
	Point3D p1(0.0, 0.0, 0.0);
	Point3D p2(last.getX(), last.getY(), 0.0);
	return abs(atan2(p1.getY() - p2.getY(), p1.getX() - p2.getX()) * 180 /  3.14159);
}

bool CmdGlobObserver::recognition(HOrientedPoint3D* rh, HOrientedPoint3D* srh)
{
	Point3D dir = historicDirection(rh);
	Point3D sdir = historicDirection(srh);
	float distance = historicDistance(rh);
	float sdistance = historicDistance(srh);
	Point3D speed = historicSpeed(rh);
	Point3D sspeed = historicSpeed(srh);
	int thresholdControlPosition = 40;
	float angleDifference = angleBetweenHands(rh, srh);
	float angleMouvementDiff = angleMouvement(rh, srh);
	
	if( speed.getZ() < -30 && sspeed.getZ() < -30)
	{
		setCmdName("baisse");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
		setDirection(dir);
		controlPosIteration = 0;
	}
	else if (speed.getZ() > 30 && sspeed.getZ() > 30)
	{
		setCmdName("leve");
		setAmplitude((abs(dir.getZ()) + abs(sdir.getZ()))/2.0);
		setSpeed((abs(speed.getZ()) + abs(sspeed.getZ()))/2.0);
		setDirection(dir);
		controlPosIteration = 0;
	}
	/*else if (angleDifference < -0.1 &&
		dir.getX() + 50 > sdir.getX() && dir.getX() - 50 < sdir.getX() &&
		dir.getY() + 50 > sdir.getY() && dir.getY() - 50 < sdir.getY())
	{
		setCmdName("droite");
		setAmplitude((abs(dir.getY()) + abs(sdir.getY()))/2.0);
		setSpeed((abs(speed.getY()) + abs(sspeed.getY()))/2.0);
		setDirection(dir);
		controlPosIteration = 0;
	}
	else if  (angleDifference > 0.1 &&
		dir.getX() + 50 > sdir.getX() && dir.getX() - 50 < sdir.getX() &&
		dir.getY() + 50 > sdir.getY() && dir.getY() - 50 < sdir.getY() )
	{
		setCmdName("gauche");
		setAmplitude((abs(dir.getY()) + abs(sdir.getY()))/2.0);
		setSpeed((abs(speed.getY()) + abs(sspeed.getY()))/2.0);
		setDirection(dir);
		controlPosIteration = 0;
	}*/
	else if (angleDifference > 0.2 &&
		distance > 70 && sdistance > 70)
	{
		setCmdName("reserre");
		setSpeed(getAngleFromHandsToCenter(rh, srh));
		setAmplitude((distance + sdistance) /2.0);
		controlPosIteration = 0;
	}
	else if (angleDifference < -0.2 &&
		distance > 70 && sdistance > 70)
	{
		setCmdName("ecarte");
		setSpeed(getAngleFromHandsToCenter(rh, srh));
		setAmplitude((distance + sdistance) /2.0);
		controlPosIteration = 0;
	}
	else if (angleDifference < 0.01 && angleDifference > -0.01  &&
		gotSimilarHeight(rh, srh, thresholdControlPosition) &&
		controlPosIteration > 10)
	{
		Point3D last = getLastPosition(rh);
		Point3D lasts = getLastPosition(srh);
		setCmdName("ctrlPos");
								
		setSpeed(_hauteurCamera - ((last.getZ() + lasts.getZ())/2.0));
		setAmplitude(0.0);
		controlPosIteration++;
		controlPosLastTimestamp = _timestamp;
	}
	else if (angleDifference < 0.01 && angleDifference > -0.01  &&
		gotSimilarHeight(rh, srh, thresholdControlPosition))
	{
		controlPosIteration++;
		controlPosLastTimestamp = _timestamp;
	}

	if(controlPosLastTimestamp + 3000 < _timestamp)
		controlPosIteration = 0;
	return getCmdName().compare("") == 0;
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
	return true;
}

bool CmdGlobObserver::oneHandRecognition(HOrientedPoint3D* rh)
{
	/*Point3D dir = historicDirection(rh);
	Point3D speed = historicSpeed(rh);
	if(speed.getZ() < 5 && speed.getZ() > -5 &&
		getAngleFromOneHandToCenter(rh) > 30.0)
	{
		Point3D last = getLastPosition(rh);
		setCmdName("droite");
		setSpeed(0.0);
		setAmplitude(0.0);
		return true;
	}
	else if(speed.getZ() < 5 && speed.getZ() > -5 &&
		getAngleFromOneHandToCenter(rh) < -30.0)
	{
		Point3D last = getLastPosition(rh);
		setCmdName("gauche");
		setSpeed(0.0);
		setAmplitude(0.0);
		return true;
	}*/
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
				/*	if(oneHandRecognition(rh))
				break;*/
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
							if(srh->getHistoric().size() < 3 || srh->getLastTimestamp() != _timestamp || !notSameLevel(rh, srh))
								continue;
							if(recognition(rh, srh))
								break;

						}		
					}		
				}		
			}		
		}		
	}		

	return true;
}