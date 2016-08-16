#include "OneDollarRecognizerObserver.h"

#undef min
#undef max

#define MAX_DOUBLE std::numeric_limits<double>::max();

OneDollarRecognizerObserver::OneDollarRecognizerObserver(string name) : Observer(name)
{
    _run = false;

    numPointsInGesture = 128;
    squareSize = 250;
    halfDiagonal = 0.5 * sqrt((250.0 * 250.0) + (250.0 * 250.0));
    anglePrecision = 2.0;
    goldenRatio = 0.5 * (-1.0 + sqrt(5.0));
    loadTemplates();
	angleRange = 0;
}

OneDollarRecognizerObserver::~OneDollarRecognizerObserver()
{

}

void OneDollarRecognizerObserver::loadTemplates()
{
    SampleGestures samples;

    addTemplate("Circle", samples.getGestureCircle());
    addTemplate("Circle_Inv",samples.getGestureCircleInv());
    //addTemplate("X",samples.getGestureX());
    //addTemplate("X",samples.getGestureXInv());
    //addTemplate("Stroke", samples.getGestureStroke());
    //addTemplate("Stroke", samples.getGesturePerfectStroke());

}

int OneDollarRecognizerObserver::addTemplate(string name, Path2D points)
{
    points = normalizePath(points);
    _templates.push_back(GestureTemplate(name, points));
    int numInstancesOfGesture = 0;

    return numInstancesOfGesture;
}

Node* OneDollarRecognizerObserver::clone(string cloneName) const
{
    return new OneDollarRecognizerObserver(cloneName);
}

bool OneDollarRecognizerObserver::start()
{
    if(!_environment)
    {
        if(_environment->getVerboseLevel() != LG_ENV_VERBOSE_MUTE)
            cout << "OneDollarRecognizerObserver Error : Parent environment has not be set." << endl;
        return false;
    }

    if(_environment->getVerboseLevel() != LG_ENV_VERBOSE_MUTE)
        cout << "Start OneDollarRecognizerObserver" << endl;

    loadTemplates();
    cout << "start" << endl;
    return true;
}

bool OneDollarRecognizerObserver::stop()
{
    if(_environment->getVerboseLevel() != LG_ENV_VERBOSE_MUTE)
        cout << "Stop MouseControlObserver" << endl;
    return true;
}

bool OneDollarRecognizerObserver::observe(map<string, Group3D *> groups3D, map<string, Group2D *> groups2D, map<string, Group1D *> groups1D, map<string, GroupSwitch *> groupsSwitch)
{
    for(map<string,Group3D*>::iterator mit = groups3D.begin();mit != groups3D.end();mit++)
    {
		if(_numPointsInGestures.find(mit->first) == _numPointsInGestures.end())
			_numPointsInGestures[mit->first] = MAX_LENGTH_OF_PATH;
		
		set<HOrientedPoint3D*> hops = mit->second->getElementsByType(LG_ORIENTEDPOINT3D_RIGHT_HAND);
		if(hops.size() < 1) continue;
        HOrientedPoint3D* hop = *hops.begin();
		OrientedPoint3D* op = hop->getLast();
		if(!op) continue;
		Point3D p3d = op->getPosition();

		cout << p3d.getX() << endl;

		bool found = false;
		int i = 0;
		while((!found)&&(i<((int)_paths[mit->first].size() - MIN_NB_BEFORE_RECOG))){
			//cout << i << " ; " << _paths[mit->first].size() << " ; " << MIN_NB_BEFORE_RECOG << " ; " <<(i<((int)_paths[mit->first].size() - MIN_NB_BEFORE_RECOG)) << endl;
			found = (Point2D(p3d.getX(),p3d.getY()).distanceTo(*(_paths[mit->first].begin() + i)) < DISTANCE_FOR_RECOGNIZE);
			i++;
		}

		if(found) //_numPointsInGestures[mit->first] = _paths[mit->first].size() - i;
			for(int c = 0; c<i;c++) _paths[mit->first].erase(_paths[mit->first].begin());

		//  ajout du point seulement quand il est loin du precedent DISTANCE_FOR_ADD
		if(_paths[mit->first].size() ==0)
			_paths[mit->first].push_back(Point2D(p3d.getX(),p3d.getY()));
		else
		{
			if(Point2D(p3d.getX(),p3d.getY()).distanceTo(*(_paths[mit->first].begin() + _paths[mit->first].size() -1)) > DISTANCE_FOR_ADD)
				_paths[mit->first].push_back(Point2D(p3d.getX(),p3d.getY()));
		}
		if(_paths[mit->first].size() > MAX_LENGTH_OF_PATH) _paths[mit->first].erase(_paths[mit->first].begin());

		//cout << "Recognize" << endl;
		//if(temp.size() >= MIN_LENGTH_OF_PATH){
		if(_paths[mit->first].size() >= MIN_LENGTH_OF_PATH){
			RecognitionResult rr = recognize(_paths[mit->first]);
			updateProbability(mit->first,rr.score);
			//if(rr.score >0.8) cout << "Speed : " << speed << endl;
			//cout << rr.name << " , " << rr.score << " ; " << l << endl;
		}
	}
    return true;
}

set<string> OneDollarRecognizerObserver::need() const
{
    set<string> needed;
	needed.insert(LG_ORIENTEDPOINT3D_RIGHT_HAND);
    return needed;

}

RecognitionResult OneDollarRecognizerObserver::recognize(Path2D points)
{
    if (_templates.empty())
    {
        std::cout << "No templates loaded so no symbols to match." << std::endl;
        return RecognitionResult("Unknown", NULL);
    }

    points = normalizePath(points);
    double bestDistance = MAX_DOUBLE;
    int indexOfBestMatch = -1;

    for (int i = 0; i < (int)_templates.size(); i++)
    {
        double distance = distanceAtBestAngle(points, _templates[i]);
        if (distance < bestDistance)
        {
            bestDistance     = distance;
            indexOfBestMatch = i;
        }
    }

    double score = 1.0 - (bestDistance / halfDiagonal);

    if (-1 == indexOfBestMatch)
    {
        cout << "Couldn't find a good match." << endl;
        return RecognitionResult("Unknown", 1);
    }

    RecognitionResult bestMatch(_templates[indexOfBestMatch].name, score);
    return bestMatch;
}


Path2D OneDollarRecognizerObserver::normalizePath(Path2D points)
{
    points = resample(points);

    if (getRotationInvariance())
        points = rotateToZero(points);

    points = scaleToSquare(points);
    points = translateToOrigin(points);

    return points;
}

Path2D OneDollarRecognizerObserver::resample(Path2D points)
{
    double interval = pathLength(points) / (numPointsInGesture - 1); // interval length
    double D = 0.0;
    Path2D newPoints;

    //--- Store first point since we'll never resample it out of existence
    newPoints.push_back(points.front());

    for(int i = 1; i < (int)points.size(); i++)
    {
        Point2D currentPoint  = points[i];
        Point2D previousPoint = points[i-1];

        double d = getDistance(previousPoint, currentPoint);
        if ((D + d) >= interval)
        {
            double qx = previousPoint.getX() + ((interval - D) / d) * (currentPoint.getX() - previousPoint.getX());
            double qy = previousPoint.getY() + ((interval - D) / d) * (currentPoint.getY() - previousPoint.getY());
            Point2D point(qx, qy);

            newPoints.push_back(point);
            points.insert(points.begin() + i, point);
            D = 0.0;
        }
        else D += d;
    }

    // somtimes we fall a rounding-error short of adding the last point, so add it if so
    if (newPoints.size() == (numPointsInGesture - 1))
    {
        newPoints.push_back(points.back());
    }



    return newPoints;
}

double OneDollarRecognizerObserver::distanceAtBestAngle(Path2D points, GestureTemplate aTemplate)
{
    double startRange = -angleRange;
    double endRange   =  angleRange;
    double x1 = goldenRatio * startRange + (1.0 - goldenRatio) * endRange;
    double f1 = distanceAtAngle(points, aTemplate, x1);
    double x2 = (1.0 - goldenRatio) * startRange + goldenRatio * endRange;
    double f2 = distanceAtAngle(points, aTemplate, x2);
    while (abs(endRange - startRange) > anglePrecision)
    {
        if (f1 < f2)
        {
            endRange = x2;
            x2 = x1;
            f2 = f1;
            x1 = goldenRatio * startRange + (1.0 - goldenRatio) * endRange;
            f1 = distanceAtAngle(points, aTemplate, x1);
        }
        else
        {
            startRange = x1;
            x1 = x2;
            f1 = f2;
            x2 = (1.0 - goldenRatio) * startRange + goldenRatio * endRange;
            f2 = distanceAtAngle(points, aTemplate, x2);
        }
    }
    return min(f1, f2);
}

Path2D OneDollarRecognizerObserver::rotateToZero(Path2D points)
{
    Point2D c = centroid(points);
    double rotation = atan2(c.getY() - points[0].getY(), c.getX() - points[0].getX());
    return rotateBy(points, -rotation);
}

Path2D OneDollarRecognizerObserver::scaleToSquare(Path2D points)
{
    MyRectangle box = boundingBox(points);
    Path2D newPoints;
    for (Path2DIterator i = points.begin(); i != points.end(); i++)
    {
        Point2D point = *i;

        double scaledX = point.getX() * (this->squareSize / box.width);
        double scaledY = point.getY() * (this->squareSize / box.height);

        newPoints.push_back(Point2D(scaledX, scaledY));
    }
    return newPoints;
}

Path2D OneDollarRecognizerObserver::translateToOrigin(Path2D points)
{
    Point2D c = centroid(points);
    Path2D newPoints;
    for (Path2DIterator i = points.begin(); i != points.end(); i++)
    {
        Point2D point = *i;
        double qx = point.getX() - c.getX();
        double qy = point.getY() - c.getY();
        newPoints.push_back(Point2D(qx, qy));
    }
    return newPoints;
}

double OneDollarRecognizerObserver::pathLength(Path2D points)
{
    double distance = 0;
    for (int i = 1; i < (int)points.size(); i++)
        distance += getDistance(points[i - 1], points[i]);
    return distance;
}

double OneDollarRecognizerObserver::getDistance(Point2D p1, Point2D p2)
{
    double dx = p2.getX() - p1.getX();
    double dy = p2.getY() - p1.getY();
    double distance = sqrt((dx * dx) + (dy * dy));
    return distance;
}

double OneDollarRecognizerObserver::distanceAtAngle(
        Path2D points, GestureTemplate aTemplate, double rotation)
{
    Path2D newPoints = rotateBy(points, rotation);
    return pathDistance(newPoints, aTemplate.points);
}

Point2D OneDollarRecognizerObserver::centroid(Path2D points)
{
    double x = 0.0, y = 0.0;
    for (Path2DIterator i = points.begin(); i != points.end(); i++)
    {
        Point2D point = *i;
        x += point.getX();
        y += point.getY();
    }
    x /= points.size();
    y /= points.size();
    return Point2D(x, y);
}

Path2D OneDollarRecognizerObserver::rotateBy(Path2D points, double rotation)
{
    Point2D c     = centroid(points);
    double cosine = cos(rotation);
    double sine   = sin(rotation);

    Path2D newPoints;
    for (Path2DIterator i = points.begin(); i != points.end(); i++)
    {
        Point2D point = *i;
        double qx = (point.getX() - c.getX()) * cosine - (point.getY() - c.getY()) * sine   + c.getX();
        double qy = (point.getX() - c.getX()) * sine   + (point.getY() - c.getY()) * cosine + c.getY();
        newPoints.push_back(Point2D(qx, qy));
    }
    return newPoints;
}

double OneDollarRecognizerObserver::pathDistance(Path2D pts1, Path2D pts2)
{
    // assumes pts1.size == pts2.size

    //   cout <<"pts1 : " << pts1.size() << endl;
    //   cout <<"pts2 : " << pts2.size() << endl;

    double distance = 0.0;
    for (int i = 0; i < (int)pts1.size(); i++)
        distance += getDistance(pts1[i], pts2[i]);
    return (distance / pts1.size());
}

MyRectangle OneDollarRecognizerObserver::boundingBox(Path2D points)
{
    double minX =  MAX_DOUBLE;
    double maxX = -MAX_DOUBLE;
    double minY =  MAX_DOUBLE;
    double maxY = -MAX_DOUBLE;

    for (Path2DIterator i = points.begin(); i != points.end(); i++)
    {
        Point2D point = *i;
        if (point.getX() < minX)
            minX = point.getX();
        if (point.getX() > maxX)
            maxX = point.getX();
        if (point.getY() < minY)
            minY = point.getY();
        if (point.getY() > maxY)
            maxY = point.getY();
    }
    MyRectangle bounds(minX, minY, (maxX - minX), (maxY - minY));
    return bounds;
}
